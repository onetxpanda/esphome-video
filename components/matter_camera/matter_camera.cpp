#include "matter_camera.h"

#include "esphome/components/network/util.h"
#include "esphome/core/log.h"

#include <esp_matter.h>
#include <app/server/Server.h>
#include <data_model_provider/clusters/camera_av_stream_management/integration.h>
#include <data_model_provider/clusters/webrtc_transport_provider/integration.h>

#include "app_webrtc.h"
#include "esp_work_queue.h"
#include "kvs_peer_connection.h"
#include "media_stream.h"

#include "camera-app.h"
#include "camera-device.h"
#include "matter_signaling.h"

// From media_stream's video sink registry (video_sink_priv.h): deliver one
// encoded frame to every enabled sink, and whether any sink is enabled.
extern "C" void video_sink_dispatch(const video_frame_t *frame);
extern "C" bool video_sink_any_enabled(void);

// Globals the camera example's cluster and WebRTC code refers to.
Camera::CameraDevice gCameraDevice;
uint16_t camera_endpoint_id = 0;
// Standby: when set, sessions are answered with a deferred offer.
bool deferred_offer = false;
extern std::unique_ptr<CameraApp> gCameraApp;

namespace esphome {
namespace matter_camera {

static const char *const TAG = "matter_camera";

MatterCamera *MatterCamera::instance_ = nullptr;

// WebRTC's work queue runs the peer-connection setup, which needs a deep stack.
static constexpr uint32_t WORK_QUEUE_STACK = 48 * 1024;

MatterCamera::MatterCamera() { instance_ = this; }

// ---- Video source for the WebRTC media layer ------------------------------
// Frames reach WebRTC through the video sink registry (on_encoded_frame); the
// capture interface only tells the media layer a source exists.

static esp_err_t capture_init(video_capture_config_t *config, video_capture_handle_t *handle) {
  (void) config;
  *handle = reinterpret_cast<video_capture_handle_t>(1);
  return ESP_OK;
}
static esp_err_t capture_start(video_capture_handle_t handle) {
  (void) handle;
  if (MatterCamera::instance() != nullptr && MatterCamera::instance()->encoder() != nullptr)
    MatterCamera::instance()->encoder()->request_keyframe();
  return ESP_OK;
}
static esp_err_t capture_noop(video_capture_handle_t handle) {
  (void) handle;
  return ESP_OK;
}
static esp_err_t capture_get_frame(video_capture_handle_t handle, video_frame_t **frame, uint32_t timeout_ms) {
  (void) handle;
  (void) frame;
  (void) timeout_ms;
  return ESP_ERR_NOT_SUPPORTED;
}
static esp_err_t capture_release_frame(video_capture_handle_t handle, video_frame_t *frame) {
  (void) handle;
  (void) frame;
  return ESP_OK;
}

static media_stream_video_capture_t *capture_interface() {
  static media_stream_video_capture_t iface{};
  iface.init = capture_init;
  iface.start = capture_start;
  iface.stop = capture_noop;
  iface.get_frame = capture_get_frame;
  iface.release_frame = capture_release_frame;
  iface.deinit = capture_noop;
  iface.set_bitrate = nullptr;  // the encoder's bitrate is set in its config
  iface.get_bitrate = nullptr;
  return &iface;
}

}  // namespace matter_camera
}  // namespace esphome

// The media layer asks the capture side for keyframes and the stream size.
extern "C" esp_err_t video_capture_request_keyframe(void) {
  auto *cam = esphome::matter_camera::MatterCamera::instance();
  if (cam == nullptr || cam->encoder() == nullptr)
    return ESP_ERR_INVALID_STATE;
  cam->encoder()->request_keyframe();
  return ESP_OK;
}

extern "C" esp_err_t video_capture_get_active_resolution(video_resolution_t *resolution) {
  auto *cam = esphome::matter_camera::MatterCamera::instance();
  if (resolution == nullptr || cam == nullptr || cam->encoder() == nullptr)
    return ESP_ERR_INVALID_ARG;
  const uint16_t w = cam->encoder()->get_encoded_width();
  const uint16_t h = cam->encoder()->get_encoded_height();
  if (w == 0 || h == 0)
    return ESP_ERR_NOT_SUPPORTED;
  resolution->width = w;
  resolution->height = h;
  resolution->fps = 0;
  return ESP_OK;
}

namespace esphome {
namespace matter_camera {

static void webrtc_event_handler(app_webrtc_event_data_t *event, void *user_ctx) {
  (void) user_ctx;
  if (event == nullptr)
    return;
  switch (event->event_id) {
    case APP_WEBRTC_EVENT_PEER_CONNECTED:
      ESP_LOGI(TAG, "Viewer connected: %s", event->peer_id);
      break;
    case APP_WEBRTC_EVENT_PEER_DISCONNECTED:
      ESP_LOGI(TAG, "Viewer disconnected: %s", event->peer_id);
      break;
    case APP_WEBRTC_EVENT_STREAMING_STARTED:
      ESP_LOGI(TAG, "Streaming to %s", event->peer_id);
      break;
    case APP_WEBRTC_EVENT_STREAMING_STOPPED:
      ESP_LOGI(TAG, "Stopped streaming to %s", event->peer_id);
      break;
    case APP_WEBRTC_EVENT_ERROR:
    case APP_WEBRTC_EVENT_SIGNALING_ERROR:
    case APP_WEBRTC_EVENT_PEER_CONNECTION_FAILED:
      ESP_LOGW(TAG, "WebRTC error (event %d, code %d): %s", static_cast<int>(event->event_id),
               static_cast<int>(event->status_code), event->message);
      break;
    default:
      ESP_LOGD(TAG, "WebRTC event %d", static_cast<int>(event->event_id));
      break;
  }
}

// Brings up the WebRTC application with Matter as its signaling channel. Runs
// on the work queue (deep stack).
static void webrtc_start(void *arg) {
  (void) arg;
  app_webrtc_register_event_callback(webrtc_event_handler, nullptr);

  static matter_signaling_config_t signaling_cfg{};
  signaling_cfg.provider_manager = static_cast<void *>(&gCameraDevice.GetWebRTCProviderDelegate());

  app_webrtc_config_t cfg = APP_WEBRTC_CONFIG_DEFAULT();
  cfg.signaling_client_if = getMatterSignalingClientInterface();
  cfg.signaling_cfg = &signaling_cfg;
  cfg.peer_connection_if = kvs_peer_connection_if_get();
  cfg.video_capture = capture_interface();

  WEBRTC_STATUS status = app_webrtc_init(&cfg);
  if (status != WEBRTC_STATUS_SUCCESS) {
    ESP_LOGE(TAG, "WebRTC init failed: 0x%08" PRIx32, static_cast<uint32_t>(status));
    return;
  }
  status = app_webrtc_run();
  if (status != WEBRTC_STATUS_SUCCESS) {
    ESP_LOGE(TAG, "WebRTC start failed: 0x%08" PRIx32, static_cast<uint32_t>(status));
    app_webrtc_terminate();
    return;
  }
  ESP_LOGI(TAG, "WebRTC ready");
}

// ---- Matter endpoint -------------------------------------------------------

bool MatterCamera::build_endpoint(esp_matter::endpoint_t *endpoint) {
  using namespace chip::app::Clusters;
  MatterCamera *self = instance_;
  if (self == nullptr)
    return false;

  // The stream management cluster needs its features set before it is
  // created (it aborts otherwise). Video, Audio and Snapshot as in Espressif's
  // camera example; no audio is sent and snapshot requests fail.
  gCameraDevice.Init();
  esp_matter::endpoint::camera::config_t cam_config;
  namespace avsm = esp_matter::cluster::camera_av_stream_management::feature;
  cam_config.camera_av_stream_management.feature_flags =
      avsm::audio::get_id() | avsm::video::get_id() | avsm::snapshot::get_id();
  if (esp_matter::endpoint::camera::add(endpoint, &cam_config) != ESP_OK) {
    ESP_LOGE(TAG, "Could not add the camera device type");
    return false;
  }
  camera_endpoint_id = esp_matter::endpoint::get_id(endpoint);

  static CameraAvStreamManagement::CameraAvStreamManagementConfig avsm_config;
  CameraAppInit(&gCameraDevice, avsm_config);
  CameraAvStreamManagement::SetConfig(camera_endpoint_id, avsm_config);
  CameraAvStreamManagement::SetDelegate(camera_endpoint_id, &gCameraDevice.GetCameraAVStreamMgmtDelegate());
  WebRTCTransportProvider::SetDelegate(camera_endpoint_id, &gCameraDevice.GetWebRTCProviderDelegate());
  return true;
}

// Runs after esp_matter::start(), from the matter component's setup.
class StartedHook : public matter::MatterEndpointMappingBase {
 public:
  StartedHook(MatterCamera *camera, uint16_t endpoint_id)
      : MatterEndpointMappingBase(endpoint_id), camera_(camera) {}
  void initialize() override { this->camera_->on_matter_started(); }

 protected:
  MatterCamera *camera_;
};

void MatterCamera::on_matter_started() {
  using namespace chip::app::Clusters;
  esp_matter::lock::ScopedChipStackLock lock(portMAX_DELAY);
  CHIP_ERROR err = gCameraApp->InitializeCameraAVStreamMgmt();
  if (err != CHIP_NO_ERROR) {
    ESP_LOGE(TAG, "Camera AV stream management init failed: %" CHIP_ERROR_FORMAT, err.Format());
    this->mark_failed();
    return;
  }
  auto *server = WebRTCTransportProvider::GetServer(camera_endpoint_id);
  if (server == nullptr) {
    ESP_LOGE(TAG, "No WebRTC transport provider on endpoint %u", camera_endpoint_id);
    this->mark_failed();
    return;
  }
  gCameraDevice.SetWebRTCTransportProvider(server);
  this->matter_started_ = true;
}

// ---- Component -------------------------------------------------------------

void MatterCamera::set_matter(matter::MatterComponent *matter) {
  this->matter_ = matter;
  matter->register_mapping(new StartedHook(this, this->endpoint_id_));  // NOLINT
}

void MatterCamera::setup() {
  esp_work_queue_config_t wq = ESP_WORK_QUEUE_CONFIG_DEFAULT();
  wq.stack_size = WORK_QUEUE_STACK;
  if (esp_work_queue_init_with_config(&wq) != ESP_OK || esp_work_queue_start() != ESP_OK) {
    ESP_LOGE(TAG, "Could not start the WebRTC work queue");
    this->mark_failed();
    return;
  }
  if (this->encoder_ != nullptr)
    this->encoder_->add_encoded_sink(this);
}

void MatterCamera::loop() {
  // WebRTC binds its sockets to the device's addresses, so it starts once the
  // network is up and Matter is running.
  if (this->webrtc_started_ || !this->matter_started_ || !network::is_connected())
    return;
  if (esp_work_queue_add_task(webrtc_start, nullptr) == ESP_OK)
    this->webrtc_started_ = true;
}

void MatterCamera::dump_config() {
  ESP_LOGCONFIG(TAG,
                "Matter camera:\n"
                "  Endpoint: %u\n"
                "  Frames sent: %" PRIu32,
                this->endpoint_id_, this->frames_sent_.load());
}

void MatterCamera::on_encoded_frame(const uint8_t *data, size_t length, uint32_t timestamp_ms, bool keyframe) {
  video_frame_t frame{};
  frame.buffer = const_cast<uint8_t *>(data);
  frame.len = static_cast<uint32_t>(length);
  frame.timestamp = static_cast<uint64_t>(timestamp_ms) * 1000ULL;
  frame.type = keyframe ? VIDEO_FRAME_TYPE_I : VIDEO_FRAME_TYPE_P;
  video_sink_dispatch(&frame);
  this->frames_sent_++;
}

bool MatterCamera::wants_frames() const { return video_sink_any_enabled(); }

}  // namespace matter_camera
}  // namespace esphome
