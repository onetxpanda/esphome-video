# esphome-video

ESPHome external components for building a video pipeline on an ESP32-P4:
a sensor produces frames, processing stages transform them, and outputs
encode or serve them. Encoding uses the P4's hardware H.264 and JPEG
encoders; there is no software encoding.

| Component | Role |
|---|---|
| `mlx90640` | MLX90640 32x24 thermal sensor. Radiometric source (temperatures). |
| `seek_compact_pro` | Seek Thermal Compact Pro USB thermal camera (Android model, Fast Frame too), 320x240. Radiometric source: temperatures from the camera's own calibration, processed as Seek's app does, correctable. |
| `thermal_master_p1` | Thermal Master P1 USB thermal camera, 160x120 at 25 fps. Radiometric source (the camera's own temperatures, correctable; both ranges, and the vendor app's automatic switching) and image source (the camera's own processed picture); its command set, as the vendor app's library has it, as entities. |
| `hti301` | HTI-301 USB thermal camera, 384x288 at 25 fps. Radiometric source in radiometric mode; image source (the camera's own processed image) in processed mode. |
| `radiometric` | Temperature processing: invalid-pixel repair, outlier filter, motion-adaptive temporal filter, edge-preserving smoothing, median filter, statistics sensors. |
| `isp` | Radiometric frames to images: tone mapping onto a palette, mirroring, upscaling (CPU or the P4's PPA), output format; spot meters placed at run time, each corrected for its material's emissivity. |
| `overlay` | Draws a frame's annotations (min/max/centre markers and spots, with values) into the image. |
| `h264_encoder` | ESP32-P4 hardware H.264 encoder, with a `select` platform to switch between sources. |
| `rtsp_server` | Serves the encoder's output over RTSP (RTP over TCP). |
| `matter` | Matter device support (from [esphome-matter](https://github.com/DavidvtWout/esphome-matter), Apache 2.0). |
| `matter_camera` | A Matter camera: controllers such as Home Assistant stream the encoder's output live over WebRTC. |
| `video_camera` | ESPHome camera entity (Home Assistant, web server), JPEG from the P4's hardware encoder. |
| `video_test_pattern` | Synthetic image source for checking encoders and streams. |
| `video_palette` | One select entity choosing the palette of several ISPs and cameras at once. |
| `video_switch` | Chooses which of several image sources reaches the consumers after it (encoder, camera), with a select entity. |
| `assert_capture` | Records why the firmware aborted (failed assert with file and line, `abort()` caller, task stack overflow) and reports it after the reboot. |
| `task_stats` | Logs each FreeRTOS task's share of the CPU and each core's idle time; optional CPU load sensor. |
| `video_core`, `video_controls` | Shared frame types and runtime option entities; loaded automatically. |

## Usage

```yaml
external_components:
  - source: github://onetxpanda/esphome-video@<tag>
    # video_core and video_controls are used by the others and must be listed too.
    components: [video_core, video_controls, mlx90640, radiometric, isp, overlay, video_camera, h264_encoder, rtsp_server]

psram:
  mode: hex

i2c:
  sda: GPIO7
  scl: GPIO8
  frequency: 1MHz        # the MLX90640 supports up to 1 MHz

mlx90640:
  id: thermal_sensor
  refresh_rate: 32Hz     # sub-page rate; a full frame is two sub-pages
  frame_mode: merge      # or interpolate (a frame per sub-page)

radiometric:
  id: thermal
  source: thermal_sensor
  median_filter: true
  max:
    name: "Thermal Max"

isp:
  id: thermal_image
  source: thermal
  scale: 20              # 32x24 -> 640x480
  range_min: 15          # degrees mapped to the first and last palette colour
  range_max: 40
  upscale: bilinear

overlay:
  id: thermal_marked
  source: thermal_image

video_camera:
  name: "Thermal Camera"
  source: thermal_marked

h264_encoder:
  id: encoder
  source: thermal_marked

rtsp_server:
  encoder: encoder
  path: thermal          # rtsp://<device>:554/thermal
```

`tests/esphome/p4-thermal-rtsp.yaml` is a complete configuration with every
option entity.

### Several cameras

A `video_switch` gives the encoder and the camera entity the same choice of
source, with one select. An entry may list several sources: frames from any
of them pass while it is chosen.

```yaml
video_switch:
  id: stream_switch
  sources:
    - name: "MLX90640"
      source: thermal_marked
    - name: "HTI-301"
      source: [hti_marked, hti_camera]   # radiometric chain, or the camera's own image
  select:
    name: "Stream Source"

video_camera:
  name: "Thermal Camera"
  source: stream_switch

h264_encoder:
  id: encoder
  source: stream_switch
```

### HTI-301

The HTI-301 connects to the P4's USB 2.0 high-speed port and needs ESPHome's
`usb_host`. In place of `mlx90640` and `i2c` above:

```yaml
usb_host:

hti301:
  id: thermal_sensor
  mode: radiometric      # or processed: the camera's own image, as an image source
  range: normal          # -20..120 °C; high: -20..450 °C
  controls:
    mode:
      name: "Thermal Mode"
    calibrate:
      name: "Thermal Calibrate"
  streaming:
    name: "Thermal Streaming"
```

In radiometric mode the camera feeds `radiometric` as above. In processed
mode it produces images itself, so an encoder or `video_camera` can take it
as a `source:` directly. To switch between the two at runtime, give
`h264_encoder` both as sources and use its select (see
`tests/esphome/p4-hti301.yaml`).

### Seek Compact Pro

The Android Compact Pro (USB-C or micro-USB; the iPhone model's Lightning
connector cannot work) connects to the P4's USB 2.0 high-speed port and
needs ESPHome's `usb_host`. In place of `mlx90640` and `i2c` above:

```yaml
usb_host:

seek_compact_pro:
  id: thermal_sensor
  controls:
    gain:
      name: "Thermal Gain"
    offset:
      name: "Thermal Offset"
    shutter:
      name: "Thermal Shutter"
  streaming:
    name: "Thermal Streaming"
```

It feeds `radiometric` as the MLX90640 does; set `outlier_level: 0` there,
since the camera's frames are already corrected. Its temperatures come from
the camera's own calibration, as Seek's app computes them (see How it works);
to match a reference, set `gain` and `offset`. Its picture is not mirrored:
leave the `isp` without `mirror_x`.

### Thermal Master P1

The P1 (USB-C) connects to the P4's USB 2.0 high-speed port and needs
ESPHome's `usb_host`. In place of `mlx90640` and `i2c` above:

```yaml
usb_host:

thermal_master_p1:
  id: thermal_sensor
  controls:
    gain:
      name: "Thermal Gain"
    offset:
      name: "Thermal Offset"
    shutter:
      name: "Thermal Shutter"
  streaming:
    name: "Thermal Streaming"
  status:
    name: "Thermal Status"
```

It feeds `radiometric` as the MLX90640 does; set `outlier_level: 0` there,
since the camera's frames are already corrected. The temperatures are the
camera's own; to match a reference, set `gain` and `offset`. The camera has
two ranges: high, -4 to 302 °F (the default), and low, 302 to 1112 °F. In
the low range everything under 302 °F reads 302 °F, as the vendor app shows
it. `settings: {range: auto}` (or the `range` select) switches between them
the way the vendor app's "Auto" does. The camera does a shutter cycle by
itself about every 90 s; the `shutter` button does one now. `scale: 4` on
the `isp` gives 640x480.

It is also an image source: the camera's own processed picture, which its
palette, brightness, contrast, noise reduction and the other picture
settings change. Give it to `video_camera` or an encoder as a `source:`.

The camera's settings can be set at every start (`settings:`) and as
entities (`controls:`). An entity shows what the camera reports after a
change (read back where the camera has a read), and a change is kept for
later starts. As the vendor app does at every start, auto shutter and sun
detection are switched on unless `settings:` says otherwise. With sun
detection on, `sun_detected` turns on when the camera flags the sun in view
(the vendor app then closes the shutter: `shutter_open` does that here).
Readings (`device_temperature`, `device_status`, `camera_max_temperature`
and the others below) are polled every `update_interval`; the device
information is read once a start. Commands that change or wipe what the
camera keeps are entities only when listed under `maintenance:`. See
`tests/esphome/p4-thermal-master.yaml` for every one. Not yet run on a
camera: the commands are the vendor app's library's, and most of them the
app itself never sends to a P1.

### Spot meters

An `isp` can show up to 8 spots: reticles with the temperature under them,
placed and removed while it runs and kept across reboots. Each spot's
reticle and value are drawn in its own colour (cyan, green, magenta, blue,
yellow, orange, pink, violet: the first free one when it is placed, kept
while it exists). Each spot has a material, and its value is corrected for
that material's emissivity. A Home Assistant dashboard card places them:
click the image to add a spot, click a spot to remove it, and pick each
spot's material in the list below the image, which shows each spot's
colour.

On the device, give the `isp` a `spots` text sensor and add these API actions
(the card calls them by these names):

```yaml
isp:
  - id: thermal_isp
    # ...
    spots:
      name: "Thermal Spots"
    controls:
      clear_spots:
        name: "Thermal Clear Spots"

api:
  actions:
    - action: thermal_spot          # a click: removes the spot there, or adds one
      variables: {x: float, y: float, material: string}
      then:
        - lambda: 'id(thermal_isp)->toggle_spot(x, y, material);'
    - action: thermal_spot_material
      variables: {spot: int, material: string}
      then:
        - lambda: 'id(thermal_isp)->set_spot_material(spot, material);'
    - action: thermal_remove_spot
      variables: {spot: int}
      then:
        - lambda: 'id(thermal_isp)->remove_spot(spot);'
    - action: thermal_clear_spots
      then:
        - lambda: 'id(thermal_isp)->clear_spots();'
```

`x` and `y` are percent of the image as shown, from the top left; spots are
numbered from 1 in the order placed. Materials (by key or label):

| Key | Material | Emissivity |
|---|---|---|
| `default` | Default: as the source computed it | — |
| `tortoise_shell` | Tortoise shell | 0.96 |
| `skin` | Skin | 0.98 |
| `black_plastic` | Black plastic | 0.95 |
| `plastic` | Plastic | 0.92 |
| `rubber` | Rubber | 0.95 |
| `wood` | Wood | 0.90 |
| `paper` | Paper, cardboard | 0.93 |
| `fabric` | Fabric | 0.95 |
| `water` | Water | 0.96 |
| `glass` | Glass | 0.92 |
| `ceramic` | Ceramic tile | 0.92 |
| `concrete` | Concrete | 0.94 |
| `brick` | Brick | 0.93 |
| `soil` | Soil | 0.94 |
| `sand` | Sand | 0.90 |
| `plants` | Plants | 0.96 |
| `paint` | Painted surface | 0.93 |
| `anodized` | Aluminum, anodized | 0.77 |
| `aluminum` | Aluminum, bare | 0.10 |
| `stainless_steel` | Stainless steel | 0.16 |

These are typical long-wave infrared values; a real surface varies with its
finish and the viewing angle. Bare metals reflect most of what they see, so
their readings follow the surroundings more than the metal.

The card is `ha/thermal-spot-card.js`. Add it to Home Assistant as a
dashboard resource (JavaScript module), then add a card:

```yaml
type: custom:thermal-spot-card
camera: camera.esp32_p4_dev_kit_thermal_camera
spots: sensor.esp32_p4_dev_kit_thermal_spots
device: esp32_p4_devkit   # the ESPHome node name, - written as _
```

It shows the camera live (paused while the page is hidden) and remembers
the material for new spots per browser.

### Matter camera

`matter_camera` makes the device a Matter camera (device type 0x0142).
Controllers that support Matter cameras (Home Assistant 2026.11 and later
with its Matter Server, Apple Home, ...) show it with live video, streamed
over WebRTC from the same encoder that feeds `rtsp_server`. The encoder runs
while either has a viewer.

```yaml
network:
  enable_ipv6: true      # Matter needs IPv6

wifi:                    # Matter here runs over Wi-Fi (not Ethernet)
  ssid: !secret wifi_ssid
  password: !secret wifi_password

matter:

matter_camera:
  encoder: encoder       # an h264_encoder
  endpoint: 1            # default

text_sensor:
  - platform: matter
    manual_pairing_code:
      name: "Matter Pairing Code"
    qr_code:
      name: "Matter QR Code"
```

To add the camera, use the pairing code (or QR code) in the controller, for
example Home Assistant's "Add Matter device". The device needs PSRAM. The
camera has video only: no audio and no snapshots.

### Options

Every stage takes its options in YAML. Most can also be changed at runtime
through Home Assistant entities, declared under `controls:`. Each entity
restores its last value on boot.

`radiometric`, `isp` and `overlay` can each be configured more than once, as
a YAML list, for example one chain per camera.

`mlx90640`, `hti301`, `radiometric`, `isp` and `overlay` take `device_id`: an
ESPHome sub-device (`esphome: devices:`) that all of the component's entities
belong to, unless an entity names its own. Give each camera's chain its own
device and its entities appear under that device in Home Assistant, so two
chains can use the same names:

```yaml
esphome:
  devices:
    - id: hti_device
      name: "HTI-301"
    - id: mlx_device
      name: "MLX90640"

isp:
  - id: hti_isp
    source: hti_rad
    device_id: hti_device
    controls:
      range_min:
        name: "Range Min"      # shown as "HTI-301 Range Min"
  - id: mlx_isp
    source: mlx_rad
    device_id: mlx_device
    controls:
      range_min:
        name: "Range Min"      # shown as "MLX90640 Range Min"
```

- **mlx90640**: `refresh_rate` (0.5Hz–64Hz), `frame_mode` (interpolate, merge),
  `emissivity`. Controls: `refresh_rate`, `frame_mode`.
- **seek_compact_pro**: `gain` (default 1), `offset` (°C, default 0),
  `transfers` (bulk transfers kept queued, default 3), `scene_filter` (Seek's
  default picture filter, default on; see How it works), `bad_columns` (picture
  columns, 0 at the left, filled from their neighbours like dead pixels: a
  column that flickers; up to 8). Controls: `gain`,
  `offset` (in °C: a difference, so Home Assistant does not convert it),
  `shutter` (button). Sensors: `streaming`, `status`.
- **thermal_master_p1**: `gain` (default 1), `offset` (°C, default 0),
  `transfers` (bulk transfers kept queued, 8 KB of internal RAM each, default
  4), `update_interval` (readings, default 60 s), `heartbeat_interval` (the
  library's keepalive; off unless set: the vendor app sends none to a P1),
  `camera_point` (`x`, `y`: the point `camera_point_temperature` reads; the
  centre by default), `settings` (written at every start; the keys below, in
  the entities' units: selects by option key, e.g. `range: auto`; pairs as
  `isothermal_limit: {low: -20, high: 120}` and `sun_detect_pixel_ratio:
  {min: 5, max: 50}`). Controls: `gain`, `offset` (in °C: a difference);
  buttons `shutter` (the offset updated, as the vendor app's button does),
  `shutter_ooc`, `shutter_both`, `ffc_update`, `ffc_without_shutter`; camera
  settings `range` (high, low, auto), `palette` (the camera's palette index;
  the names are the `palettes` text), `brightness`, `contrast`,
  `global_contrast` (the vendor app's contrast), `detail_enhance`,
  `noise_reduction`, `spatial_noise_reduction`, `temporal_noise_reduction`,
  `gamma` (0-100), `edge_enhance` (off, level_1, level_2), `roi_level` (off,
  third, half), `profession_mode` (no read), `mirror_flip` (none, mirror,
  flip, both), `isothermal_mode` (0-5), `isothermal_limit_low`,
  `isothermal_limit_high` (whole °C), `isothermal`, `sun_detect`,
  `sun_detect_pixel_ratio_min`, `_max`, `sun_detect_roundness_min`, `_max`
  (0-100), `overexposure_protection` (off, on, mode_2), `auto_shutter`,
  `auto_shutter_threshold`, `auto_shutter_min_interval`,
  `auto_shutter_max_interval` (units not given), `shutter_open`,
  `env_correction`, `emissivity`, `transmittance` (0.001-1), `ambient`,
  `reflected` (-40 to 100 °C, whole degrees), `stream_mid_mode` (picture,
  tpd, lce, ir, snr), `palette_switch`, `frame_temp_display` (no read).
  Maintenance: buttons `reboot`, `save_settings`, `restore_settings`,
  `reset_to_rom`, `reset_to_bootloader`, `save_defaults`,
  `restore_calibration`, `ooc_save`, `k_calibrate_low`, `k_calibrate_high`,
  `k_calculate`, `k_cancel`, `k_clear`, `k_save`, `k_restore`, `dpc_auto`,
  `dpc_cancel`, `dpc_clear`, `dpc_save`, `dpc_restore`, `rmcover_auto`,
  `rmcover_cancel`, `rmcover_clear`, `rmcover_save`, `rmcover_restore`,
  `tpd1_cancel`, `tpd1_clear`, `tpd1_save`, `tpd_restore`, `tpd2_cancel`,
  `tpd2_clear`, `tpd2_save`, `tpd2_restore`, `crg_save`, `cursor_move`,
  `dpc_mark`, `dpc_unmark` (the pixel at `cursor_x`, `cursor_y`),
  `tpd1_recalibrate`, `tpd2_point1`, `tpd2_point2` (with `tpd_reference`,
  a whole number whose unit the library does not give), `vl_read`,
  `vl_write`, `crg_write`; switches `cursor`, `nuc_bypass`; numbers
  `cursor_x`, `cursor_y`, `tpd_reference`, `vl_register`, `vl_value`,
  `crg_new_value`; sensor `vl_result`. Sensors: `streaming`, `status`,
  `sun_detected`, `shutter_countdown`, `camera_max_temperature`,
  `camera_min_temperature`, `camera_point_temperature`,
  `device_temperature`, `powered_time`, `crg_value`, `sensor_vtemp`; text
  `range_in_use`, `palettes`, `device_status`, `shutter_status`,
  `stream_source`, `digital_video_output`, `system_data_flag`,
  `cursor_position`, and the device information `model`, `firmware`,
  `customer_id`, `vendor_id`, `product_id`, `part_number`, `serial`,
  `isp_version`, `abstract_version`, `system_version`,
  `calibration_version`, `data_version`, `package_version`, `sensor_type`,
  `sensor_id`.
- **hti301**: `mode` (radiometric, processed), `range` (normal, high),
  `palette` (processed mode: white_hot, black_hot, ironbow, lava, rainbow,
  iron_gray, red_hot, rainbow_2), `emissivity` (default 0.95), and
  `reflected_temperature`, `ambient_temperature`, `humidity`, `distance`
  (left out: the values stored in the camera). `auto_calibrate` (default
  true) sends the shutter calibration on a schedule, since the camera never
  calibrates on its own. `transfers` and
  `packets_per_transfer` size the USB buffering (default 8 x 16 packets,
  16 ms). Binary sensor: `streaming` (on while frames arrive). Text sensor:
  `usb_status` (while no camera is connected, the USB devices the host sees
  and the state of each hub port; then the stream's state). Controls:
  `mode`, `range`, `palette`, `emissivity`, a `calibrate` button
  (shutter calibration), and a `restart_usb` button (powers the USB port off
  and on, so everything on it re-enumerates).
- **radiometric**: `temporal_frames` (1–16; 1 = off), `motion_threshold`
  (degrees; default 0.5), `edge_smoothing` (degrees; 0 = off), `median_filter`,
  `outlier_level` (degrees; 0 = off), `valid_min` and `valid_max` (readings
  outside are repaired; default −40..300 °C), `update_interval` for the
  sensors. The filters are meant for the MLX90640's noisy low-resolution
  frames; for a camera that does its own correction (the HTI-301) set
  `outlier_level: 0` and leave the others off. The median statistic is
  computed only when its sensor is configured.
  Sensors: `min`, `max`, `mean`, `median`, `center`. Controls:
  `temporal_frames`, `motion_threshold`, `edge_smoothing`, `median_filter`,
  `outlier_level`.
- **isp**: `scale` (1–20), `format` (RGB565_LE, RGB565_BE, RGB888, GRAY8),
  `palette` (white_hot, black_hot, ironbow, lava, rainbow, iron_gray,
  red_hot, rainbow_2; the order of the HTI-301's own palettes), `range_min`, `range_max`, `mirror_x`, `mirror_y`,
  `upscale` (nearest, bilinear, bicubic), `scaler` (cpu, ppa), `equalize`
  (percent; 0 = linear), `detail` (gain; 0 = off), `spots` (text sensor
  listing the spots as `x,y,material,colour;...`; see Spot meters). Controls:
  `range_min`, `range_max` (temperatures in °C, so Home Assistant shows them
  in its own unit), `mirror_x`, `mirror_y`, `upscale`, `scaler`, `equalize`,
  `detail`, `clear_spots` (button).
- **overlay**: `marks` (min, max, center, spots), `hold` (how often a mark's
  position and value are refreshed; default 1 s; a spot that is moved moves
  at once), `unit` (C, F), `enabled`. Controls: `enabled`, `unit`.
- **h264_encoder**: `source` (list), `fps` (default: measured from the
  source), `gop` (default: one keyframe per second), `bitrate` (default:
  automatic), `qp_min`, `qp_max`. Select platform: switches the active source.
- **rtsp_server**: `port`, `path`, `session_name`, `max_clients`, `buffer_size`.
- **video_camera**: `quality`, `subsampling` (444, 422, 420).
- **video_test_pattern**: `width`, `height`, `fps`.
- **assert_capture**: `last_crash` (text sensor). The record is also logged
  at boot and in the config dump.
- **task_stats**: `update_interval` (default 10 s), `top` (busiest tasks
  listed, default 12), `load` (sensor: CPU in use, % of all cores). Task
  shares are logged as % of one core. It turns on FreeRTOS run-time
  statistics, which read a timer at every task switch.
- **video_palette**: `targets` (ISPs, and cameras that colour their own image
  such as the HTI-301), `palette` (initial). The select restores its last
  value.
- **video_switch**: `sources` (each a `name` and one or more `source`s),
  `select` (the entity choosing the entry; restores its last value).

Several stages can be marked `log_timing: true` (`mlx90640`, `hti301`,
`radiometric`, `isp`). They then log their per-stage times every 10 s.
The `hti301` also logs what its last frame was converted with: the
detector, shutter and core temperatures and calibration words from the
frame, the scene parameters, and the centre pixel's raw value and temperature.

## How it works

### Frames and connections

Stages pass two kinds of frame (`video_core/frame.h`):

- `RadiometricFrame`: one float per pixel, in degrees C.
- `ImageFrame`: pixels in an `ImageFormat`, with a row stride.

Both carry a capture timestamp, a sequence number, and up to four
*annotations*: points of interest with a value, such as the hottest pixel.
A radiometric frame can also carry the statistics its source worked out
while producing it (min and max with their positions, mean), so a later
stage need not read the whole frame again.

A stage is connected to its upstream stage with `source:`. Every source
delivers each frame to its sinks by calling them directly, on the source's
own task. No stage copies a frame to pass it on. Frame buffers come from
fixed pools and are reference counted (`FrameRef`). A consumer that needs a
frame after its call returns keeps a reference, and the buffer goes back to
its pool when the last reference is dropped. If a source finds every buffer
still in use, it drops that frame rather than wait. Stages' whole-frame working
memory (the temporal filter's state, filter scratch, palette indices) is allocated
in PSRAM on the device. If it cannot be allocated, the stage logs an error
and drops frames instead of stopping the device.

### Tasks

- `mlx90640` reads the sensor on its own task. It polls the data-ready flag
  in steps of 1/8 of the sub-page period, reads and calculates each sub-page,
  and assembles frames. `radiometric` runs on the same task; its processing
  takes well under a millisecond per frame. Its sensors are published from
  the main loop every `update_interval`.
- `hti301` receives on the USB host client's task: each isochronous packet's
  payload is copied into the frame being assembled, and a complete frame goes
  to the camera's own processing task. Only the newest frame waits there;
  older ones are dropped. That task converts the frame and runs the sinks
  (`radiometric` in radiometric mode). The main loop sets up the stream and
  sends commands.
- `isp` has its own task. Only the newest frame waits for it; older frames
  are dropped. The `overlay` and the outputs it feeds run on this task.
- `h264_encoder` encodes on its own task, one frame at a time. Frames that
  arrive while it is busy are dropped. A frame already in the encoder's
  layout (RGB565 little-endian, both sizes a multiple of 16, packed rows) is
  encoded straight from the producer's buffer, held until the hardware is
  done. Any other frame is first converted into a padded buffer. Without a
  configured `fps`, the encoder declares the measured source rate in the
  stream. If that rate later stays more than 25% off for 3 s, it restarts the
  stream with a keyframe.
- `rtsp_server` sends each client whole frames only. When a client's backlog
  would exceed `buffer_size`, its frames are dropped until the next keyframe.
  It asks the encoder for that keyframe once, when the client's backlog has
  drained to half of `buffer_size`, so a slow link is not flooded with
  keyframes.
- `video_camera` encodes a JPEG only after a client has asked for one. It
  uses the P4's hardware encoder, on the calling task, and hands the image to
  ESPHome's camera clients on the main loop.

Nothing upstream does work for nobody: a source asks its sinks
`wants_frames()` and idles while none do. A `radiometric` with sensors but
nothing downstream wanting frames takes one frame per `update_interval` (at
most 1 s apart), so its source converts only those. It takes every frame
while something downstream wants frames, or while `temporal_frames` is above 1.

### Processing

`radiometric` repairs invalid pixels (non-finite, or outside `valid_min`..`valid_max`)
from their valid neighbours. Then come the outlier filter (a pixel differing
from both row neighbours by at least `outlier_level` becomes their average),
the temporal filter, edge-preserving smoothing and the 3x3 median. Statistics are computed last, in
sensor coordinates. The min, max and centre become annotations on the frame.

The temporal filter keeps one filtered frame. Each new reading moves a
pixel's value towards it by 1/`temporal_frames` while the pixel's 3x3
neighbourhood changed, on average, by no more than `motion_threshold` since
the last frame; that is noise, and still areas end up averaged over about
`temporal_frames` frames. As the change grows to twice the threshold, the
step grows to the whole difference, so anything that moves or warms up
shows at once instead of smearing across frames. Looking at the
neighbourhood rather than the pixel alone keeps a single noisy reading from
passing as movement. The MLX90640 in chess mode refreshes half the pixels per
frame, which halves the neighbourhood change of real movement; lower
`motion_threshold` if moving edges still trail.

Edge-preserving smoothing is a 5x5 bilateral filter: each pixel becomes a
weighted average of its neighbours, weighted by distance (a Gaussian of
1 pixel) and by temperature difference (a Gaussian of `edge_smoothing`
degrees, nothing beyond three times that). Neighbours at about the same
temperature are averaged, which removes noise, while those across an edge
count for nothing, so edges stay as sharp as they were.
With no filter active and every pixel valid, the frame is passed on as it
is and only the statistics are computed, in one read-only pass. If the
source supplied statistics and no median is wanted, they are used and the
frame is not read at all.

`seek_compact_pro` follows libseek-thermal (MIT) for the protocol and Seek's
own library (libseekip in the Seek Thermal app 2.3.4, studied for this camera)
for the calibration. On connection it claims interface 0 and sends the vendor's
start-up sequence as vendor control requests: stop (in case the camera was
left running), target platform, firmware and chip ID, factory settings, the
factory settings block (2560 words, 32 at a time: all the camera keeps; its
64 KB beyond that is empty), image processing mode 8, then running. Each
frame is asked for with START_GET_IMAGE_TRANSFER (its word count) and
arrives on bulk endpoint 0x81 as 342x260 16-bit words, in 13 pieces of
13680 bytes that each end in a short packet; three 13824-byte transfers stay
queued. The next frame is asked for from the USB task the moment the last
piece arrives: asked for late, the camera stops for good. Word 0 of every
frame is 1401 (a frame without it is dropped), word 2 says what it is. The
picture is the 320x240 at column 2, row 12 (the camera's geometry record).
After each start the camera sends its calibration as frames, once: 4 each
pixel's response (the gain is 16384 x the mean over the pixel's value; 0
marks a bad pixel), 9 a 16384-entry table every raw value goes through
first (halved below 15000, doubled from it), 14 a temperature correction
table, 8 the start-up shutter frame and 7 offsets for raw values from 15000
up (the camera's second range); then images (3), with a shutter frame (1),
and a frame just before it (6), every few seconds. An image pixel is 3000 +
(table 9 of the raw value - table 9 of the last shutter frame) x gain /
16384. Bad pixels, and the configured `bad_columns`, take the mean of their
neighbours. Temperatures, for factory thermography version 3: s is the mean
of row 2's first 324 pixels (smoothed 7:1 over frames 1, 3, 6 and 8) less
the factory reference (byte 0x64); x = k0 + k1 (pixel - 3000) (0x98, 0x9C);
seven coefficients, each a quadratic in s (0xA0 on), give T = D + C x + A x^2
+ B x^3, or G + F x + E x^2 from the threshold at 0xF4 on; frame 14's table
(format 3: nodes in temperature along an axis in s, 1/32 °C) then takes off a
correction that is piecewise linear in T; and a warm-up transient, 6.7 °C x
exp(-t / 95 s) with t the camera's own uptime from the frame header, comes
off last, then `gain` and `offset`. On the camera measured, a wall read
70.8 °F against the room sensor's 71.3 °F, and a face 88 °F. Before the
conversion, Seek's default picture filter runs on the pixels (`scene_filter`,
on by default): a pixel more than 15 counts (about 0.6 °F) off its 3x3 median
takes the median, every frame; with enough scene motion (over 10% of the flat
pixels changing by more than 14 counts between frames) it learns each
pixel's offset, which decays 127/128 a frame otherwise; then small
frame-to-frame changes (under 24 counts) are smoothed. The picture after a
shutter frame does not learn, and the smoothing starts again. On still
scenes it never learns; the median step took the fine speckle from 0.42-0.53
to 0.32-0.34 °F, and the smoothing to 0.27-0.31 °F. A
factory block of another version, or for another camera, gives no
temperatures (logged). The tables come once per start: a shutter frame or
image before all three means one was lost (the camera sometimes sends a
short first frame), and the camera is then started again.
Table and shutter frames are always processed, in order (up to four
waiting); only images are dropped when processing falls behind. A frame that does not arrive within a
second is asked for again, its partial data discarded; after three in a row
the camera is started again.

`thermal_master_p1` follows the vendor's Android app, Thermal Master 2.3.11,
decompiled: its command library (libircmd020), its USB stream code and its
Java. The start-up sequence and the frame layout as seen on the wire are
also those of the open drivers that ran on a P1 (jvdillon/p3-ir-camera,
Apache-2.0, from captures of the vendor's Windows tool; Open-TM-P1). The
camera is a vendor device, not UVC; the library puts it in its device type
15 with the P3.

A command goes in a vendor control request (0x41, 0x20). The library's id
0xAABBCC is bytes 0xCC, 0xBB (the group), 0xAA (the command); the first
byte picks the header: 18 bytes (response length at 12-13, CRC-16/XMODEM at
16-17) or, for the camera's own temperature measurements, 24 bytes (length
at 16-17, CRC at 22-23). Each command puts its values where the library
does: one byte at 4 (most), at 5 (the palette, the isothermal mode and
switch), 16 bits at 4-5 (emissivity, transmittance, the kelvin
temperatures), a type at 4 and 16 bits at 5-6 (the auto shutter
parameters), two bytes at 4 and 5 (sun detection's min and max), two signed
16-bit values at 8-11 (isothermal limits). Reads send 0 at byte 4, except
the picture levels' "current" (1); 2 there reads the camera's limits (max,
min, step), which are logged. After a write, the 1-byte status (0xC1, 0x22)
is read until it is not 1 (busy), for as long as the library allows the
command (2 s; a range change 22 s, a shutter cycle 4 s, recalibrations up to
32 s), and it must end at 3. A read's status must be 2, then the response
(0xC1, 0x21, exactly the length asked for), then the status 3. Resets
(reboot, factory settings, bootloader) are sent with no status read. A
command the camera refuses is logged and its entity keeps its last value;
one the camera does not answer restarts the camera.

On connection: interface 1 to alternate setting 0 (in case it was left
streaming), the camera's name, firmware and system version, the range read,
1 s, interface 1 alternate setting 1 (bulk IN 0x81), the vendor request
0xEE, 2 s, bulk transfers queued, the range read again, then the range.
That is the Windows tool's sequence (without its delays the camera takes
every command and sends nothing); the vendor's Android app only selects the
interface, sends 0xEE and queues its reads. After a range change the
camera's state (0x830101) is read every second while it says the change is
under way, then the range is read back. A frame arrives as a 12-byte start
marker (a UVC payload header), 242 rows of 160 16-bit words (the camera's
YUYV picture, two metadata rows, then the temperatures), ending in a short
packet, and a 12-byte end marker as a transfer of its own; four 8 KB
transfers stay queued. A frame is kept when its end marker comes exactly
after its rows, with start and end markers matching (sync bytes 0x8C/0x8D
and 0x8E/0x8F, the same frame counter). An end marker anywhere else ends a
frame joined part way, and a frame longer than a frame (the one after a
shutter cycle) is dropped up to its end marker. The markers' phase steps
about 40 a frame, so frames the camera made and never sent are counted.

A temperature is value / 64 - 273.15 °C in both ranges, times `gain` plus
`offset`. In the high range a value under 200 K is no reading and takes its
left (or upper) neighbour's, and a frame with no readings at all is not
passed on and shows in `status`. In the low range a value under 302 °F
reads 302 °F (the vendor app shows "<150 °C"; at room temperature such
values are about -357 °F). The automatic range is the library's, run on
every frame: more than 95 % of the pixels under 338 °F votes for the high
range, otherwise more than 10 % at or over 338 °F votes for the low range;
after more than 125 frames in a row with the same vote the range is
changed if it differs, and the next 176 frames are not looked at. The
metadata rows are read where the vendor's Java reads them: byte 42 the
shutter countdown, 138 the camera's sun flag. If no frame comes within 5 s
of the start, or 3 s while streaming, the camera is started again (the
vendor app has no such watchdog). The picture rows are YUYV 4:2:2,
converted to RGB565.

The vendor app corrects for emissivity, ambient temperature and distance
itself, from tables of its own, and never sends the camera's environment
correction commands; they are here as the camera's settings, unused unless
set. The library's other commands are not offered: zoom (not in its command
table; the apps zoom in software), scene modes (the app offers them only on
other models), the stream source setting and the camera's point display
(refused for device type 15), pause and preview stop (the component starts
the stream again when frames stop), and the engineering ones whose contents
the library does not describe (ISP and algorithm registers, TPD parameters,
OEM calibration files, the EEPROM update flag, the second calibration
flags).

`hti301` follows the camera's own driver. On connection it reads the UVC
descriptors, sets the streaming interface to alternate setting 0,
negotiates the stream with probe/commit, selects the isochronous alternate
setting and queues the transfers, pausing between steps as that driver does.
A frame is 384x292 16-bit words: 288 image rows and 4 rows of camera
metadata. A frame counts only if every packet arrived and it has exactly the
negotiated size; others are dropped. The component sets the USB controller's
FIFO split to favour received data (`CONFIG_USB_HOST_HW_BUFFER_BIAS_IN`):
packets wait there while the controller's DMA is held up by other memory
traffic. The USB interrupt starts each queued transfer as the one before it
ends and has one 125 us microframe to do it; ESP-IDF runs most of that path
from flash, through the cache that frame processing keeps filling with PSRAM
data. A linker fragment (`hti301/hti301_usb_iram`, about 4 KB of IRAM) moves
it to IRAM, so the interrupt is not late. Cache write-backs before DMA
(H.264, JPEG, PPA) run with interrupts off and hold a lock the USB driver
takes, so the component has ESP-IDF do them in 8 KB pieces
(`CONFIG_ESP_MM_CACHE_MSYNC_C2M_CHUNKED_OPS`); a whole frame at once would
outlast the microframe. With `log_timing`, lost packets are split into those the controller
did not run in time (and how many of them opened a transfer, which means the
next transfer started late) and those received with errors. Commands (mode, range, palette,
calibrate) are written to the camera terminal's Zoom (Absolute) control. The
camera cannot report its mode, so after every stream start the component
sends the configured mode, range and palette, and passes frames on only
once the camera has accepted the mode. After a mode or range change it skips
5 frames. If no complete frame arrives for 3 s, it restarts the stream.

The camera never does a shutter calibration on its own. Its raw values drift
as its detector warms, about 4 °C of reading per °C of detector (a cold
start warms it by 5 °C or more), and only a calibration resets them. With
`auto_calibrate`, the component sends one 30 s after the stream starts, then
whenever the detector has moved 0.25 °C since the last one (at most every
30 s), and at least every 10 minutes. In processed mode, where the detector's
temperature is not available, only the time limits apply. A press of the
`calibrate` button counts as one. After each calibration it skips 1 s of
frames, which mix the old and new calibration while the shutter moves.
While no camera is connected, it lists every 10 s the devices the USB host
has enumerated and asks each hub for the state of its ports (empty, device
attached, not enabled, over-current). With nothing enumerated, it reads
the host port's own status: whether anything is electrically attached, and
whether the port was enabled. If the camera is on the bus but its
connection event was missed, it connects to it.

In radiometric mode each pixel is a 14-bit raw value. The metadata rows hold
the camera's calibration (detector and shutter temperatures, calibration
constants) and the scene parameters stored in it. The conversion is the one
in ht301_hacklib, including atmospheric transmission over the distance. For
each frame a table is built over the frame's range of raw values, and every
pixel is looked up in it; the frame's statistics are gathered in the same
pass. This loop, and `isp`'s scale-1 loop below, are compiled for speed
(-O2, unrolled) while the rest of the firmware is compiled for size. In
processed mode the image rows are full-range YUYV, converted to RGB565. For the high range, raise `radiometric`'s
`valid_max` (default 300 °C) above 450 °C so hot pixels are not treated as
invalid.

`isp` tone-maps once, at sensor resolution: each value becomes an index
into the 256-entry palette, and mirroring is applied at the same time.
By default the map is linear over `range_min`..`range_max`. With `equalize`
above 0 it bends towards histogram equalization: a histogram of the frame's
in-range values over 64 bins, each bin capped at three times the average
(a plateau, so a large uniform background cannot take most of the colours
and show its noise), summed into a curve that gives each band of
temperatures colours in proportion to how much of the scene it covers.
`equalize` is the percentage of that curve mixed into the linear one. Each
frame's curve is blended into the previous one with a time constant of
150 ms (about a quarter per frame at 25 frames/s, all of it at one frame a
second or slower), so the colours do not flicker with noise and follow the
scene equally fast at any frame rate. With `detail` above 0, the frame is
also smoothed with a 3x3 kernel, and each pixel's difference from the
smoothed frame is multiplied by 1 + `detail` after mapping. Flat areas are
unchanged; edges and small features gain contrast, with a slight bright and
dark rim on either side of an edge.
Scaling then works on these indices. At scale 1 there is nothing to scale,
so each value is tone-mapped and coloured straight into the output frame in
one pass (unless `equalize` or `detail` is on). The CPU path has three modes. Nearest
builds each output row once and copies it. Bilinear blends the two source
rows once per output row, then interpolates along the row, in fixed point.
Bilinear and bicubic interpolate the mapped values before they are rounded
to a palette index (16 bits, 8 of them fraction) and round per output pixel.
Rounding first would leave nothing to interpolate where neighbouring pixels
are less than one palette entry apart, as in a scene much narrower than the
range: the steps between entries would then sit on the sensor's pixel grid
and show as blocks, however large the scale.
Bicubic (Catmull-Rom) does the same with four rows and four columns; its
taps and weights depend only on an output pixel's position within its
input pixel, so they are worked out once per position. It gives smoother
curves than bilinear, without bilinear's diamond-shaped artefacts at large
scales, and overshoots slightly at edges, which reads as sharpening. The PPA
scales only bilinearly, so bicubic always scales on the CPU.
The PPA path has the PPA scale the index image (8-bit grey, chip revision 3
or later) bilinearly, then looks the palette up on the CPU, one table read
per output pixel. The PPA takes 8-bit pixels, so on this path the indices
are rounded before scaling. Scaling the coloured image instead would blend colours,
which leaves colours between pixels that are not on the palette. ESP-IDF 5.5
has no driver for the PPA's own colour look-up table. The PPA scales only
bilinearly, so with `upscale: nearest` the CPU scales, which only copies. If
the PPA rejects the operation (an older chip revision), the ISP scales on
the CPU from then on.
Annotations are moved through the same mirroring and scaling as the pixels.

The ISP keeps the last frame it drew. When one of its options, the palette,
a spot or a downstream overlay's options change, it draws that frame again
at once, so a change shows without waiting for the next frame, however
slow the sensor's frame rate. Options of the sensor and of `radiometric`
still apply from the next frame.

Spots belong to the image as shown: a spot stays where it was placed when
the image is mirrored, and measures whatever is shown there. Its value is the
temperature at that point, interpolated between the four nearest sensor
pixels as the image is, then corrected for the spot's material. Sources
report the emissivity they computed temperatures with and the reflected
(background) temperature they assumed: the MLX90640 its `emissivity` and its
ambient temperature less 8 °C (Melexis' figure for open air), the HTI-301 its
emissivity and reflected temperature. Taking a surface as a grey body,
radiance in proportion to the fourth power of its absolute temperature, the
source saw `e_s·T⁴ + (1 − e_s)·R⁴`; a surface of emissivity `e` giving that
is at `To⁴ = (e_s·T⁴ + (e − e_s)·R⁴) / e`. A surface at the reflected
temperature reads the same at any emissivity; one warmer than it is warmer
still at a lower emissivity, one colder, colder still. A click within 6 % of
the image height of a spot (about its reticle) removes it.

`overlay` draws only its own pixels: a reticle and a label per annotation,
white (a spot: its colour) with a one-pixel black outline. Each label goes below, above, right or
left of its reticle, taking the first position that is inside the image and
clear of other labels and reticles. A mark's position and value are
refreshed together once per `hold`, so neither the reticle nor its label
jumps with every frame's noise. Because it draws into the frame it
receives, it must be its source's only consumer. A source with an overlay
and another consumer fails at setup with an error. Attach other consumers to
the overlay instead.

### Matter camera

The camera endpoint carries the Camera AV Stream Management and WebRTC
Transport Provider clusters. A controller allocates a video stream, then
sends a WebRTC offer through the WebRTC Transport Provider cluster; the
answer and the ICE candidates go back the same way, so Matter carries all
the signaling. The media goes directly from the device to the viewer as
H.264 in RTP over DTLS-SRTP.

The cluster delegates and the signaling bridge come from Espressif's
esp-matter camera example (standalone mode, release v1.6). WebRTC itself is
Espressif's ESP-IDF port of Amazon's KVS WebRTC SDK, fetched at a pinned
commit when the firmware is built. Its own camera, ISP and encoder code is
left out: `matter_camera` is one of the encoder's consumers and passes each
access unit to KVS's frame bus, from which every WebRTC session sends it.
KVS asks the encoder for a keyframe when a session starts or drops a frame.
KVS keeps its buffers and task stacks in PSRAM, and mbedTLS and the Matter
stack are switched to PSRAM too, so a session costs little internal RAM.

`matter` downloads Espressif's esp_matter component (esphome-matter's
build) and drops its pins of the ESP32-P4 Wi-Fi libraries, so ESPHome's
`esp32_hosted` versions are used.

## Tests

`tests/run_tests.py` runs:

- unit tests under AddressSanitizer and UBSan: frames and pools, radiometric
  processing, the CPU ISP, overlay drawing, sub-page assembly, the Seek
  Compact Pro core (frame header and geometry, factory block, the start-up
  tables: gains and bad pixels, raw-value table, correction table, second
  range; bad columns, shutter offsets, sensor value, temperatures, warm-up,
  the order of frames), the Thermal Master P1 core (the commands byte for
  byte as captured from the vendor's tool; the vendor library's command
  set: headers, value placement, reads, limits, time allowed, and where it
  meets the captures; markers, frame assembly: joined part way, the long
  frame after a shutter cycle, bad markers, no buffer, frames the camera
  skipped; temperatures in both ranges; the metadata rows; the automatic
  range's votes; the picture),
  the HTI-301
  core (descriptors, packet assembly, the temperature conversion against the
  reference code, YUYV), the RTSP protocol helpers, and the RTSP server's
  keyframe requests for a client that falls behind; the frame pool also runs
  under ThreadSanitizer;
- the RTSP server on the host against ffmpeg, ffprobe and GStreamer clients;
- the encoder's build guard;
- ESPHome configurations: invalid ones must be rejected with the expected
  message, and valid ones (including a Matter camera) must compile for the
  ESP32-P4 without warnings in these components.

`--skip-esphome` skips the ESPHome compiles.
