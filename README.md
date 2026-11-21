# esphome-video

ESPHome external components for building a video pipeline on an ESP32-P4:
a sensor produces frames, processing stages transform them, and outputs
encode or serve them. Encoding uses the P4's hardware H.264 and JPEG
encoders; there is no software encoding.

| Component | Role |
|---|---|
| `mlx90640` | MLX90640 32x24 thermal sensor. Radiometric source (temperatures). |
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
Bicubic (Catmull-Rom) does the same with four rows and four columns; its
taps and weights depend only on an output pixel's position within its
input pixel, so they are worked out once per position. It gives smoother
curves than bilinear, without bilinear's diamond-shaped artefacts at large
scales, and overshoots slightly at edges, which reads as sharpening. The PPA
scales only bilinearly, so bicubic always scales on the CPU.
The PPA path has the PPA scale the index image (8-bit grey, chip revision 3
or later) bilinearly, then looks the palette up on the CPU, one table read
per output pixel. Scaling the coloured image instead would blend colours,
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
  processing, the CPU ISP, overlay drawing, sub-page assembly, the HTI-301
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
