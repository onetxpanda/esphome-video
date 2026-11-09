# esphome-video

ESPHome external components for building a video pipeline on an ESP32-P4:
a sensor produces frames, processing stages transform them, and outputs
encode or serve them. Encoding uses the P4's hardware H.264 and JPEG
encoders; there is no software encoding.

| Component | Role |
|---|---|
| `mlx90640` | MLX90640 32x24 thermal sensor. Radiometric source (temperatures). |
| `hti301` | HTI-301 USB thermal camera, 384x288 at 25 fps. Radiometric source in radiometric mode; image source (the camera's own processed image) in processed mode. |
| `radiometric` | Temperature processing: invalid-pixel repair, outlier filter, temporal average, median filter, statistics sensors. |
| `isp` | Radiometric frames to images: tone mapping onto a palette, mirroring, upscaling (CPU or the P4's PPA), output format. |
| `overlay` | Draws a frame's annotations (e.g. min/max/centre markers with values) into the image. |
| `h264_encoder` | ESP32-P4 hardware H.264 encoder, with a `select` platform to switch between sources. |
| `rtsp_server` | Serves the encoder's output over RTSP (RTP over TCP). |
| `video_camera` | ESPHome camera entity (Home Assistant, web server), JPEG from the P4's hardware encoder. |
| `video_test_pattern` | Synthetic image source for checking encoders and streams. |
| `video_palette` | One select entity choosing the palette of several ISPs and cameras at once. |
| `video_switch` | Chooses which of several image sources reaches the consumers after it (encoder, camera), with a select entity. |
| `assert_capture` | Records why the firmware aborted (failed assert with file and line, `abort()` caller, task stack overflow) and reports it after the reboot. |
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

### Options

Every stage takes its options in YAML. Most can also be changed at runtime
through Home Assistant entities, declared under `controls:`. Each entity
restores its last value on boot.

`radiometric`, `isp` and `overlay` can each be configured more than once, as
a YAML list, for example one chain per camera.

- **mlx90640**: `refresh_rate` (0.5Hz–64Hz), `frame_mode` (interpolate, merge),
  `emissivity`. Controls: `refresh_rate`, `frame_mode`.
- **hti301**: `mode` (radiometric, processed), `range` (normal, high),
  `palette` (processed mode: white_hot, black_hot, ironbow, lava, rainbow,
  iron_gray, red_hot, rainbow_2), `emissivity` (default 0.95), and
  `reflected_temperature`, `ambient_temperature`, `humidity`, `distance`
  (left out: the values stored in the camera). `transfers` and
  `packets_per_transfer` size the USB buffering (default 8 x 16 packets,
  16 ms). Binary sensor: `streaming` (on while frames arrive). Text sensor:
  `usb_status` (while no camera is connected, the USB devices the host sees
  and the state of each hub port; then the stream's state). Controls:
  `mode`, `range`, `palette`, `emissivity`, a `calibrate` button
  (shutter calibration), and a `restart_usb` button (powers the USB port off
  and on, so everything on it re-enumerates).
- **radiometric**: `temporal_frames` (1–8; 1 = off), `median_filter`,
  `outlier_level` (degrees; 0 = off), `valid_min` and `valid_max` (readings
  outside are repaired; default −40..300 °C), `update_interval` for the
  sensors. The filters are meant for the MLX90640's noisy low-resolution
  frames; for a camera that does its own correction (the HTI-301) set
  `outlier_level: 0` and leave the others off. The median statistic is
  computed only when its sensor is configured.
  Sensors: `min`, `max`, `mean`, `median`, `center`. Controls:
  `temporal_frames`, `median_filter`, `outlier_level`.
- **isp**: `scale` (1–20), `format` (RGB565_LE, RGB565_BE, RGB888, GRAY8),
  `palette` (white_hot, black_hot, ironbow, lava, rainbow, iron_gray,
  red_hot, rainbow_2; the order of the HTI-301's own palettes), `range_min`, `range_max`, `mirror_x`, `mirror_y`,
  `upscale` (nearest, bilinear), `scaler` (cpu, ppa). Controls: `range_min`,
  `range_max`, `mirror_x`, `mirror_y`, `upscale`, `scaler`.
- **overlay**: `marks` (min, max, center), `hold` (how often a mark's value
  is refreshed while the mark stays put; default 1 s), `unit` (C, F),
  `enabled`. Controls: `enabled`, `unit`.
- **h264_encoder**: `source` (list), `fps` (default: measured from the
  source), `gop` (default: one keyframe per second), `bitrate` (default:
  automatic), `qp_min`, `qp_max`. Select platform: switches the active source.
- **rtsp_server**: `port`, `path`, `session_name`, `max_clients`, `buffer_size`.
- **video_camera**: `quality`, `subsampling` (444, 422, 420).
- **video_test_pattern**: `width`, `height`, `fps`.
- **assert_capture**: `last_crash` (text sensor). The record is also logged
  at boot and in the config dump.
- **video_palette**: `targets` (ISPs, and cameras that colour their own image
  such as the HTI-301), `palette` (initial). The select restores its last
  value.
- **video_switch**: `sources` (each a `name` and one or more `source`s),
  `select` (the entity choosing the entry; restores its last value).

Several stages can be marked `log_timing: true` (`mlx90640`, `hti301`,
`radiometric`, `isp`). They then log their per-stage times every 10 s.

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
memory (the temporal history, filter scratch, palette indices) is allocated
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
  stream. If that rate later drifts by more than 25%, it restarts the stream
  with a keyframe.
- `video_camera` encodes a JPEG only after a client has asked for one. It
  uses the P4's hardware encoder, on the calling task, and hands the image to
  ESPHome's camera clients on the main loop.

Nothing upstream does work for nobody: a source asks its sinks
`wants_frames()` and idles while none do.

### Processing

`radiometric` repairs invalid pixels (non-finite, or outside `valid_min`..`valid_max`)
from their valid neighbours. Then come the outlier filter (a pixel differing
from both row neighbours by at least `outlier_level` becomes their average),
the running average and the 3x3 median. Statistics are computed last, in
sensor coordinates. The min, max and centre become annotations on the frame.
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
pass. In processed mode the image rows are full-range
YUYV, converted to RGB565. For the high range, raise `radiometric`'s
`valid_max` (default 300 °C) above 450 °C so hot pixels are not treated as
invalid.

`isp` tone-maps once, at sensor resolution: each value becomes an index
into the 256-entry palette, and mirroring is applied at the same time.
Scaling then works on these indices. At scale 1 there is nothing to scale,
so each value is tone-mapped and coloured straight into the output frame in
one pass. The CPU path has two modes. Nearest
builds each output row once and copies it. Bilinear blends the two source
rows once per output row, then interpolates along the row, in fixed point.
The PPA path colours the small image and has the PPA scale it. The PPA's
interpolation method is fixed by the hardware and cannot be selected.
Annotations are moved through the same mirroring and scaling as the pixels.

`overlay` draws only its own pixels: a reticle and a label per annotation,
white with a one-pixel black outline. Each label goes below, above, right or
left of its reticle, taking the first position that is inside the image and
clear of other labels and reticles. A mark moves as soon as its annotation
does, and its label shows the new value at once; while the mark stays on the
same pixel, its value is refreshed once per `hold`, so the label does not
change with every frame's noise. Because it draws into the frame it
receives, it must be its source's only consumer. A source with an overlay
and another consumer fails at setup with an error. Attach other consumers to
the overlay instead.

## Tests

`tests/run_tests.py` runs:

- unit tests under AddressSanitizer and UBSan: frames and pools, radiometric
  processing, the CPU ISP, overlay drawing, sub-page assembly, the HTI-301
  core (descriptors, packet assembly, the temperature conversion against the
  reference code, YUYV), and the RTSP protocol helpers; the frame pool also runs under ThreadSanitizer;
- the RTSP server on the host against ffmpeg, ffprobe and GStreamer clients;
- the encoder's build guard;
- ESPHome configurations: invalid ones must be rejected with the expected
  message, and valid ones must compile for the ESP32-P4 without warnings in
  these components.

`--skip-esphome` skips the ESPHome compiles.
