// MLX90640 on a Linux I2C bus (i2c-dev: a Raspberry Pi's pins, or a USB-I2C
// adapter with a kernel driver): the component's set-up and capture loop with
// the Melexis API and SubpageAssembler; only the two I2C functions the API
// calls are different, since there is no ESPHome bus here.
//
// arg: "<bus>[,<address>[,<rate>]]", e.g. "/dev/i2c-1,0x33,16Hz" (those are
// the defaults; rates as the component's refresh_rate, in sub-pages a second).

#include "tv_camera.h"

#ifdef __linux__

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <thread>

#include "mlx90640/MLX90640_API.h"
#include "mlx90640/MLX90640_I2C_Driver.h"
#include "mlx90640/subpage_assembler.h"

// ---------------------------------------------------------------- the API's I2C functions
static int g_fd = -1;

void MLX90640_I2CInit(esphome::i2c::I2CDevice *) {}

int MLX90640_I2CRead(uint8_t addr, unsigned int start, unsigned int words, uint16_t *data) {
  if (g_fd < 0)
    return -1;
  // The bus driver limits one read; 256 words fit every adapter.
  static constexpr unsigned STEP = 256;
  uint8_t buf[STEP * 2];
  while (words > 0) {
    const unsigned n = words < STEP ? words : STEP;
    uint8_t reg[2] = {static_cast<uint8_t>(start >> 8), static_cast<uint8_t>(start & 0xFF)};
    i2c_msg msgs[2] = {{addr, 0, 2, reg}, {addr, I2C_M_RD, static_cast<uint16_t>(n * 2), buf}};
    i2c_rdwr_ioctl_data x{msgs, 2};
    if (ioctl(g_fd, I2C_RDWR, &x) < 0)
      return -1;
    for (unsigned i = 0; i < n; i++)
      *data++ = static_cast<uint16_t>(buf[2 * i] << 8 | buf[2 * i + 1]);
    words -= n;
    start += n;
  }
  return 0;
}

int MLX90640_I2CWrite(uint8_t addr, unsigned int reg, uint16_t value) {
  if (g_fd < 0)
    return -1;
  uint8_t buf[4] = {static_cast<uint8_t>(reg >> 8), static_cast<uint8_t>(reg & 0xFF), static_cast<uint8_t>(value >> 8),
                    static_cast<uint8_t>(value & 0xFF)};
  i2c_msg msg{addr, 0, 4, buf};
  i2c_rdwr_ioctl_data x{&msg, 1};
  return ioctl(g_fd, I2C_RDWR, &x) < 0 ? -1 : 0;
}

namespace tv {
namespace {

using namespace esphome::mlx90640;
constexpr int TA_SHIFT = 8;  // reflected temperature offset for open air (Melexis)
constexpr uint8_t SETTLE_SUBPAGES = 4;
constexpr float EMISSIVITY = 0.95f;  // the component's default
const char *const RATES[] = {"0.5hz", "1hz", "2hz", "4hz", "8hz", "16hz", "32hz", "64hz"};

class Mlx : public Camera {
 public:
  void run(Host &host, const std::string &arg) override {
    std::string bus = "/dev/i2c-1";
    uint8_t addr = 0x33, rate = 5;
    size_t a = 0;
    for (int field = 0; a <= arg.size() && !arg.empty(); field++) {
      const size_t b = std::min(arg.find(',', a), arg.size());
      std::string v = arg.substr(a, b - a);
      a = b + 1;
      if (v.empty())
        continue;
      if (field == 0) {
        bus = v;
      } else if (field == 1) {
        addr = static_cast<uint8_t>(std::strtoul(v.c_str(), nullptr, 0));
      } else if (field == 2) {
        for (auto &c : v)
          c = static_cast<char>(std::tolower(c));
        for (uint8_t i = 0; i < 8; i++)
          if (v == RATES[i])
            rate = i;
      }
    }
    g_fd = ::open(bus.c_str(), O_RDWR);
    if (g_fd < 0) {
      host.status("MLX90640: cannot open " + bus + " (" + std::strerror(errno) + ")");
      return;
    }
    this->capture_(host, addr, rate);
    ::close(g_fd);
    g_fd = -1;
  }

 protected:
  void capture_(Host &host, uint8_t addr, uint8_t rate) {
    static uint16_t ee[832], raw[834];
    static float temps[SENSOR_PIXELS];
    paramsMLX90640 params{};
    if (MLX90640_DumpEE(addr, ee) != 0) {
      host.status("MLX90640: cannot read the sensor EEPROM");
      return;
    }
    const int status = MLX90640_ExtractParameters(ee, &params);
    if (status != 0) {
      host.status("MLX90640: calibration data unusable (" + std::to_string(status) + ")");
      return;
    }
    // Chess mode and the calibrated ADC resolution, set explicitly (as the component).
    MLX90640_SetChessMode(addr);
    MLX90640_SetResolution(addr, params.resolutionEE);
    if (MLX90640_SetRefreshRate(addr, rate) != 0) {
      host.status("MLX90640: cannot write the sensor control register");
      return;
    }
    const int chess = MLX90640_GetCurMode(addr);
    SubpageAssembler assembler;
    assembler.set_mode(FrameMode::INTERPOLATE);
    uint8_t settle = SETTLE_SUBPAGES;
    unsigned frames = 0, errors = 0;
    const unsigned step_ms = std::max(1u, (2000u >> rate) / 8);
    auto report = std::chrono::steady_clock::now();
    while (!host.stopping()) {
      uint16_t st;
      if (MLX90640_I2CRead(addr, MLX90640_STATUS_REG, 1, &st) != 0) {
        errors++;
        if (!host.wait(100))
          break;
        continue;
      }
      if (!MLX90640_GET_DATA_READY(st)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(step_ms));
        continue;
      }
      if (MLX90640_GetFrameData(addr, raw) < 0) {
        errors++;
        assembler.set_mode(FrameMode::INTERPOLATE);
        continue;
      }
      if (settle > 0) {
        settle--;
        continue;
      }
      const float ta = MLX90640_GetTa(raw, &params);
      MLX90640_CalculateTo(raw, &params, EMISSIVITY, ta - TA_SHIFT, temps);
      MLX90640_BadPixelsCorrection(params.brokenPixels, temps, chess, &params);
      MLX90640_BadPixelsCorrection(params.outlierPixels, temps, chess, &params);
      if (assembler.add(temps, MLX90640_GetSubPageNumber(raw), chess == 1)) {
        frames++;
        host.frame(assembler.frame(), SENSOR_COLS, SENSOR_ROWS, nullptr, EMISSIVITY, ta - TA_SHIFT);
      }
      if (std::chrono::steady_clock::now() >= report) {
        report += std::chrono::seconds(1);
        host.status("MLX90640 32 x 24, " + std::string(RATES[rate]) + " sub-pages - " + std::to_string(frames) +
                    " frames, " + std::to_string(errors) + " read errors");
      }
    }
    host.status("Stopped");
  }
};

}  // namespace

CameraType mlx90640_type() {
  return {"mlx90640", "MLX90640 (I2C)", "bus[,address[,rate]]  e.g. /dev/i2c-1,0x33,16Hz", "", {}, 20, 10.0f,
          []() -> Camera * { return new Mlx(); }};
}

}  // namespace tv

#else

namespace tv {
CameraType mlx90640_type() {
  return {"mlx90640", "MLX90640 (I2C)", "", "needs a Linux I2C bus (/dev/i2c-N); this system has none", {}, 20, 10.0f,
          []() -> Camera * { return nullptr; }};
}
}  // namespace tv

#endif
