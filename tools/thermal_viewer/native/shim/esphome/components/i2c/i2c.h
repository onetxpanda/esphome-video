#pragma once

// Stands in for ESPHome's I2C header when the Melexis API is built for the
// desktop viewer: MLX90640_I2C_Driver.h only names the device class. The
// viewer has its own MLX90640_I2CRead / MLX90640_I2CWrite (cam_mlx90640.cpp).

namespace esphome {
namespace i2c {
class I2CDevice;
}  // namespace i2c
}  // namespace esphome
