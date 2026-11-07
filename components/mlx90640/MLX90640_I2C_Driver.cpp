/**
   @copyright (C) 2017 Melexis N.V.

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.

*/
#include "MLX90640_I2C_Driver.h"
#include "esphome/core/log.h"

static const char *TAG = "MLX90640_I2C";
static esphome::i2c::I2CDevice *mlx90640_i2c_device = nullptr;

void MLX90640_I2CInit(esphome::i2c::I2CDevice *device) {
  mlx90640_i2c_device = device;
}

//Read a number of words from startAddress. Store into Data array.
//Returns 0 if successful, -1 if error
int MLX90640_I2CRead(uint8_t _deviceAddress, unsigned int startAddress, unsigned int nWordsRead,
                     uint16_t *data) {
  if (mlx90640_i2c_device == nullptr) {
    ESP_LOGE(TAG, "I2C device not initialized");
    return -1;
  }
  (void) _deviceAddress;

  uint16_t bytesRemaining = nWordsRead * 2;
  uint16_t dataSpot = 0;

  while (bytesRemaining > 0) {
    uint16_t numberOfBytesToRead = bytesRemaining;
    if (numberOfBytesToRead > I2C_BUFFER_LENGTH) {
      numberOfBytesToRead = I2C_BUFFER_LENGTH;
    }

    uint8_t reg[2] = {static_cast<uint8_t>(startAddress >> 8),
                      static_cast<uint8_t>(startAddress & 0xFF)};
    uint8_t buffer[I2C_BUFFER_LENGTH];
    esphome::i2c::ErrorCode err = mlx90640_i2c_device->write_read(reg, sizeof(reg), buffer, numberOfBytesToRead);
    if (err != esphome::i2c::ErrorCode::ERROR_OK)
    {
      ESP_LOGE(TAG, "I2C read failed: %d", (int)err);
      return -1;
    }

    for (uint16_t x = 0; x < numberOfBytesToRead / 2; x++) {
      data[dataSpot] = static_cast<uint16_t>(buffer[x * 2]) << 8;
      data[dataSpot] |= buffer[x * 2 + 1];
      dataSpot++;
    }

    bytesRemaining -= numberOfBytesToRead;
    startAddress += numberOfBytesToRead / 2;
  }

  return 0;
}

//Write two bytes to a two byte address
int MLX90640_I2CWrite(uint8_t _deviceAddress, unsigned int writeAddress, uint16_t data) {
  if (mlx90640_i2c_device == nullptr) {
    ESP_LOGE(TAG, "I2C device not initialized");
    return -1;
  }
  (void) _deviceAddress;

  uint8_t buffer[4] = {static_cast<uint8_t>(writeAddress >> 8),
                       static_cast<uint8_t>(writeAddress & 0xFF),
                       static_cast<uint8_t>(data >> 8),
                       static_cast<uint8_t>(data & 0xFF)};
  esphome::i2c::ErrorCode err = mlx90640_i2c_device->write(buffer, sizeof(buffer));

  if (err != esphome::i2c::ErrorCode::ERROR_OK)
  {
    ESP_LOGE(TAG, "I2C write failed: %d", (int)err);
    return -1;
  }

  return 0;
}
