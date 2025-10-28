
#pragma once
#include <Arduino.h>
#include <Wire.h>

class MPU6050Simple {
public:
  int16_t ax, ay, az, gx, gy, gz;
  float accX_g, accY_g, accZ_g;
  float gyroX_dps, gyroY_dps, gyroZ_dps;

  bool begin(TwoWire &w = Wire, uint8_t addr = 0x68);
  void setOffsets(int16_t gx_off, int16_t gy_off, int16_t gz_off);
  void calibrate(uint16_t samples = 500);
  void update();

  inline float getAccX() const { return accX_g; }
  inline float getAccY() const { return accY_g; }
  inline float getAccZ() const { return accZ_g; }
  inline float getGyroX() const { return gyroX_dps; }
  inline float getGyroY() const { return gyroY_dps; }
  inline float getGyroZ() const { return gyroZ_dps; }

private:
  TwoWire *wire;
  uint8_t address;
  int16_t gx0=0, gy0=0, gz0=0;
  static constexpr float ACC_SENS = 16384.0f;
  static constexpr float GYRO_SENS = 131.0f;
  void write8(uint8_t reg, uint8_t val);
  void readN(uint8_t reg, uint8_t *buf, size_t n);
};
