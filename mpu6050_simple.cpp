
#include "mpu6050_simple.h"

#define REG_PWR_MGMT_1  0x6B
#define REG_SMPLRT_DIV  0x19
#define REG_CONFIG      0x1A
#define REG_GYRO_CONFIG 0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B

bool MPU6050Simple::begin(TwoWire &w, uint8_t addr){
  wire = &w; address = addr;
  wire->begin();
  write8(REG_PWR_MGMT_1, 0x00);
  delay(5);
  write8(REG_SMPLRT_DIV, 0x04);
  write8(REG_CONFIG, 0x03);
  write8(REG_GYRO_CONFIG, 0x00);
  write8(REG_ACCEL_CONFIG, 0x00);
  delay(10);
  uint8_t buf[2]={0};
  readN(REG_ACCEL_XOUT_H, buf, 2);
  return true;
}

void MPU6050Simple::setOffsets(int16_t gx_off, int16_t gy_off, int16_t gz_off){
  gx0=gx_off; gy0=gy_off; gz0=gz_off;
}

void MPU6050Simple::calibrate(uint16_t samples){
  long sum_gx=0, sum_gy=0, sum_gz=0;
  for(uint16_t i=0;i<samples;i++){
    update();
    sum_gx += gx;
    sum_gy += gy;
    sum_gz += gz;
    delay(2);
  }
  gx0 = (int16_t)(sum_gx / (long)samples);
  gy0 = (int16_t)(sum_gy / (long)samples);
  gz0 = (int16_t)(sum_gz / (long)samples);
}

void MPU6050Simple::update(){
  uint8_t buf[14];
  readN(REG_ACCEL_XOUT_H, buf, 14);
  ax = (int16_t)((buf[0]<<8)|buf[1]);
  ay = (int16_t)((buf[2]<<8)|buf[3]);
  az = (int16_t)((buf[4]<<8)|buf[5]);
  gx = (int16_t)((buf[8]<<8)|buf[9]);
  gy = (int16_t)((buf[10]<<8)|buf[11]);
  gz = (int16_t)((buf[12]<<8)|buf[13]);

  accX_g = ax / ACC_SENS;
  accY_g = ay / ACC_SENS;
  accZ_g = az / ACC_SENS;

  gyroX_dps = (gx - gx0) / GYRO_SENS;
  gyroY_dps = (gy - gy0) / GYRO_SENS;
  gyroZ_dps = (gz - gz0) / GYRO_SENS;
}

void MPU6050Simple::write8(uint8_t reg, uint8_t val){
  wire->beginTransmission(address);
  wire->write(reg);
  wire->write(val);
  wire->endTransmission();
}

void MPU6050Simple::readN(uint8_t reg, uint8_t *buf, size_t n){
  wire->beginTransmission(address);
  wire->write(reg);
  wire->endTransmission(false);
  wire->requestFrom((int)address, (int)n);
  for(size_t i=0;i<n && wire->available();i++){
    buf[i]=wire->read();
  }
}
