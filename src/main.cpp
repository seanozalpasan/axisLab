#include "daisy_seed.h"
#include <stdio.h>

using namespace daisy;
using namespace daisy::seed;

// MPU6050 I2C address
#define MPU6050_ADDR 0x68

// MPU6050 Registers
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H  0x43

DaisySeed hw;
I2CHandle i2c;

void initMPU6050() {
  uint8_t data[2];

  // Wake up MPU6050 (write 0 to PWR_MGMT_1 register)
  data[0] = MPU6050_PWR_MGMT_1;
  data[1] = 0x00;

  i2c.TransmitBlocking(MPU6050_ADDR, data, 2, 100);
  System::Delay(100);
}

void readMPU6050(int16_t &accelX, int16_t &accelY, int16_t &accelZ,
                 int16_t &gyroX, int16_t &gyroY, int16_t &gyroZ) {
  uint8_t reg;
  uint8_t buffer[6];

  // Read accelerometer data
  reg = MPU6050_ACCEL_XOUT_H;
  i2c.TransmitBlocking(MPU6050_ADDR, &reg, 1, 100);
  i2c.ReceiveBlocking(MPU6050_ADDR, buffer, 6, 100);

  accelX = (buffer[0] << 8) | buffer[1];
  accelY = (buffer[2] << 8) | buffer[3];
  accelZ = (buffer[4] << 8) | buffer[5];

  // Read gyroscope data
  reg = MPU6050_GYRO_XOUT_H;
  i2c.TransmitBlocking(MPU6050_ADDR, &reg, 1, 100);
  i2c.ReceiveBlocking(MPU6050_ADDR, buffer, 6, 100);

  gyroX = (buffer[0] << 8) | buffer[1];
  gyroY = (buffer[2] << 8) | buffer[3];
  gyroZ = (buffer[4] << 8) | buffer[5];
}

int main(void) {
  // Initialize Daisy Seed hardware
  hw.Init();

  // Configure I2C
  I2CHandle::Config i2c_conf;
  i2c_conf.periph = I2CHandle::Config::Peripheral::I2C_1;
  i2c_conf.mode   = I2CHandle::Config::Mode::I2C_MASTER;
  i2c_conf.speed  = I2CHandle::Config::Speed::I2C_400KHZ;
  i2c_conf.pin_config.scl = D11; // I2C1 SCL
  i2c_conf.pin_config.sda = D12; // I2C1 SDA

  // Initialize I2C handle
  i2c.Init(i2c_conf);

  System::Delay(100);

  // Initialize MPU6050
  initMPU6050();

  hw.PrintLine("Daisy Seed + MPU6050 Started!");

  int16_t accelX, accelY, accelZ;
  int16_t gyroX, gyroY, gyroZ;

  while(1) {
    // Read IMU data
    readMPU6050(accelX, accelY, accelZ, gyroX, gyroY, gyroZ);

    // Print to console
    hw.PrintLine("Accel: X=%d Y=%d Z=%d | Gyro: X=%d Y=%d Z=%d",
                 accelX, accelY, accelZ, gyroX, gyroY, gyroZ);

    System::Delay(100); // Read every 100ms
  }
}