// Pedal IMU firmware for Seeed XIAO nRF52840 Sense.
//
// Reads the on-board LSM6DS3TR-C accelerometer at 100 Hz and emits one
// CSV line per sample over USB-CDC: "ax,ay,az\n" in units of g.
//
// Host (skynet, pedal_teleop) does all filtering, gravity-vector
// projection, axis-sign mapping, and deadband. This sketch is the
// dumbest possible firmware on purpose.
//
// Library: Seeed_Arduino_LSM6DS3
//   https://github.com/Seeed-Studio/Seeed_Arduino_LSM6DS3
//
// Flash via UF2: double-tap RESET on the XIAO, drag the compiled .uf2
// onto the mounted XIAO-BOOT volume. Identical binary on both pedals.

#include "LSM6DS3.h"
#include "Wire.h"

LSM6DS3 imu(I2C_MODE, 0x6A);

const unsigned long PERIOD_US = 10000UL; // 100 Hz
unsigned long next_us = 0;

void setup() {
  Serial.begin(115200);
  // Don't block on Serial — pedals must work whether or not a host is attached.
  while (imu.begin() != 0) {
    delay(100);
  }
  next_us = micros();
}

void loop() {
  unsigned long now = micros();
  if ((long)(now - next_us) < 0) return;
  next_us += PERIOD_US;
  // If we fell badly behind (e.g., host paused), resync rather than spin.
  if ((long)(micros() - next_us) > (long)(5 * PERIOD_US)) {
    next_us = micros() + PERIOD_US;
  }

  float ax = imu.readFloatAccelX();
  float ay = imu.readFloatAccelY();
  float az = imu.readFloatAccelZ();

  Serial.print(ax, 6);
  Serial.print(',');
  Serial.print(ay, 6);
  Serial.print(',');
  Serial.println(az, 6);
}
