#include <Wire.h>
#include <Servo.h>

#include "./mpu6050.hpp"
#include "./math.hpp"


mpu6050 mpu;

void setup() {
    Serial.begin(19200);
    
    Wire.begin(); // Start I2C connection.

    mpu.wake_up(); // WAKE UP WAKE UP
    mpu.set_accel_range(mpu6050::g_2); // Set accelerometer range
    mpu.set_gyro_range(mpu6050::deg_1000); // Set gyro range
    mpu.set_clock(mpu6050::y_gyro); // Set clock to y gyro
    mpu.set_dlpf(mpu6050::hz_21); // Set lowpass filter

    delay(1000);
    Serial.println("Calibrating mpu6050");
    mpu.calibrate(100, 24); // 100 / 24 about 4 secs to calibrate. Make sure to keep still.
    Serial.println("Calibration finished!");
}

void loop() {

}