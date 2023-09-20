# Arduino/MPU6050 Gimbal

  A simple project to keep a "flag" upright using an Arduino, an IMU (mpu6050) and a servo.


    To run, open gimbal-two-axis/gimbal-two-axis.ino in the Arduino IDE.


## gimbal-two-axis

  This sketch should be able control up to two servos in order to keep a "flag" upright. It uses quarternion math
  in order to store rotation, while fusing both the accelerometer and gyroscope sensor.
  
  This sketch is able to accurately determine pitch and roll of the system.

## zero-servos
  This sketch just zeroes the servo(s). Facilitates the mounting of the brackets.

## calibrate
  This sketch runs a calibration script, outputting a set of calibration parameters.

  Output looks like this:
  > -0.05, -0.19, -1.74, 0.11, -0.05, 0.01,

  Simply replace the mpu.set_offsets in the gimbal-two-axis sketch with your calibration output:
  ```cpp
  mpu.set_offsets(-0.05, -0.19, -1.74, 0.11, -0.05, 0.01);
  ```

## docs
    Contains the datasheet for the MPU6050