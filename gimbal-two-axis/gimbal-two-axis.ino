#include <Wire.h>
#include <Servo.h>

#include "./mpu6050.hpp"
#include "./math.hpp"

/**
 * Parameters
 * 
 */

double tau = 0.5; 

double update_rate = 100;

double g = 9.8;
double g_squared = g*g;

double a_tolerance = 50;


int ms = (int) (1000 / update_rate);
double dt = 1.0 / update_rate; 

#define SERVO_PITCH 9
#define SERVO_ROLL 10

#define min(a, b) ((a > b) ? b : a)
#define max(a, b) ((a > b) ? a : b)

#define clamp(val, MIN, MAX) (min(max(val, MIN), MAX))

Servo roll_servo, pitch_servo;
mpu6050 mpu;

void setup(void) {
    Serial.begin(19200);
    
    Wire.begin(); // Start I2C connection.

    mpu.wake_up(); // WAKE UP WAKE UP
    mpu.set_accel_range(mpu6050::g_2); // Set accelerometer range
    mpu.set_gyro_range(mpu6050::deg_1000); // Set gyro range
    mpu.set_clock(mpu6050::y_gyro); // Set clock to y gyro
    mpu.set_dlpf(mpu6050::hz_5); // Set lowpass filter


    roll_servo.attach(SERVO_SERVO);
    pitch_servo.attach(SERVO_PITCH);

    // Center the servos.
    roll_servo.write(90);
    pitch_servo.write(90);
}

math::quarternion orientation;

void loop() {
    double data[6];
    mpu.get_data(data);

    double a_mag = data[0] * data[0] + data[1] * data[1] + data[2] * data[2];
    double accelerometer_roll = atan2 (data[1], data[2]);
    double accelerometer_pitch = atan2 (-data[0], sqrt(data[1] * data[1] + data[2] * data[2]));

    math::vector gyro_vector = math::vector(data[3] * dt,data[4] * dt,data[5] * dt);
    math::quarternion gyro_quarternion = math::quarternion::fromEulerZYX(gyro_vector);

    // Integrate angular velocity reading.
    orientation = gyro_quarternion * orientation;
    
    math::vector orientation_euler = math::quarternion::toEuler(orientation);
    
    if(abs(a_mag - g_squared) < a_tolerance) {
        // If the acceleration seems in range, use it to calculate orientation.
        orientation_euler.x = accelerometer_roll * tau + (1 - tau) * orientation_euler.x;
        orientation_euler.y = accelerometer_pitch * tau + (1 - tau) * orientation_euler.y;

        orientation = math::quarternion::fromEulerZYX(orientation_euler);
    }


    int roll_servo_drive = clamp((int) (90 - orientation_euler.x * RAD_TO_DEG), 0, 180);
    roll_servo.write(roll_servo_drive);

    int pitch_servo_drive = clamp((int) (90 - -1 * orientation_euler.y * RAD_TO_DEG), 0, 180);
    pitch_servo.write(pitch_servo_drive);
    

    // log
    Serial.print("roll:");
    Serial.print(orientation_euler.x * RAD_T_DEG);

    Serial.print(", pitch:");
    Serial.print(orientation_euler.y * RAD_T_DEG);

    Serial.print(", a_roll:");
    Serial.print(accelerometer_roll * RAD_T_DEG);

    Serial.print(", a_pitch:");
    Serial.print(accelerometer_pitch * RAD_T_DEG);

    Serial.print(", g_vroll:");
    Serial.print(data[3] * RAD_T_DEG);

    Serial.print(", g_vpitch:");
    Serial.print(data[4] * RAD_T_DEG);

    Serial.println();

    delay(ms);
}