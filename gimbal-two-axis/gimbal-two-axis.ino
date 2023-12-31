#include <Wire.h>
#include <Servo.h>

#include "./mpu6050.hpp"
#include "./math.hpp"

#define SERVO_PITCH 9
#define SERVO_ROLL 10

/**
 * @brief Complementary filter factor. Between 0 and 1.
 * 
 * 1 is full gyroscope.
 * 0 is full accelerometer.
 */
double tau = 0.9; 

/**
 * @brief Update rate in hz.
 * 
 */
double update_rate = 24;

/**
 * @brief The allowed difference between measured acceleration and gravity in order to calculate absolute orientation.
 * 
 */
double a_tolerance = 2;
double a_tolerance_squared = a_tolerance_squared * a_tolerance_squared;

double g = 9.8;
double g_squared = g*g;


int ms = (int) (1000 / update_rate); // delay
double dt = 1.0 / update_rate;  // dt. we love calculus.

#define min(a, b) ((a > b) ? b : a)
#define max(a, b) ((a > b) ? a : b)

#define clamp(val, MIN, MAX) (min(max(val, MIN), MAX))

Servo roll_servo, pitch_servo;
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

    // USE THE CALIBRATE PROGRAM TO GET THE OFFSETS
    mpu.set_offsets(-0.05, -0.19, -1.74, 0.11, -0.05, 0.01);
    // Serial.println("Calibrating mpu6050");
    // mpu.calibrate(100, 24); // 100 / 24 about 4 secs to calibrate. Make sure to keep still.
    // Serial.println("Calibration finished!");

    roll_servo.attach(SERVO_ROLL);
    pitch_servo.attach(SERVO_PITCH);

    // Center the servos.
    roll_servo.write(90);
    pitch_servo.write(90);
}


math::vector calculate_roll_pitch(const math::vector& acceleration) {
    math::vector euler;
    euler.x = atan2(acceleration.y, acceleration.z);
    euler.y = atan2(-acceleration.x, sqrt(acceleration.y * acceleration.y + acceleration.z * acceleration.z));
    return euler;
}

math::quarternion orientation = math::quarternion::fromEulerZYX(math::vector(0, 0, 0));

void loop() {
    math::vector acceleration;
    math::vector angular_velocity;

    mpu.get_data(acceleration, angular_velocity);

    double a_mag = math::length(acceleration);


    // Integrate angular velocity reading.
    math::quarternion gyro_quarternion = math::quarternion::fromEulerZYX(angular_velocity*dt);
    orientation = gyro_quarternion * orientation;
    
    math::vector orientation_euler = math::quarternion::toEuler(orientation);
    math::vector accelerometer_orientation;

    if(abs(a_mag - g) < a_tolerance) {
        // If the acceleration seems in range, use it to calculate orientation.
        accelerometer_orientation = calculate_roll_pitch(acceleration);

        // Apply a complementary filter
        orientation_euler = accelerometer_orientation * tau + orientation_euler * (1 - tau);

        // Convert the fused angles back into the quarternion.
        orientation = math::quarternion::fromEulerZYX(orientation_euler);
    }


    { // Drive the servos.
        int roll_servo_drive = clamp((int) (90 - orientation_euler.x * RAD_TO_DEG), 0, 180);
        roll_servo.write(roll_servo_drive);

        int pitch_servo_drive = clamp((int) (90 - -1 * orientation_euler.y * RAD_TO_DEG), 0, 180);
        pitch_servo.write(pitch_servo_drive);
    }
    
    
    { // log
        // Serial.print("roll:");
        // Serial.print(orientation_euler.x * RAD_TO_DEG);

        // Serial.print(", pitch:");
        // Serial.print(orientation_euler.y * RAD_TO_DEG);

        // Serial.print(", a_roll:");
        // Serial.print(accelerometer_orientation.x * RAD_TO_DEG);

        // Serial.print(", a_pitch:");
        // Serial.print(accelerometer_orientation.y * RAD_TO_DEG);

        // Serial.print(", g_vroll:");
        // Serial.print(angular_velocity.x * RAD_TO_DEG);

        // Serial.print(", g_vpitch:");
        // Serial.print(angular_velocity.y * RAD_TO_DEG);

        // Serial.print(", accel:");
        // Serial.print(a_mag);
        // Serial.print(", Diffrenece_g:");
        // Serial.print(abs(a_mag - g));
        // Serial.print(", tolerance:");
        // Serial.print(a_tolerance);

        // Serial.print("A:");
        // Serial.print(a_mag);

        // Serial.print(", Ax:");
        // Serial.print(acceleration.x);
        // Serial.print(",Ay:");
        // Serial.print(acceleration.y);
        // Serial.print(",Az:");
        // Serial.print(acceleration.z);
        
        // Serial.print(",Vr:");
        // Serial.print(angular_velocity.x);
        // Serial.print(",Vp:");
        // Serial.print(angular_velocity.y);
        // Serial.print(",Vy:");
        // Serial.print(angular_velocity.z);

        Serial.println();
    }



    delay(ms);
}