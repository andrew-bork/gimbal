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
double a_tolerance = 20;
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
    mpu.set_accel_range(mpu6050::g_8); // Set accelerometer range
    mpu.set_gyro_range(mpu6050::deg_2000); // Set gyro range
    mpu.set_clock(mpu6050::y_gyro); // Set clock to y gyro
    mpu.set_dlpf(mpu6050::hz_5); // Set lowpass filter


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

math::quarternion orientation;

void loop() {
    math::vector acceleration;
    math::vector angular_velocity;

    mpu.get_data(acceleration, angular_velocity);

    double a_mag = math::square_length(acceleration);


    // Integrate angular velocity reading.
    math::quarternion gyro_quarternion = math::quarternion::fromEulerZYX(angular_velocity * dt);
    orientation = gyro_quarternion * orientation;
    
    math::vector orientation_euler = math::quarternion::toEuler(orientation);
    math::vector accelerometer_orientation;

    if(abs(a_mag - g_squared) < a_tolerance_squared) {
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
        Serial.print("roll:");
        Serial.print(orientation_euler.x * RAD_TO_DEG);

        Serial.print(", pitch:");
        Serial.print(orientation_euler.y * RAD_TO_DEG);

        Serial.print(", a_roll:");
        Serial.print(accelerometer_orientation.x * RAD_TO_DEG);

        Serial.print(", a_pitch:");
        Serial.print(accelerometer_orientation.y * RAD_TO_DEG);

        Serial.print(", g_vroll:");
        Serial.print(angular_velocity.x * RAD_TO_DEG);

        Serial.print(", g_vpitch:");
        Serial.print(angular_velocity.y * RAD_TO_DEG);
        Serial.print(", accel:");
        Serial.print(sqrt(a_mag));
        // Serial.print(", Diffrenece_g:");
        // Serial.print(abs(a_mag - g_squared));

        Serial.print(", A_tolerance:");
        Serial.print(a_tolerance);

        Serial.println();
    }



    delay(ms);
}