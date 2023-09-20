/** 
 * Set all the servos to 90 degrees.
*/
#include <Servo.h>

#define SERVO_PITCH 9
#define SERVO_ROLL 10

Servo roll_servo, pitch_servo;

void setup() {
    roll_servo.attach(SERVO_ROLL);
    pitch_servo.attach(SERVO_PITCH);

    // Center the servos.
    roll_servo.write(90);
    pitch_servo.write(90);
}

void loop() {
}