#include <Wire.h>
#include <Servo.h>

/**
 * Parameters
 * 
 */
double tau = 0.01; 

double update_rate = 100;

double g = 9.8;
double g_squared = g*g;

double a_tolerance = 50;*/


int ms = (int) (1000 / update_rate);
double dt = 1.0 / update_rate; 

/**
    From a previous project https://github.com/andrew-bork/autonomous/blob/main/include/backend/mpu6050.h 
    Register map: https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf#%5B%7B%22num%22%3A24%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C62%2C613%2C0%5D

    Register stuff:
*/
#define RAD_T_DEG 57.29577951308 //Radians to degrees (180/PI)
#define DEG_T_RAD 0.0174533
#define DEG_TO_RAD 0.017453292519943
#define RAD_TO_DEG 57.29577951308232

#define MPU6050_DEFAULT_ADDR 0x68

#define REG_PWR_MNG_1 0x6B // Register map 4.28
#define REG_PWR_MNG_2 0x6C // Register map 4.29
#define REG_CFG 0x1A
#define REG_GYRO_CFG 0x1B
#define REG_ACCL_CFG 0x1C
#define REG_FIFO_EN 0x23
#define REG_ACCL_OUT_STRT 0x3B
#define REG_TEMP_OUT_STRT 0x41
#define REG_GYRO_OUT_STRT 0x43
#define REG_SIG_PTH_RST 0x68 //Signal Path Reset
#define REG_USR_CTRL 0x6A

#define OUT_XACCL_H 0x3B
#define OUT_XACCL_L 0x3C
#define OUT_YACCL_H 0x3D
#define OUT_YACCL_L 0x3E
#define OUT_ZACCL_H 0x3F
#define OUT_ZACCL_L 0x40
#define OUT_TEMP_H 0x41
#define OUT_TEMP_L 0x42
#define OUT_XGYRO_H 0x43
#define OUT_XGYRO_L 0x44
#define OUT_YGYRO_H 0x45
#define OUT_YGYRO_L 0x46
#define OUT_ZGYRO_H 0x47
#define OUT_ZGYRO_L 0x48

#define X_ACCL_SHIFT 1265
#define Y_ACCL_SHIFT 215
#define Z_ACCL_SHIFT 1066
#define X_GYRO_SHIFT 65437
#define Y_GYRO_SHIFT 72
#define Z_GYRO_SHIFT 2337

// Combine two unsigned bytes into 1 16 bit signed integer with two's complement
#define combine(msb, lsb) ((((int16_t) msb) << 8) | lsb)

/**
 * @brief An encapsulation of the mpu6050.
 * From a previous project https://github.com/andrew-bork/autonomous/blob/main/include/backend/mpu6050.h 
 */
class mpu6050 {
    private: 
        
        /**
         * @brief Write a byte to a register
         * 
         * @param reg Register to write to.
         * @param byte Value to write.
         */
        void write(uint8_t reg, uint8_t byte) {
            Wire.beginTransmission(addr);
            Wire.write(reg);
            Wire.write(byte);
            Wire.endTransmission();
        }
        
        /**
         * @brief Read n bytes, starting from reg and then proceeding upwards. 
         *  Some ICs may wrap around, meaning a burst read will not always read towards higher memory.
         * 
         * @param reg Register to start reading from
         * @param bytes Desination Buffer
         * @param n Number of bytes to read. 
         * @return 
         */
        void read_burst(uint8_t reg, uint8_t * bytes, uint8_t n) {
            Wire.beginTransmission(addr);
            Wire.write(reg);
            Wire.endTransmission(false);

            Wire.requestFrom((int) addr, (int) n);
            Wire.readBytes(bytes, n);
        }
        
        /**
         * @brief Read a byte from a register
         * 
         * @param reg Register to read from.
         * @return uint8_t Value of that register
         */
        uint8_t read(uint8_t reg) {
            Wire.beginTransmission(addr);
            Wire.write(reg);
            Wire.endTransmission(false);

            Wire.requestFrom((int) addr, (int) 1);
            //while(Wire.available() < 1) { delay(10); } // Wait until one byte is availiable
            return Wire.read();
        }

        uint8_t addr; // Device address. Default: 0x68

        double accel_scale = 1.0, gyro_scale = 1.0; // Set based on the range of the accelerometer and gyro.
    public:
        /**
         * @brief Used with set_accel_range. g_2 is a range of 2*g and so on.
         * 
         */
        enum accel_range {
            g_2 = 0b00,
            g_4 = 0b01,
            g_8 = 0b10,
            g_16 = 0b11, 
        };

        /**
         * @brief Used with set_gyro_range. deg_250 is a range of 250 degrees and so on.
         * 
         */
        enum gyro_range {
            deg_250 = 0,
            deg_500 = 1,
            deg_1000 = 2,
            deg_2000 = 3,
        };

        /**
         * @brief Used with set_dlpf. hz_260 is a low pass cutoff of around 260 hz (check the datasheet)
         * 
         */
        enum dlpf_setting {
            hz_260 = 0,
            hz_184 = 1,
            hz_94 = 2,
            hz_44 = 3,
            hz_21 = 4,
            hz_10 = 5,
            hz_5 = 6,
        };
        
        /**
         * @brief Clock settings. Check the datasheet. Usually set to y_gyro.
         * 
         */
        enum clock_setting {
            int_oscl,
            x_gyro,
            y_gyro,
            z_gyro,
            ext_32kHz,
            ext_19MHz,
            reserved,
            stop
        };
        
        /**
         * @brief Construct a new mpu6050 object
         * 
         * @param _addr Address of device. Defaults to 0x68
         */
        mpu6050(uint8_t _addr = MPU6050_DEFAULT_ADDR) : addr(_addr) {
            
        }

        /**
         * @brief Wake up the mpu6050. Clears the sleep bit of the REG_PWR_MNG_1
         * 
         */
        void wake_up() {
            write(REG_PWR_MNG_1, read(REG_PWR_MNG_1) & (~0b01000000) ); // Clear the sleep bit to wake up
        }
        /**
         * @brief Set the clock setting of the mpu6050
         * 
         * @param setting
         */
        void set_clock(clock_setting setting) {
            write(REG_PWR_MNG_1,( read(REG_PWR_MNG_1) & (~0b00000111)) | setting); // CLKSEL bits 0-2
        }

        /**
         * @brief Set the low pass filter setting.
         * 
         * @param setting 
         */
        void set_dlpf(dlpf_setting setting) {
            write(REG_CFG, (read(REG_CFG) & (~0b00000111)) | setting); // CLKSEL bits 0-2
        }

        /**
         * @brief Set the accelerometer range. 
         * 
         * @param range 
         */
        void set_accel_range(accel_range range) {
            switch(range){
                case g_16:
                    accel_scale = 16 * 9.81;
                    break;
                case g_8: 
                    accel_scale = 8 * 9.81;
                    break;
                case g_4: 
                    accel_scale = 4 * 9.81;
                    break;
                case g_2: 
                    accel_scale = 2 * 9.81;
                    break;
            }
            write(REG_ACCL_CFG, read(REG_ACCL_CFG) & (~0b00011000) | (range << 3));
        }

        /**
         * @brief Set the gyroscope range.
         * 
         * @param range 
         */
        void mpu6050::set_gyro_range(gyro_range range){
            switch(range){
                case deg_250:
                    gyro_scale = 250 * RAD_T_DEG;
                    break;
                case deg_500:
                    gyro_scale = 500 * RAD_T_DEG;
                    break;
                case deg_1000:
                    gyro_scale = 1000 * RAD_T_DEG;
                    break;
                case deg_2000:
                    gyro_scale = 2000 * RAD_T_DEG;
                    break;
            }
            write(REG_GYRO_CFG, read(REG_GYRO_CFG) & (~0b00011000) | (range << 3));
        }

        /**
         * @brief Get the sensor data. Make sure the data array is at least 6 doubles
         * 0 - 2 Ax, Ay, Az (Acclerometer Data) (m/s)
         * 3 - 5 Gr, Gp, Gy (Gyroscope Data) (radians)
         * @param data An array of 6 doubles
         */
        void get_data(double * data) {
            uint8_t buf[14]; // 0-5 Accelerometer 
                                                // 6-7 Temp 
                                                // 8-13 Gyro 
            
            read_burst(OUT_XACCL_H, buf, 14); // All registers are in order. Just burst read them all.


            // Combine, convert to signed, and scale.
            data[0] = (((double) combine(buf[0], buf[1])) / 0x7fff) * accel_scale;
            data[1] = (((double) combine(buf[2], buf[3])) / 0x7fff) * accel_scale;
            data[2] = (((double) combine(buf[4], buf[5])) / 0x7fff) * accel_scale;

            data[3] = (((double) combine(buf[8], buf[9])) / 0x7fff) * gyro_scale;
            data[4] = (((double) combine(buf[10], buf[11])) / 0x7fff) * gyro_scale;
            data[5] = (((double) combine(buf[12], buf[13])) / 0x7fff) * gyro_scale;
        }
        
        /**
         * @brief Debug function. Gets the value of a register. An alias for read
         * 
         * @param reg Register to read
         * @return uint8_t Value of the Register.
         */
        uint8_t query_register(uint8_t reg) {
            return read(reg);
        }
};

/**
    Pulled from an older project https://github.com/andrew-bork/autonomous/blob/main/include/math/math.h 
    A set of complicated math objects.
*/
namespace math {

        /**
         * @brief 3D vector.
         * 
         */
        struct vector{
                double x,y,z;

                /**
                 * @brief Construct a zero vector.
                 * 
                 */
                vector();
                /**
                 * @brief Construct a new vector: [x, 0, 0]
                 * 
                 * @param x 
                 */
                vector(double x);
                /**
                 * @brief Construct a new vector: [x, y, 0]
                 * 
                 * @param x 
                 * @param y 
                 */
                vector(double x, double y);
                /**
                 * @brief Construct a new vector: [x, y, z]
                 * 
                 * @param x 
                 * @param y 
                 * @param z 
                 */
                vector(double x, double y, double z);

                /**
                 * @brief Vector addition.
                 * 
                 * @param r 
                 * @return vector 
                 */
                vector operator+ (const vector& r);
                /**
                 * @brief Vector scalar scaling.
                 * 
                 * @param s 
                 * @return vector 
                 */
                vector operator* (const double& s);
        };

        /**
         * @brief QUARETNERIOSN. 
         * Basically a better way to represent rotations.
         * https://en.wikipedia.org/wiki/Quaternion (Have fun)
         *  Q2 * Q1 -> rotation Q1 is applied before rotation Q2.
         */
        struct quarternion{
                double w, x, y, z;
                bool unit; // Unit quarternion flag.

                /**
                 * @brief Construct a zero quarternion
                 * 
                 */
                quarternion();
                /**
                 * @brief Construct a new quarternion object (x, y, z)
                 * 
                 * @param x 
                 * @param y 
                 * @param z 
                 */
                quarternion(double x, double y, double z);
                /**
                 * @brief Construct a new quarternion object (w, x, y, z)
                 * 
                 * @param w 
                 * @param x 
                 * @param y 
                 * @param z 
                 */
                quarternion(double w, double x, double y, double z);
                /**
                 * @brief Construct a new quarternion object (w, x, y, z, unit)
                 * 
                 * @param w 
                 * @param x 
                 * @param y 
                 * @param z 
                 * @param unit 
                 */
                quarternion(double w, double x, double y, double z, bool unit);

                /**
                 * @brief Element-wise addition.
                 * 
                 * @param r 
                 * @return quarternion 
                 */
                quarternion operator+ (const quarternion& r);
                /**
                 * @brief Quarternion multiplicatoin. "Rotation"
                 * Look at the wikipedia article.
                 * r will get applied before this.
                 * 
                 * @param r this rotation will get applied first
                 * @return quarternion 
                 */
                quarternion operator* (const quarternion& r);

                /**
                 * @brief Invert the quarternion. Check wikipedia
                 * 
                 * @param n 
                 * @return quarternion 
                 */
                static quarternion inverse(const quarternion& n);
                /**
                 * @brief Get the quareternion conjugate. Used for rotations. Check Wikipedia.
                 * 
                 * @param n 
                 * @return quarternion 
                 */
                static quarternion conjugate(const quarternion& n);
                /**
                 * @brief Get a new quarternion that represents the rotation around an axis by an angle theta.
                 * 
                 * @param theta 
                 * @param axis 
                 * @return quarternion 
                 */
                static quarternion rotateAxis(double theta, const vector& axis);
                /**
                 * @brief Get a new quarternion that represents euler angles.
                 * x - roll
                 * y - pitch
                 * z - yaw
                 * The rotations are applied in the order Z-Y-X
                 * 
                 * @param euler 
                 * @return quarternion 
                 */
                static quarternion fromEulerZYX(const vector& euler);
                
                /**
                 * @brief Convert a quarternion back into euler angles
                 * x - roll
                 * y - pitch
                 * z - yaw
                 * The rotations are applied in the order Z-Y-X
                 * 
                 * @param q 
                 * @return vector 
                 */
                static vector toEuler(const quarternion& q);
                /**
                 * @brief Convert a quarternion into a vector. The vector's direction is the rotation axis and the vector's magnitude is the rotation angle.
                 * 
                 * @param q 
                 * @return vector 
                 */
                static vector toMagAxis(const quarternion& q);

                /**
                 * @brief Rotate a vector using a quarternion.
                 * v' = p * in * p'
                 * Look at the wikipedia for more details.
                 * 
                 * @param q 
                 * @param in 
                 * @return vector 
                 */
                static vector rotateVector(math::quarternion& q, math::vector& in);
        };
        
        /**
         * @brief Magnitude of a quarternion.
         * 
         * @param q 
         * @return double 
         */
        double length(const math::quarternion& q);
        /**
         * @brief Magnitude (length) of a vector.
         * 
         * @param v 
         * @return double 
         */
        double length(const math::vector& v);
}

Servo rollServo, pitchServo;

mpu6050 mpu;
void setup(void) {
    Serial.begin(19200);
    
    Wire.begin(); // Start I2C connection.

    mpu.wake_up(); // WAKE UP WAKE UP
    mpu.set_accel_range(mpu6050::g_2); // Set accelerometer range
    mpu.set_gyro_range(mpu6050::deg_250); // Set gyro range
    mpu.set_clock(mpu6050::y_gyro); // Set clock to y gyro
    mpu.set_dlpf(mpu6050::hz_5); // Set lowpass filter


    // Switch servos if needed.
    if(switchServos) {
        rollServo.attach(9);
        pitchServo.attach(10);
    }else {
        rollServo.attach(10);
        pitchServo.attach(9);
    }

    // Center the servos.
    rollServo.write(90);
    pitchServo.write(90);
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

    // Integrate gyro
    orientation = gyro_quarternion * orientation;
    
    math::vector orientation_euler = math::quarternion::toEuler(orientation);
    
    if(abs(a_mag - grav_squared) < a_tolerance) {
        // If the acceleration seems in range, use it to calculate orientation.
        orientation_euler.x = accelerometer_roll * tau + (1 - tau) * orientation_euler.x;
        orientation_euler.y = accelerometer_pitch * tau + (1 - tau) * orientation_euler.y;

        orientation = math::quarternion::fromEulerZYX(orientation_euler);
    }

    int roll_servo_drive = (int) (90 - orientation_euler.x * RAD_TO_DEG);
    if(roll_servo_drive >= 0 && roll_servo_drive <= 180) rollServo.write(roll_servo_drive);

    int pitch_servo_drive = (int) (90 - -1 * orientation_euler.y * RAD_TO_DEG);
    if(pitch_servo_drive >= 0 && pitch_servo_drive <= 180) pitchServo.write(pitch_servo_drive);
    

    // log
    Serial.print("roll:");
    Serial.print(orientation_euler.x);

    Serial.print(", pitch:");
    Serial.print(orientation_euler.y);

    Serial.println();

    delay(10);
}










/**
 * IMPLEMENTATIONS OF MATH STUFF.
 * 
 */

math::quarternion::quarternion(){
    w = x = y = z = 0;
    unit = false;
}

math::quarternion::quarternion(double x, double y, double z){
    w = 0;
    this->x = x;
    this->y = y;
    this->z = z;
    unit = false;
}
math::quarternion::quarternion(double w, double x, double y, double z){
    this->w = w;
    this->x = x;
    this->y = y;
    this->z = z;
    unit = false;
}

math::quarternion::quarternion(double w, double x, double y, double z, bool unit){
    this->w = w;
    this->x = x;
    this->y = y;
    this->z = z;
    this->unit = unit;
}



math::quarternion math::quarternion::operator*(const math::quarternion& r){
    quarternion res;
    res.w = w*r.w - x*r.x - y*r.y - z*r.z;
    res.x = r.w*x + r.x*w + r.y*z - r.z*y;
    res.y = r.w*y + r.y*w + r.z*x - r.x*z;
    res.z = r.w*z + r.z*w + r.x*y - r.y*x;
    return res;
}

math::quarternion math::quarternion::operator+(const math::quarternion& r){
    quarternion res;
    res.w = r.w + w;
    res.x = r.x + x;
    res.y = r.y + y;
    res.z = r.z + z;
    return res;
}

math::quarternion math::quarternion::conjugate(const math::quarternion& q){
    quarternion res;
    res.w = q.w;
    res.x = -q.x;
    res.y = -q.y;
    res.z = -q.z;
    return res;
}

double math::length(const math::quarternion& q){
    return sqrt(q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z);
}

double math::length(const math::vector& v){
    return sqrt(v.x*v.x+v.y*v.y+v.z*v.z);
}

math::quarternion math::quarternion::inverse(const math::quarternion& q){
    double len;
    if(q.unit) len = 1;
    else len = length(q);
    quarternion res;
    res.w = q.w;
    res.x = -q.x;
    res.y = -q.y;
    res.z = -q.z;
    return res;
}
math::quarternion math::quarternion::rotateAxis(double theta, const math::vector& axis){
    quarternion res;
    res.w = cos(theta/2);
    res.x = axis.x*sin(theta/2);
    res.y = axis.y*sin(theta/2);
    res.z = axis.z*sin(theta/2);
    return res;
}

math::quarternion math::quarternion::fromEulerZYX(const math::vector& euler){
    double cy = cos(euler.z*0.5);
    double sy = sin(euler.z*0.5);
    double cp = cos(euler.y*0.5);
    double sp = sin(euler.y*0.5);
    double cr = cos(euler.x*0.5);
    double sr = sin(euler.x*0.5);

    quarternion res;
    res.w = cr*cp*cy + sr*sp*sy;
    res.x = sr*cp*cy - cr*sp*sy;
    res.y = cr*sp*cy + sr*cp*sy;
    res.z = cr*cp*sy - sr*sp*cy;
    return res;
}

math::vector math::quarternion::toEuler(const math::quarternion& q){
    double  sin_p = 2*(q.w*q.y - q.z*q.x);
    if(sin_p >=1){
        vector res(-2*atan2(q.x, q.w), 1.570796326794897, 0);
        return res;
    }else if(sin_p <= -1){
        vector res(2 *atan2(q.x, q.w), -1.570796326794897,  0);
        return res;
    }
    //asin(2*(q.w*q.y - q.z*q.x));
    vector res(atan2(2*(q.w*q.x+q.y*q.z), 1 - 2* (q.x*q.x + q.y*q.y)), asin(sin_p), atan2(2*(q.w*q.z+q.x*q.y), 1 - 2 * (q.y*q.y+q.z*q.z)));
    return res;
}

math::vector math::quarternion::toMagAxis(const math::quarternion& q){
    double s = 1 - q.w*q.w;
    double mag = acos(q.w)*2;

    if(q.w > -0.00001 && q.w < 0.00001){
            vector res(0,0,0);
            return res;
    }
    vector res(q.x*mag/s, q.y*mag/s,q.z*mag/s);
    return res;
}

math::vector::vector (){
    x = y = z = 0;
}
math::vector::vector (double _x, double _y){
    x = _x;
    y = _y;
    z = 0;
}
math::vector::vector (double _x, double _y, double _z){
    x = _x;
    y = _y;
    z = _z;
}

math::vector math::vector::operator+ (const math::vector& r){
    vector res(x+r.x,y+r.y,z+r.z);
    return res;
}

math::vector math::vector::operator*(const double& s){
    vector res(x*s, y*s, z*s);
    return res;
}

math::vector math::quarternion::rotateVector(math::quarternion& q, math::vector& r){
    quarternion temp(0, r.x, r.y, r.z);
    temp = q * temp * quarternion::conjugate(q);
    vector out = math::vector(temp.x, temp.y, temp.z);
    return out;
}
