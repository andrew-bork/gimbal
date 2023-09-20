#ifndef MPU6050_INCLUDE_GUARD
#define MPU6050_INCLUDE_GUARD

#include <Wire.h>
#include "./math.hpp"


#define DEG_TO_RAD 0.017453292519943
#define RAD_TO_DEG 57.29577951308232

/**
    From a previous project https://github.com/andrew-bork/autonomous/blob/main/include/backend/mpu6050.h 
    Register map: https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf

    Register stuff:
*/

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

#define SIGNED_16_BIT_MAX 0x7fff

// Combine two unsigned bytes into a single 16 bit signed integer with two's complement
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
                    accel_scale = 16 * 9.81 / SIGNED_16_BIT_MAX;
                    break;
                case g_8: 
                    accel_scale = 8 * 9.81 / SIGNED_16_BIT_MAX;
                    break;
                case g_4: 
                    accel_scale = 4 * 9.81 / SIGNED_16_BIT_MAX;
                    break;
                case g_2: 
                    accel_scale = 2 * 9.81 / SIGNED_16_BIT_MAX;
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
                    gyro_scale = 250 * DEG_TO_RAD / SIGNED_16_BIT_MAX;
                    break;
                case deg_500:
                    gyro_scale = 500 * DEG_TO_RAD / SIGNED_16_BIT_MAX;
                    break;
                case deg_1000:
                    gyro_scale = 1000 * DEG_TO_RAD / SIGNED_16_BIT_MAX;
                    break;
                case deg_2000:
                    gyro_scale = 2000 * DEG_TO_RAD / SIGNED_16_BIT_MAX;
                    break;
            }
            write(REG_GYRO_CFG, read(REG_GYRO_CFG) & (~0b00011000) | (range << 3));
        }

        /**
         * @brief Get the sensor data. Make sure the data array is at least 6 doubles
         * 0 - 2 Ax, Ay, Az (Acclerometer Data) (m/s^2)
         * 3 - 5 Gr, Gp, Gy (Gyroscope Data) (radians/s)
         * @param data An array of 6 doubles
         */
        void get_data(double * data) {
            uint8_t buf[14]; // 0-5 Accelerometer 
                            // 6-7 Temp 
                            // 8-13 Gyro 
            
            read_burst(OUT_XACCL_H, buf, 14); // All registers are in order. Just burst read them all.

            // Combine, convert to signed, and scale.
            data[0] = (((double) combine(buf[0], buf[1]))) * accel_scale;
            data[1] = (((double) combine(buf[2], buf[3]))) * accel_scale;
            data[2] = (((double) combine(buf[4], buf[5]))) * accel_scale;

            data[3] = (((double) combine(buf[8], buf[9]))) * gyro_scale;
            data[4] = (((double) combine(buf[10], buf[11]))) * gyro_scale;
            data[5] = (((double) combine(buf[12], buf[13]))) * gyro_scale;
        }

        /**
         * @brief Get the sensor data. Data is nicely loaded into two vectors.
         * 
         * @param acceleration (m/s^2)
         * @param gyroscope (radians/s)
         */
        void get_data(math::vector& acceleration, math::vector& gyroscope) {
            uint8_t buf[14]; // 0-5 Accelerometer 
                            // 6-7 Temp 
                            // 8-13 Gyro 
            
            read_burst(OUT_XACCL_H, buf, 14); // All registers are in order. Just burst read them all.

            // Combine, convert to signed, and scale.
            acceleration.x = (((double) combine(buf[0], buf[1]))) * accel_scale;
            acceleration.y = (((double) combine(buf[2], buf[3]))) * accel_scale;
            acceleration.z = (((double) combine(buf[4], buf[5]))) * accel_scale;

            gyroscope.x = (((double) combine(buf[8], buf[9]))) * gyro_scale;
            gyroscope.y = (((double) combine(buf[10], buf[11]))) * gyro_scale;
            gyroscope.z = (((double) combine(buf[12], buf[13]))) * gyro_scale;
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

#endif