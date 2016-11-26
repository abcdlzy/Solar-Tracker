#include <stdio.h>
#include <wiringPi.h>
#include <stdint.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <math.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>

#define SENSORS_MPU9250_ATTACHED
#define SENSORS_MPU6500_ATTACHED
    #define SENSORS_AK8963_ATTACHED

struct Vector3 {
    union {
        float v[3];
        struct {
            float x;
            float y;
            float z;
        };
    };

    Vector3() : x(0), y(0), z(0) { }
};

class Accelerometer {
public:
    // Earth's gravity in m/s^2
    static const float STANDARD_GRAVITY= 9.80665f;

    // Get the current acceleration vector, in m/s^2
    virtual Vector3 getAcceleration() = 0;
};


class Magnetometer {
public:
    // Gauss to microTesla multiplier
    static const float GAUSS_TO_MICROTESLA= 100.0f;

    // Get the current magnetic field vector, in μT
    virtual Vector3 getMagneticField() = 0;

    // Get the current azimuth (compass direction), optionally adjusting for declination
    float getAzimuth(float declination = 0.0){
        // Get the magnetic field vector from the device, in uT
        Vector3 magneticField = getMagneticField();

        // Calculate the compass heading
        float heading = atan2(magneticField.y, magneticField.x);

        // Adjust the compass heading for local declination (in rads)
        heading += declination;

        // Adjust for overflow
        if(heading < 0)         heading += 2*M_PI;
        if(heading > 2*M_PI)    heading -= 2*M_PI;

        // Return the heading in degrees
        return heading * 180/M_PI;
    }
};



class Barometer {
public:
    // Standard atmosphere, or average sea-level pressure in hPa (millibars)
    static const float PRESSURE_STANDARD_ATMOSPHERE= 1013.25f;

    // Get the current air pressure in hPa
    virtual float getPressure() = 0;

    // Get the current altitude in m, given a baseline pressure in hPa
    float getAltitude(float baselinePressure = PRESSURE_STANDARD_ATMOSPHERE){
        float pressure = getPressure();
        float altitude = 44330 * (1.0 - pow(pressure / baselinePressure, 1 / 5.255));
        return altitude;
    }

    // Get the pressure at sea-level in hPa, given the current altitude in m
    float getSealevelPressure(float altitude){
        float pressure = getPressure();
        return pressure / pow(1.0 - altitude / 44330, 5.255);
    }
};

class Thermometer {
public:
    // Get the current ambient temperature in °C
    virtual float getTemperature() = 0;
};

class Gyroscope {
public:
    // Get the current rotational speed vector, in rad/s
    virtual Vector3 getRotation() = 0;
};

class I2CDevice {
public:
    // Constructor
    I2CDevice(uint8_t address);

    // Perform any required device initialization
    virtual void initialize() = 0;

    // Confirm that this device is actually connected to the I2C bus
    virtual bool testConnection() = 0;

protected:
    // Read data from the specified I2C register on this device
    bool readBit(uint8_t regAddr, uint8_t bitNum, uint8_t *data);
    bool readBits(uint8_t regAddr, uint8_t bitNum, uint8_t length, uint8_t *data);
    bool readByte(uint8_t regAddr, uint8_t *data);
    bool readBytes(uint8_t regAddr, uint8_t length, uint8_t *data);
    bool readWord(uint8_t regAddr, uint16_t *data);
    bool readWords(uint8_t regAddr, uint8_t length, uint16_t *data);

    // Write data to the specified I2C register on this device
    bool writeBit(uint8_t regAddr, uint8_t bitNum, uint8_t data);
    bool writeBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
    bool writeByte(uint8_t regAddr, uint8_t data);
    bool writeBytes(uint8_t regAddr, uint8_t length, uint8_t *data);

    // Platform-independent sleep/delay
    void usleep(unsigned int us);

    // The I2C address for this device
    uint8_t address;

    // Convenient buffer for read operations
    uint8_t buffer[64];

    // I2C device handle (internal)
    int handle;
};

bool I2CDevice::readBit(uint8_t regAddr, uint8_t bitNum, uint8_t *data) {
    uint8_t b;
    bool status = readByte(regAddr, &b);
    *data = b & (1 << bitNum);

    return status;
}

bool I2CDevice::readBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data) {
    bool status = false;
    uint8_t b;
    if ((status = readByte(regAddr, &b))) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        b &= mask;
        b >>= (bitStart - length + 1);
        *data = b;
    }
    return status;
}

bool I2CDevice::readByte(uint8_t regAddr, uint8_t *data) {
    return readBytes(regAddr, 1, data);
}

bool I2CDevice::readWord(uint8_t regAddr, uint16_t *data) {
    return readWords(regAddr, 1, data);
}

bool I2CDevice::readWords(uint8_t regAddr, uint8_t length, uint16_t *data) {
    uint8_t temp[length*2];
    bool status = readBytes(regAddr, length*2, temp);

    for(int i=0; i<length; i++) {
        data[i] = (temp[i*2] << 8) | temp[i*2 + 1];
    }

    return status;
}

bool I2CDevice::writeBit(uint8_t regAddr, uint8_t bitNum, uint8_t data) {
    uint8_t b;
    readByte(regAddr, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));

    return writeByte(regAddr, b);
}

bool I2CDevice::writeBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) {
    uint8_t b;
    if(readByte(regAddr, &b) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1); // shift data into correct position
        data &= mask; // zero all non-important bits in data
        b &= ~(mask); // zero all important bits in existing byte
        b |= data; // combine data with existing byte

        return writeByte(regAddr, b);
    } else {
        return false;
    }
}

bool I2CDevice::writeByte(uint8_t regAddr, uint8_t data) {
    return writeBytes(regAddr, 1, &data);
}

I2CDevice::I2CDevice(uint8_t address) : address(address) {
    handle = wiringPiI2CSetup(address);
}

bool I2CDevice::readBytes(uint8_t regAddr, uint8_t length, uint8_t *data) {
    uint8_t count = i2c_smbus_read_i2c_block_data(handle, regAddr, length, data);

    return (count == length);
}

bool I2CDevice::writeBytes(uint8_t regAddr, uint8_t length, uint8_t *data) {
    uint8_t count = i2c_smbus_write_i2c_block_data(handle, regAddr, length, data);

    return (count == length);
}

void I2CDevice::usleep(unsigned int us) {
    delayMicroseconds(us);
}

// Device info
#define AK8963_ADDRESS_00           0x0C
#define AK8963_ADDRESS_01           0x0D
#define AK8963_ADDRESS_10           0x0E
#define AK8963_ADDRESS_11           0x0F
#define AK8963_DEFAULT_ADDRESS      AK8963_ADDRESS_00
#define AK8963_DEVICE_ID            0x48
#define AK8963_TWAT_US              100

// Register map
enum {
    AK8963_RA_WIA       = 0x00, // R
    AK8963_RA_INFO      = 0x01, // R
    AK8963_RA_ST1       = 0x02, // R
    AK8963_RA_HXL       = 0x03, // R
    AK8963_RA_HXH       = 0x04, // R
    AK8963_RA_HYL       = 0x05, // R
    AK8963_RA_HYH       = 0x06, // R
    AK8963_RA_HZL       = 0x07, // R
    AK8963_RA_HZH       = 0x08, // R
    AK8963_RA_ST2       = 0x09, // R
    AK8963_RA_CNTL1     = 0x0A, // R/W
    AK8963_RA_CNTL2     = 0x0B, // R/W
    AK8963_RA_ASTC      = 0x0C, // R/W
    AK8963_RA_TS1       = 0x0D, // R/W - test registers for shipment test, do not use
    AK8963_RA_TS2       = 0x0E, // R/W - test registers for shipment test, do not use
    AK8963_RA_I2CDIS    = 0x0F, // R/W
    AK8963_RA_ASAX      = 0x10, // R
    AK8963_RA_ASAY      = 0x11, // R
    AK8963_RA_ASAZ      = 0x12  // R
};

// ST1
#define AK8963_ST1_DRDY_BIT     0
#define AK8963_ST1_DOR_BIT      1

// ST2
#define AK8963_ST2_HOFL_BIT     3
#define AK8963_ST2_BITM_BIT     4

// CNTL1
#define AK8963_CNTL1_MODE_BIT     3
#define AK8963_CNTL1_MODE_LEN     4
#define AK8963_CNTL1_BIT_BIT      4

// CNTL1 MODE
#define AK8963_MODE_POWERDOWN           0x0
#define AK8963_MODE_SINGLE              0x1
#define AK8963_MODE_CONTINUOUS_8HZ      0x2
#define AK8963_MODE_EXTERNAL            0x4
#define AK8963_MODE_CONTINUOUS_100HZ    0x6
#define AK8963_MODE_SELFTEST            0x8
#define AK8963_MODE_FUSEROM             0xF

// CNTL1 BIT (resolution)
#define AK8963_BIT_14                   0
#define AK8963_BIT_16                   1

// CNTL2
#define AK8963_CNTL2_SRST               0x01

class AK8963 : public I2CDevice, public Magnetometer {
public:
    static AK8963& getInstance() {
        static AK8963 instance;
        return instance;
    }

    // Initialization
    AK8963(uint8_t address = AK8963_DEFAULT_ADDRESS) : I2CDevice(address) {

    }

    void initialize() {
        // Fetch sensitivity adjustment values from the fuse-rom
        setMode(AK8963_MODE_FUSEROM);
        getSensitivityAdjustment(asa);

        // Enable continuous measurement at maximum resolution
        setMode(AK8963_MODE_CONTINUOUS_100HZ);
        setResolution(AK8963_BIT_16);

        // Calculate the scale factor from the configured resolution
        uint8_t resolution = getResolution();
        scale.x = getScale(asa[0], resolution);
        scale.y = getScale(asa[1], resolution);
        scale.z = getScale(asa[2], resolution);
    }

    bool testConnection() {
        return getDeviceID() == AK8963_DEVICE_ID;
    }

    // Magnetometer
    Vector3 getMagneticField() {
        Vector3 magneticField;

        // Read raw measurement data
        int16_t rawField[3];
        getRawMeasurement(rawField);

        // Apply sensitivity adjustments, scale to get uT
        magneticField.x = rawField[0] * scale.x;
        magneticField.y = rawField[1] * scale.y;
        magneticField.z = rawField[2] * scale.z;

        return magneticField;
    }

    // WIA register
    uint8_t getDeviceID() {
        readByte(AK8963_RA_WIA, buffer);
        return buffer[0];
    }

    // ST1 register
    bool getDataReady() {
        readBit(AK8963_RA_ST1, AK8963_ST1_DRDY_BIT, buffer);
        return buffer[0];
    }

    bool getDataOverrun() {
        readBit(AK8963_RA_ST1, AK8963_ST1_DOR_BIT, buffer);
        return buffer[0];
    }

    // H registers
    void getRawMeasurement(int16_t *rawField) {
        // Read data and mark data reading as finished by also reading the ST2 register
        readBytes(AK8963_RA_HXL, 7, buffer);
        rawField[0] = (((int16_t)buffer[1]) << 8) | buffer[0];
        rawField[1] = (((int16_t)buffer[3]) << 8) | buffer[2];
        rawField[2] = (((int16_t)buffer[5]) << 8) | buffer[4];
    }

    // ST2 register
    bool getOverflowStatus() {
        readBit(AK8963_RA_ST2, AK8963_ST2_HOFL_BIT, buffer);
        return buffer[0];
    }

    bool getOutputBit() {
        readBit(AK8963_RA_ST2, AK8963_ST2_BITM_BIT, buffer);
        return buffer[0];
    }

    // CNTL1 register
    uint8_t getMode() {
        readBits(AK8963_RA_CNTL1, AK8963_CNTL1_MODE_BIT, AK8963_CNTL1_MODE_LEN, buffer);
        return buffer[0];
    }

    void setMode(uint8_t mode) {
        // When user wants to change operation mode, transit to power-down mode
        // first and then transit to other modes. After power-down mode is set, at
        // least 100us(Twat) is needed before setting another mode.
        writeBits(AK8963_RA_CNTL1, AK8963_CNTL1_MODE_BIT, AK8963_CNTL1_MODE_LEN, AK8963_MODE_POWERDOWN);
        usleep(AK8963_TWAT_US);
        writeBits(AK8963_RA_CNTL1, AK8963_CNTL1_MODE_BIT, AK8963_CNTL1_MODE_LEN, mode);
    }

    uint8_t getResolution() {
        readBit(AK8963_RA_CNTL1, AK8963_CNTL1_BIT_BIT, buffer);
        return buffer[0];
    }

    void setResolution(uint8_t resolution) {
        writeBit(AK8963_RA_CNTL1, AK8963_CNTL1_BIT_BIT, resolution);
        scale.x = getScale(asa[0], resolution);
        scale.y = getScale(asa[1], resolution);
        scale.z = getScale(asa[2], resolution);
    }

    // CNTL2 register
    void reset() {
        writeByte(AK8963_RA_CNTL2, AK8963_CNTL2_SRST);
    }

    // ASA registers
    void getSensitivityAdjustment(uint8_t *asa) {
        readBytes(AK8963_RA_ASAX, 3, asa);
    }

protected:
    uint8_t asa[3];
    Vector3 scale;

    float getScale(uint8_t asa, uint8_t resolution) {
        // Get the scale factor from raw to uT, depending on resolution
        float resScale;
        if(resolution == AK8963_BIT_16) {
            resScale = 4912.0 / 32760.0;
        } else {
            resScale = 4912.0 / 8190.0;
        }

        // Apply sensitivity adjustments according to datasheet
        // Hadj = H * (ASA + 128) / 256
        return ((asa + 128.0) / 256.0) * resScale;
    }
};

// Device info
#define MPU6050_ADDRESS_AD0_LOW     0x68
#define MPU6050_ADDRESS_AD0_HIGH    0x69
#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_LOW

// Device IDs for WHO_AM_I register
#define MPU6050_DEVICE_ID           0x34
#define MPU6500_DEVICE_ID           0x70
#define MPU9150_DEVICE_ID           0x68
#define MPU9250_DEVICE_ID           0x71

// Register map
enum {
    MPU6050_RA_GYRO_CONFIG         = 0x1B,
    MPU6050_RA_ACCEL_CONFIG        = 0x1C,
    MPU6050_RA_INT_PIN_CFG         = 0x37,
    MPU6050_RA_ACCEL_XOUT_H        = 0x3B,
    MPU6050_RA_TEMP_OUT_H          = 0x41,
    MPU6050_RA_GYRO_XOUT_H         = 0x43,
    MPU6050_RA_USER_CTRL           = 0x6A,
    MPU6050_RA_PWR_MGMT_1          = 0x6B,
    MPU6050_RA_WHO_AM_I            = 0x75
};

// GYRO_CONFIG
#define MPU6050_GYRO_FS_SEL_BIT     4
#define MPU6050_GYRO_FS_SEL_LEN     2

// GYRO_CONFIG FS_SEL
#define MPU6050_GYRO_FS_250         0x00
#define MPU6050_GYRO_FS_500         0x01
#define MPU6050_GYRO_FS_1000        0x02
#define MPU6050_GYRO_FS_2000        0x03

// ACCEL_CONFIG
#define MPU6050_ACCEL_FS_SEL_BIT    4
#define MPU6050_ACCEL_FS_SEL_LEN    2

// ACCEL_CONFIG AFS_SEL
#define MPU6050_ACCEL_FS_2          0x00
#define MPU6050_ACCEL_FS_4          0x01
#define MPU6050_ACCEL_FS_8          0x02
#define MPU6050_ACCEL_FS_16         0x03

// INT_PIN_CFG
#define MPU6050_BYPASS_EN_BIT       1

// USER_CTRL
#define MPU6050_DMP_EN_BIT          7
#define MPU6050_FIFO_EN_BIT         6

// PWR_MGMT_1
#define MPU6050_DEVICE_RESET_BIT    7
#define MPU6050_SLEEP_BIT           6
#define MPU6050_CLKSEL_BIT          2
#define MPU6050_CLKSEL_LEN          3
#define MPU6050_CLOCK_INTERNAL      0x00
#define MPU6050_CLOCK_PLL           0x01
#define MPU6050_CLOCK_KEEP_RESET    0x07

class MPU6050 : public I2CDevice, public Accelerometer, public Gyroscope {
public:
    static MPU6050& getInstance() {
        static MPU6050 instance;
        return instance;
    }

    // Initialization
    MPU6050(uint8_t address = MPU6050_DEFAULT_ADDRESS) : I2CDevice(address) {

    }

    void initialize() {
        // Wake up the device
        setSleepEnabled(false);

        // Use the most accurate clock source
        setClockSource(MPU6050_CLOCK_PLL);

        // Set the sensitivity to max on gyro and accel
        setFullScaleGyroRange(MPU6050_GYRO_FS_250);
        setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

        // Allow direct I2C access to devices connected to the MPU6050 aux bus
        setI2cBypassEnabled(true);

        // Calculate the scale factors from the configured ranges
        accelScale = getAccelScale(getFullScaleAccelRange());
        gyroScale = getGyroScale(getFullScaleGyroRange());
    }

    bool testConnection() {
        switch(getDeviceID()) {
            case MPU6050_DEVICE_ID:
                return true;
            case MPU6500_DEVICE_ID:
                return true;
            case MPU9150_DEVICE_ID:
                return true;
            case MPU9250_DEVICE_ID:
                return true;
        }

        return false;
    }

    // Accelerometer
    Vector3 getAcceleration() {
        Vector3 acceleration;

        // Convert raw data into signed 16-bit data
        int16_t rawAccel[3];
        readWords(MPU6050_RA_ACCEL_XOUT_H, 3, (uint16_t *)rawAccel);

        // Apply accelerometer scale to get Gs, convert to m/s^2
        acceleration.x = (float)rawAccel[0]/accelScale * STANDARD_GRAVITY;
        acceleration.y = (float)rawAccel[1]/accelScale * STANDARD_GRAVITY;
        acceleration.z = (float)rawAccel[2]/accelScale * STANDARD_GRAVITY;

        return acceleration;
    }

    // Gyroscope
    Vector3 getRotation() {
        Vector3 rotation;

        // Convert raw data into signed 16-bit data
        int16_t rawRotation[3];
        readWords(MPU6050_RA_GYRO_XOUT_H, 3, (uint16_t *)rawRotation);

        // Apply gyroscope scale to get deg/s, convert to rad/s
        rotation.x = (float)rawRotation[0]/gyroScale * M_PI/180.0;
        rotation.y = (float)rawRotation[1]/gyroScale * M_PI/180.0;
        rotation.z = (float)rawRotation[2]/gyroScale * M_PI/180.0;

        return rotation;
    }

    // GYRO_CONFIG register
    uint8_t getFullScaleGyroRange() {
        readBits(MPU6050_RA_GYRO_CONFIG, MPU6050_GYRO_FS_SEL_BIT, MPU6050_GYRO_FS_SEL_LEN, buffer);

        return buffer[0];
    }

    void setFullScaleGyroRange(uint8_t range) {
        writeBits(MPU6050_RA_GYRO_CONFIG, MPU6050_GYRO_FS_SEL_BIT, MPU6050_GYRO_FS_SEL_LEN, range);
        gyroScale = getGyroScale(range);
    }

    // ACCEL_CONFIG register
    uint8_t getFullScaleAccelRange() {
        readBits(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACCEL_FS_SEL_BIT, MPU6050_ACCEL_FS_SEL_LEN, buffer);

        return buffer[0];
    }

    void setFullScaleAccelRange(uint8_t range) {
        writeBits(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACCEL_FS_SEL_BIT, MPU6050_ACCEL_FS_SEL_LEN, range);
        accelScale = getAccelScale(range);
    }

    // INT_PIN_CFG register
    bool getI2cBypassEnabled() {
        readBit(MPU6050_RA_INT_PIN_CFG, MPU6050_BYPASS_EN_BIT, buffer);
        return buffer[0];
    }

    void setI2cBypassEnabled(bool enabled) {
        writeBit(MPU6050_RA_INT_PIN_CFG, MPU6050_BYPASS_EN_BIT, enabled);
    }

    // USER_CTRL register
    bool getDMPEnabled() {
        readBit(MPU6050_RA_USER_CTRL, MPU6050_DMP_EN_BIT, buffer);
        return buffer[0];
    }

    void setDMPEnabled(bool enabled) {
        writeBit(MPU6050_RA_USER_CTRL, MPU6050_DMP_EN_BIT, enabled);
    }

    // PWR_MGMT_1 register
    void reset() {
        writeBit(MPU6050_RA_PWR_MGMT_1, MPU6050_DEVICE_RESET_BIT, 1);
    }
    bool getSleepEnabled() {
        readBit(MPU6050_RA_PWR_MGMT_1, MPU6050_SLEEP_BIT, buffer);
        return buffer[0];
    }

    void setSleepEnabled(bool enabled) {
        writeBit(MPU6050_RA_PWR_MGMT_1, MPU6050_SLEEP_BIT, enabled);
    }

    uint8_t getClockSource() {
        readBits(MPU6050_RA_PWR_MGMT_1, MPU6050_CLKSEL_BIT, MPU6050_CLKSEL_LEN, buffer);
        return buffer[0];
    }

    void setClockSource(uint8_t source) {
        writeBits(MPU6050_RA_PWR_MGMT_1, MPU6050_CLKSEL_BIT, MPU6050_CLKSEL_LEN, source);
    }

    // WHO_AM_I registe
    uint8_t getDeviceID() {
        readByte(MPU6050_RA_WHO_AM_I, buffer);
        return buffer[0];
    }

protected:
    float accelScale;
    float gyroScale;

    float getGyroScale(uint8_t gyroRange) {
        return 16.4 * pow(2, 3 - gyroRange);
    }

    float getAccelScale(uint8_t accelRange) {
        return 2048.0 * pow(2, 3 - accelRange);
    }
};

// MPU9150 Accelerometer, Gyroscope and Magnetometer
// The MPU9150 combines an MPU6050 and AK8975 in one chip
#ifdef SENSORS_MPU9150_ATTACHED
    #define SENSORS_MPU6050_ATTACHED
    #define SENSORS_AK8975_ATTACHED
#endif

// MPU9250 Accelerometer, Gyroscope and Magnetometer
// The MPU9250 combines an MPU6500 and AK8963 in one chip
#ifdef SENSORS_MPU9250_ATTACHED
    #define SENSORS_MPU6500_ATTACHED
    #define SENSORS_AK8963_ATTACHED
#endif

// MPU6500 Accelerometer and Gyroscope
// The MPU6500 is both supported by the MPU6050 driver
#ifdef SENSORS_MPU6500_ATTACHED
    #define SENSORS_MPU6050_ATTACHED
#endif

// BMP180 Barometer and Thermometer
// The BMP180 is supported by the BMP085 driver
#ifdef SENSORS_BMP180_ATTACHED
    #define SENSORS_BMP085_ATTACHED
#endif

class Sensors {
public:
    // Initialize attached sensors
    // Only call this after enabling and waking up the I2C bus
    static void initialize() {
        #ifdef SENSORS_AK8963_ATTACHED
        AK8963::getInstance().initialize();
        #endif

        #ifdef SENSORS_BMP085_ATTACHED
        BMP085::getInstance().initialize();
        #endif

        #ifdef SENSORS_HMC5883L_ATTACHED
        HMC5883L::getInstance().initialize();
        #endif

        #ifdef SENSORS_MPU6050_ATTACHED
        MPU6050::getInstance().initialize();
        #endif
    }

    static Accelerometer *getAccelerometer() {
        #ifdef SENSORS_MPU6050_ATTACHED
        return &MPU6050::getInstance();
        #else
        return NULL;
        #endif
    }

    static Barometer *getBarometer() {
        #ifdef SENSORS_BMP085_ATTACHED
        return &BMP085::getInstance();
        #else
        return NULL;
        #endif
    }

    static Gyroscope *getGyroscope() {
        #ifdef SENSORS_MPU6050_ATTACHED
        return &MPU6050::getInstance();
        #else
        return NULL;
        #endif
    }

    static Magnetometer *getMagnetometer() {
        #if defined(SENSORS_AK8963_ATTACHED)
        return &AK8963::getInstance();
        #elif defined(SENSORS_HMC5883L_ATTACHED)
        return &HMC5883L::getInstance();
        #else
        return NULL;
        #endif
    }

    static Thermometer *getThermometer() {
        #ifdef SENSORS_BMP085_ATTACHED
        return &BMP085::getInstance();
        #else
        return NULL;
        #endif
    }
};


int main() {
    // Initialize WiringPi pins
    wiringPiSetup();

    // Initialize devices
    Sensors::initialize();
/*
    while(true) {
        Accelerometer *accelerometer = Sensors::getAccelerometer();
        if(accelerometer) {
            Vector3 a = accelerometer->getAcceleration();
            printf("Acceleration (m/s^2)  %+7.3f, %+7.3f, %+7.3f\n", a.x, a.y, a.z);
        }

        Barometer *barometer = Sensors::getBarometer();
        if(barometer) {
            float p = barometer->getPressure();
            printf("Pressure (hPa)        %+7.3f\n", p);

            float a = barometer->getAltitude();
            printf("Altitude (m)          %+7.3f\n", a);
        }

        Gyroscope *gyroscope = Sensors::getGyroscope();
        if(gyroscope) {
            Vector3 g = gyroscope->getRotation();
            printf("Rotation (rad/s)      %+7.3f, %+7.3f, %+7.3f\n", g.x, g.y, g.z);
        }
*/
        Magnetometer *magnetometer = Sensors::getMagnetometer();
        if(magnetometer) {
            //Vector3 m = magnetometer->getMagneticField();
            //printf("Magnetic Field (uT)   %+7.3f, %+7.3f, %+7.3f\n", m.x, m.y, m.z);

            float azimuth = magnetometer->getAzimuth();
            //printf("Azimuth (deg)         %+7.3f\n", azimuth);
            printf("%f", azimuth);
            //return azimuth;
        }
/*
        Thermometer *thermometer = Sensors::getThermometer();
        if(thermometer) {
            float t = thermometer->getTemperature();
            printf("Temperature (C)       %+7.3f\n", t);
        }

        delay(50);
    }
*/
}
