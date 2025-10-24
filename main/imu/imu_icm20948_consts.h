#ifndef IMU_ICM20948_CONSTANTS
#define IMU_ICM20948_CONSTANTS

#define IMU_WHOAMI 0x0
#define LP_CONFIG 0x05
#define ACCEL_XOUT_H 0x2d
#define ACCEL_XOUT_L 0x2e
#define ACCEL_YOUT_H 0x2f
#define ACCEL_YOUT_L 0x30
#define ACCEL_ZOUT_H 0x31
#define ACCEL_ZOUT_L 0x32

#define GYRO_XOUT_H 0x33
#define GYRO_XOUT_L 0x34
#define GYRO_YOUT_H 0x35
#define GYRO_YOUT_L 0x36
#define GYRO_ZOUT_H 0x37
#define GYRO_ZOUT_L 0x38

#define WHO_AM_I 0x00
#define REG_BANK_SEL 0x7f
#define PWR_MGMT_1 0x06
#define PWR_MGMT_2 0x07
#define USER_CTRL 0x03

#define TEMP_OUT_H 0x39
#define TEMP_OUT_L 0x3a

// The equivalent of (x...z)out(h...l) for magnetometer
#define EXT_SLV_SENS_DATA_00 0x3b
#define EXT_SLV_SENS_DATA_01 0x3c
#define EXT_SLV_SENS_DATA_02 0x3d
#define EXT_SLV_SENS_DATA_03 0x3e
#define EXT_SLV_SENS_DATA_04 0x3f
#define EXT_SLV_SENS_DATA_05 0x40

#define MAG_XOUT_H EXT_SLV_SENS_DATA_00
#define MAG_XOUT_L EXT_SLV_SENS_DATA_01
#define MAG_YOUT_H EXT_SLV_SENS_DATA_02
#define MAG_YOUT_L EXT_SLV_SENS_DATA_03
#define MAG_ZOUT_H EXT_SLV_SENS_DATA_04
#define MAG_ZOUT_L EXT_SLV_SENS_DATA_05

// Register bank 1
#define SELF_TEST_X_GYRO 0x02
#define SELF_TEST_Y_GYRO 0x03
#define SELF_TEST_Z_GYRO 0x04
#define SELF_TEST_X_ACCEL 0x0E
#define SELF_TEST_Y_ACCEL 0x0F
#define SELF_TEST_Z_ACCEL 0x10

#define READ_MSB 0x80
#define WRITE_MSB 0x00

// Register bank 2
#define GYRO_SMPLRT_DIV 0x00
#define GYRO_CONFIG_1 0x01
#define GYRO_CONFIG_2 0x02
#define ODR_ALIGN_EN 0x09
#define ACCEL_SMPLRT_DIV_1 0x10
#define ACCEL_SMPLRT_DIV_2 0x11
#define ACCEL_CONFIG 0x14
#define ACCEL_CONFIG_2 0x15

// Register bank 3
#define I2C_MST_ODR_CONFIG 0x00
#define I2C_MST_CTRL 0x01
#define I2C_MST_DELAY_CTRL 0x02
#define I2C_SLV0_ADDR 0X03
#define I2C_SLV0_REG 0X04
#define I2C_SLV0_CTRL 0X05
#define I2C_SLV0_DO 0X06

#define AK09916_ADDRESS 0x0c
#define AK09916_WIA 0x01
#define AK09916_STATUS1 0x10
#define AK09916_HXL 0x11
#define AK09916_HXH 0x12
#define AK09916_HYL 0x13
#define AK09916_HYH 0x14
#define AK09916_HZL 0x15
#define AK09916_HZH 0x16
#define AK09916_STATUS2 0x18
#define AK09916_CONTROL2 0x31
#define AK09916_CONTROL3 0x32
// #define AK09916_TEST1 0X33
// #define AK09916_TEST2 0x34
// Do not use these two registers above

#define TAG "IMU_ICM20948"
#define EARTH_GRAVITY 9.80665f

#define MAXIMUM_BUS_SPEED 400 // khz

#endif // IMU_ICM20948_CONSTANTS