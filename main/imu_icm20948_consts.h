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
#define ODR_ALIGN_EN 0x09
#define PWR_MGMT_1 0x06
#define PWR_MGMT_2 0x07
#define USER_CTRL 0x03

#define ACCEL_SMPLRT_DIV_1 0x10
#define ACCEL_SMPLRT_DIV_2 0x11
#define ACCEL_CONFIG 0x14

#define GYRO_SMPLRT_DIV 0x00
#define GYRO_CONFIG_1 0x01
#define GYRO_CONFIG_2 0x02

// Register bank 1
#define SELF_TEST_X_GYRO 0x2
#define SELF_TEST_Y_GYRO 0x3
#define SELF_TEST_Z_GYRO 0x4

#define READ_MSB 0x80
#define WRITE_MSB 0x00

#define ACCEL_RANGE_VALUE _accel_2g
#define GYRO_RANGE_VALUE _gyro_1000dps

// Register bank 3
#define I2C_MST_ODR_CONFIG 0x00
#define I2C_MST_CTRL 0x01
#define I2C_MST_DELAY_CTRL 0x02
#define I2C_SLV0_ADDR 0X03
#define I2C_SLV0_REG 0X04
#define I2C_SLV0_CTRL 0X05
#define I2C_SLV0_DO 0X06

#define AK09916_ADDRESS 0x0c
#define TAG "IMU_ICM20948"
#define EARTH_GRAVITY 9.80665f

#define MAXIMUM_BUS_SPEED 400 // khz

#endif // IMU_ICM20948_CONSTANTS