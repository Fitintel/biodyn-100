idf_component_register(
	SRCS "temp/temperature.c" "accel/imu_icm20948_driver.c" "main.c" "bluetooth/ble.c" "system/led.c" "system/self_test.c"
	INCLUDE_DIRS "."
)
