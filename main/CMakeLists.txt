idf_component_register(
	SRCS "temp/temperature.c" "imu/imu_icm20948_driver.c" "main.c" "bluetooth/ble.c" "system/led.c" "system/self_test.c" "system/time_sync.c"
	INCLUDE_DIRS "."
)
