idf_component_register(
    SRCS
    # "temp/temperature.c"
    "imu/imu_icm20948_driver.c"
    "bluetooth/ble.c"
    "system/led.c"
    "system/self_test.c"
    "system/time_sync.c"
	"imu/linear.c"
    "system/data_fast.c"
    "main.c"
    INCLUDE_DIRS "."
)