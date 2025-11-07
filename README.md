# BIOmetrics and DYnamics Node 100 Firmware

This repo holds firmware for the BIODYN-100.

## Built With
- C
- ESP-IDF
- FreeRTOS
- Bluedroid

## Building & Flashing
1. Source ESP-IDF with `. $HOME/esp/esp-idf/export.sh` or manually. Ensure ESP-IDF is installed on your device properly.
2. Ensure the target it set with `idf.py set-target ESP32-S3`.
3. Set config for flash in `idf.py menuconfig` to 16Mib
4. Find the device name (usb port) in `/dev` if on MacOS or Linux. Windows will be a COM port.
5. Run `idf.py -p PORT flash monitor` to build, flash, and monitor the project. To exit the serial monitor, type ``Ctrl-]``.

## Configuration

Custom config options can be placed in `main/Kconfig.projbuild`.
They can be configured with `idf.py menuconfig`.

## Resources

- [ESP-IDF setup on MacOS and Linux](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/linux-macos-setup.html)
- [ESP Build System Overview](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/build-system.html)
