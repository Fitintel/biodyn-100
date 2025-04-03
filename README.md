# BIODYN-100 Firmware

This repo holds firmware for the BIODYN-100.

## Building & Flashing
1. Source ESP-IDF with `. $HOME/esp/esp-idf/export.sh` or manually. Ensure ESP-IDF is installed on your device properly.
2. Ensure the target it set with `idf.py set-target ESP32-S3`.
3. Find the device name (usb port) in `/dev` if on MacOS or Linux. Windows will be a COM port.
4. Run `idf.py -p PORT flash monitor` to build, flash, and monitor the project. To exit the serial monitor, type ``Ctrl-]``.

## Resources

- [ESP-IDF setup on MacOS and Linux](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/linux-macos-setup.html)
- [ESP Build System Overview](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/build-system.html)