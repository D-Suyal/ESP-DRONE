# Custom ESP32-S3 PCB (BMI270 + BMP581): flash and bring-up

## Firmware vs tuning tools

- **Flash and serial log:** ESP-IDF only — `idf.py set-target esp32s3`, `idf.py build`, `idf.py -p PORT flash monitor`.
- **Parameters / PIDs / logs after boot:** [cfclient](https://github.com/bitcraze/crazyflie-clients-python) (or Espressif-documented fork) over **Wi-Fi** (CRTP). It does not replace USB flashing.

## USB–UART bridge (e.g. NodeMCU)

Use a common GND, cross TX/RX to the UART you configured for the console, and follow your board’s **BOOT / IO0 / EN** sequence for download mode. With `sdkconfig.defaults.esp32s3`, the default console is **USB Serial/JTAG** so GPIO43 can be used for the red LED without fighting UART0 TX.

## Kconfig

- **Sensor stack:** this fork always uses the BMI270 + BMP581 HAL on ESP32-S3 (`bmi270` + `bmp581` components); there is no MPU6050/MS5611 path.
- I2C addresses: **BMI270** / **BMP581** hex options (`CONFIG_ESPDRONE_BMI270_I2C_ADDR`, `CONFIG_ESPDRONE_BMP581_I2C_ADDR`; defaults `0x68` / `0x46`).
- Pin numbers: set in **menuconfig** or via `sdkconfig` / `sdkconfig.defaults.esp32s3` (see defaults for the captured custom table).

## Bench / flight checklist (manual)

1. Build and flash; confirm BMI270 and BMP581 init messages on the console.
2. Static IMU: gravity vector and gyro near zero after bias; I2C scan if IDs fail.
3. Baro: pressure and ASL react sensibly when moving the board vertically.
4. **Motors:** props off; verify directions and mapping (J1–J4 vs FR/BR/BL/FL may need swaps).
5. Wi-Fi up; connect cfclient; log and tune stabilize, then altitude hold.
6. Tethered flight; PID retune for Z after baro change from MS5611.
