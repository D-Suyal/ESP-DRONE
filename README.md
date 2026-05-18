# ESP-Drone (this fork)

Personal firmware fork based on [Espressif ESP-Drone](https://github.com/espressif/esp-drone). Upstream remains the reference for the original hardware scope and documentation.

* [中文](./README_cn.md) (upstream; may not reflect all fork changes)

**Upstream:** [github.com/espressif/esp-drone](https://github.com/espressif/esp-drone) — track as remote `upstream` for merges.

**This repository:** [github.com/D-Suyal/ESP-DRONE](https://github.com/D-Suyal/ESP-DRONE)

---

## Sensor changes (key difference from upstream)

| Sensor role | Upstream | **This fork** |
| :--- | :--- | :--- |
| **IMU** (accelerometer) | MPU6050 | **Bosch BMI270** (I²C) |
| **Gyro** (gyroscope) | MPU6050 | **Bosch BMI270** (I²C) |
| **Barometer** | MS5611 | **Bosch BMP581** (I²C) |
| Compass | HMC5883L | removed |
| Optical flow / ToF | VL53 / PMW3901 | removed |

The MPU6050-based driver stack is gone. All flight-attitude and height-hold work runs through the BMI270 (6-axis IMU) and BMP581 (pressure/altitude).

## What is different here

| Area | This fork |
| :--- | :--- |
| **MCU / focus** | **ESP32-S3** oriented (`sdkconfig.defaults.esp32s3`, CI). Legacy ESP32 / ESP32-S2 paths are not the maintenance focus. |
| **IMU / baro** | **Bosch BMI270** (accel + gyro) + **BMP581** (baro) over I²C. Old MPU6050 / MS5611 / HMC5883L and related VL53 / PMW3901 stacks are removed. |
| **Wi-Fi / CRTP debug** | Optional **serial monitor** helpers under `menuconfig` → *ESPDrone Config* → *wireless config* → *Serial monitor debug*: UDP hex dump and throttled **RPYT** lines (`[WIFI_CTRL]`, `[WIFI_UDP]`). |
| **Docs** | [docs/CUSTOM_S3_BMI270_PORT.md](./docs/CUSTOM_S3_BMI270_PORT.md) for this port; [THIRD_PARTY.md](./THIRD_PARTY.md) for Bosch licensing notes. |

Build with **ESP-IDF [release/v5.0](https://docs.espressif.com/projects/esp-idf/en/release-v5.0/esp32s3/get-started/index.html)** (or the version you use locally), then:

```bash
idf.py set-target esp32s3
idf.py build flash monitor
```

---

## Introduction (from upstream)

**ESP-Drone** is an open source solution based on Espressif **ESP32 / ESP32-S2 / ESP32-S3**, controllable from a **mobile app** or **gamepad** over **Wi‑Fi**. The main flight stack derives from the **Crazyflie** firmware (**GPL-3.0**).

![ESP-Drone](./docs/_static/espdrone_s2_v1_2_2.png)

### Useful links (Espressif)

* **Getting started:** [Espressif ESP-Drone docs](https://docs.espressif.com/projects/espressif-esp-drone/en/latest/gettingstarted.html)
* **Hardware (S2 reference):** [mainboard schematic PDF](https://docs.espressif.com/projects/espressif-esp-drone/en/latest/_static/ESP32_S2_Drone_V1_2/SCH_Mainboard_ESP32_S2_Drone_V1_2.pdf)
* **iOS app:** [ESP-Drone-iOS](https://github.com/EspressifApps/ESP-Drone-iOS)
* **Android app:** [ESP-Drone-Android](https://github.com/EspressifApps/ESP-Drone-Android)

### PC client (cfclient)

Use a **CRTP-compatible** client over Wi‑Fi (UDP). Examples: [qljz1993/crazyflie-clients-python](https://github.com/qljz1993/crazyflie-clients-python) or [leeebo/crazyflie-clients-python](https://github.com/leeebo/crazyflie-clients-python) as referenced upstream; match **cflib** branch (`esplane`) where the project docs require it.

---

## Features (upstream + notes)

1. Stabilize mode  
2. Height-hold (needs baro — **BMP581** on this fork)  
3. Position-hold (needs flow / ToF; **not** carried in this minimal driver set unless you add hardware + drivers)  
4. App control over Wi‑Fi  
5. **cfclient** / CRTP over Wi‑Fi  
6. **ESP-BOX3**-style joystick via **ESP-NOW** (where enabled in firmware)

Height-hold / position-hold still depend on sensors and tuning; see docs and your board.

---

## Third-party code

| Component | License | Origin |
| :---: | :---: | :--- |
| `core/crazyflie` | GPL-3.0 | [Crazyflie firmware](https://github.com/bitcraze/crazyflie-firmware) (upstream snapshot) |
| `lib/dsp_lib` | (see tree) | [esp32-lin / dsp_lib](https://github.com/whyengineer/esp32-lin/tree/master/components/dsp_lib) |
| Bosch BMI270 / BMP581 driver sources | See `LICENSE` in each `bosch/` folder | Bosch Sensortec examples (see **THIRD_PARTY.md**) |

---

## Thanks

1. [Bitcraze / Crazyflie](https://www.bitcraze.io/)  
2. [Espressif / ESP-IDF](https://docs.espressif.com/projects/esp-idf/)  
3. [WhyEngineer / ESP-DSP](https://github.com/whyengineer/esp32-lin/tree/master/components/dsp_lib)
