# ESP32 Robot Tank

This small project is a firmware to control a small robot tank and esposes does controls over Bluetooth Low Energy BLE.

## Install

``` bash
arduino-cli core update-index --additional-urls https://dl.espressif.com/dl/package_esp32_index.json

arduino-cli core search esp32 --additional-urls https://dl.espressif.com/dl/package_esp32_index.json
 > esp32:esp32

arduino-cli core install esp32:esp32 --additional-urls https://dl.espressif.com/dl/package_esp32_index.json

arduino-cli lib install "Robocore - Vespa"
```

## Hardware

- Robot platform was developed by Robocore <https://www.robocore.net/kits/kit-robo-explorer-deluxe>.
- Mainboard was a custom ESP32 Dev Kit board, called Vespa <https://www.robocore.net/placa-robocore/vespa>.

## Wiring

- Left Motor Forward PWM pin = 13
- Left Motor Backward PWM pin = 14
- Right Motor Forward PWM Pin = 4
- Right Motor Backward PWM Pin = 27
- Horizontal Servo PWM Pin = 32
- Vertical Servo PWM Pin = 26
- Distance Servo PWM Pin = 33
- Claw Servo PWM Pin = 25

## Hardware notes

This mainboard has hardware timers, so it can control up to 4 Servo motors at the same time with independent PWM frequence.

Also there are 12 Pins available for PWM or digital instructions. { 25, 26, 32, 33, 5, 16, 17, 18, 19, 21, 22, 23 }

Servo motors can be manufactures with small differences between PWM range of operation due to manufactures changes. Also some Robot Arm restriction should be adjusted to do not damage Servos or Arm structure as well. So these are the servos PWM range of spectrum;

| | |
| :--- | :---: |
| Horizontal Servo | 500 ~ 2500 |
| Vertical Servo | 1500 ~ 3000 |
| Distance Servo | 1350 ~ 2500 |
| Claw Servo | 2000 ~ 3000 |

## Libraries

- BLE Arduino (came with Esp32 default library)
- VespaServo (came with Robocore - Vespa library)
- VespaBattery (came with Robocore - Vespa library)
- <https://github.com/RoboCore/RoboCore_Vespa>

## Board

![](https://d229kd5ey79jzj.cloudfront.net/1439/vespa_esquematica-01.jpg)
