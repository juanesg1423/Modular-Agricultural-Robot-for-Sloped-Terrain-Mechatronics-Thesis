# Modular Agricultural Robot for Sloped Terrain

**Undergraduate Mechatronics Thesis Project**

This repository contains the software and operating documentation for a modular agricultural robot developed as an undergraduate Mechatronics thesis project. The platform is designed to assist agricultural work on sloped and uneven terrain.

## Project overview

The prototype combines a four-wheel Hub BLDC drive system with four SteadyWin actuators used to adjust the chassis geometry. An ESP32 runs the robot firmware, receives commands from a PS5 DualSense controller, communicates with the motor controllers over CAN, and reads the onboard sensors. A browser-based human-machine interface (HMI) connects directly to the same ESP32 over Bluetooth Low Energy (BLE) and displays robot status alongside a copy of the Serial output.

## Repository contents

Recommended project organization:

```text
firmware/
  Robot_Agricola_HMI_una_ESP32.ino   # ESP32 robot firmware
hmi/
  hmi_robot_serial.html              # Browser-based BLE HMI
MANUAL_DE_USUARIO.md                 # Operating instructions and safety guidance
```

The filenames and folders can be adjusted to match the files added to this repository.

## Main software functions

- Select height-control or drive mode with the DualSense.
- Control four Hub BLDC drive motors through CANopen commands.
- Control four SteadyWin actuators for chassis height adjustment.
- Read roll and pitch from the IMU.
- Detect nearby obstacles with two ultrasonic sensors and request a stop.
- Stop and disarm the robot when the DualSense disconnects or the emergency-stop button is pressed.
- Mirror firmware Serial messages to the HMI through BLE using the ESP32's built-in radio; no auxiliary ESP32-C3 is required.
- Display the selected mode, controller connection, armed state, Hub configuration results, available SteadyWin reports, and the incoming Serial text in the HMI.

The HMI is read-only. It reports information that the firmware sends; it does not command the motors. A successful CAN configuration response does not, by itself, prove that a motor is moving.

## Hardware and communication reference

| Subsystem | Interface or reference |
|---|---|
| Main controller | ESP32 development board |
| Hub motor controllers | CANopen over ESP32 TWAI; firmware configured for 1 Mbit/s |
| CAN pins | TX: GPIO 17; RX: GPIO 16 |
| DualSense | Bluetooth Classic |
| HMI | BLE device name: `Robot_Agricola_HMI` |
| Ultrasonic sensors | GPIO 25/26 and GPIO 32/33, as configured in firmware |
| IMU | I2C SDA: GPIO 21; SCL: GPIO 22 |

Check the firmware and the actual PCB revision before wiring; pin assignments must match the installed hardware. The ESP32 requires a CAN transceiver between its TWAI pins and the CANH/CANL bus.

## Build and upload

1. Install the ESP32 board support package in Arduino IDE or the compatible build environment.
2. Install the external libraries used by the firmware: `ps5Controller`, Adafruit Unified Sensor, Adafruit LSM303 Accel, and Adafruit L3GD20 Unified. BLE and TWAI headers are provided by the ESP32 Arduino core.
3. Open the `.ino` file, select the correct ESP32 board and serial port, and compile/upload the firmware.
4. Confirm that the CAN transceiver, motor controllers, sensors, power distribution, and common ground match the wiring diagram before powering the robot.
5. Open the HMI in a browser that supports Web Bluetooth in a secure context, select **Connect ESP32**, and choose `Robot_Agricola_HMI`.

The HMI displays messages received after it connects; it does not replay messages produced earlier. Some status fields therefore remain “waiting for report” until the corresponding event occurs.

## User manual

See [MANUAL_DE_USUARIO.md](MANUAL_DE_USUARIO.md) for startup, controller operation, HMI connection, calibration, shutdown, and troubleshooting instructions.

## Safety and validation

The robot contains powered actuators and can move unexpectedly. Perform initial checks with the drive wheels raised or the robot mechanically secured, keep people clear of moving parts, and have access to the physical power disconnect. The emergency-stop button and connection-loss behavior are software functions and do not replace a physical disconnect.

For the documented prototype, the verified payload was 15 kg, and the intended maximum operating slope was 15 degrees. Do not treat a successful test on a steeper slope as a new operating limit without a separate engineering assessment. Height-adjustment behavior under load must only be used within the project's validated conditions.

## Visibility and license

Keep this GitHub repository set to **Private** if the source must not be publicly accessible. Repository visibility is controlled in GitHub settings, not by the license field. No open-source license is included in this repository; do not add one unless the authors and any relevant institution have approved the terms.
