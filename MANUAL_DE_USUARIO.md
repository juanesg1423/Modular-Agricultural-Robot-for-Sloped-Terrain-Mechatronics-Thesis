# User Manual

## Modular Agricultural Robot for Sloped Terrain

This manual describes the basic operation of the prototype controlled by an ESP32, a PS5 DualSense controller, CAN-connected Hub BLDC motors and SteadyWin chassis actuators. The BLE HMI runs in a browser and connects directly to the same ESP32.

## 1. Safety before operation

- Operate the robot in a clear area with people and animals outside its path.
- Before the first powered test, raise the drive wheels or mechanically secure the chassis so it cannot roll or tip.
- Keep hands, clothing, and tools away from wheels, joints, arms, and actuators.
- Verify the battery polarity, wiring, connectors, CAN transceiver, and motor-controller configuration before switching on.
- Keep the physical battery disconnect accessible. The DualSense Circle button and controller-disconnection stop are software functions; they do not replace a physical disconnect.
- Use the robot only within the documented operating limits. The prototype payload verified in testing was 15 kg; the intended maximum operating slope is 15 degrees.

## 2. Before powering on

1. Inspect the frame, wheels, actuator mounts, fasteners, and payload attachment.
2. Confirm that the 12 V batteries are connected in series to provide the system's 24 V supply, and verify that all controller and converter ratings match the supply.
3. Check the CAN wiring and transceiver. Do not connect CANH or CANL directly to ESP32 GPIO pins.
4. Ensure that the DualSense is charged and paired using the pairing procedure configured for the ESP32 firmware.
5. Keep the robot disarmed while connecting power and checking the system.

## 3. Starting the robot and HMI

1. Turn on the robot's power supply.
2. Wait for the ESP32 to initialize its sensors and CAN interface. If the firmware reports a fatal CAN initialization error, do not try to move the robot; switch it off and inspect the CAN setup.
3. Open the HMI HTML page in a browser that supports Web Bluetooth in a secure context.
4. Select **Connect ESP32**, then choose the BLE device named `Robot_Agricola_HMI`.
5. Turn on/connect the DualSense. The HMI should report the controller connection after it receives the corresponding firmware message.

The HMI shows the selected mode, armed state, last event, Hub CANopen configuration results, SteadyWin messages received by the firmware, and a text console mirroring Serial. It starts receiving messages only after connection, so it cannot show earlier startup messages. If a field says “Waiting for report,” that information has not been received by the HMI.

## 4. DualSense controls

| Control | Function |
|---|---|
| Triangle | Select height mode |
| Square | Select drive mode |
| Cross (X) | Arm or disarm the selected mode |
| Hold R2 | Enable manual movement while held |
| Left stick up/down in height mode | Raise/lower the chassis using the SteadyWin actuators |
| Left stick up/down in drive mode | Move forward/reverse |
| Left stick left/right in drive mode | Turn |
| D-pad up/down in drive mode, with R2 released | Increase/decrease the Hub speed limit, up to the firmware maximum |
| D-pad left in height mode, disarmed | Capture a travel endpoint during height calibration |
| Circle | Trigger software emergency stop |

Release R2 before changing modes, changing the Hub speed limit, or capturing a calibration endpoint. The firmware requires the system to be disarmed for calibration captures.

## 5. Height-travel calibration

The firmware captures lower and upper travel endpoints for the four SteadyWin actuators. Perform calibration with the chassis stable and unloaded unless the project's test procedure specifies otherwise.

1. Select height mode with Triangle and leave the system disarmed.
2. Move the actuators to the chosen lower endpoint, then press D-pad left once to record it.
3. Arm the system with Cross. Hold R2 and move the left stick upward to raise the chassis slowly.
4. Stop at the chosen upper endpoint. Release R2, then disarm with Cross.
5. Press D-pad left again to record the upper endpoint.
6. Confirm that the HMI/Serial output reports a completed travel calibration before operating within that range.

The calibration values are held in firmware memory and may need to be captured again after an ESP32 restart. Never calibrate at a mechanical endpoint that could bind or overload an actuator.

## 6. Obstacle stop and emergency stop

In drive mode, the ultrasonic sensors monitor for obstacles. When the firmware detects an obstacle within its configured stop distance, it commands a stop and disarms the system. The Circle button also requests a software emergency stop. Loss of the DualSense connection triggers a stop/disarm response in firmware.

After an emergency stop or controller disconnection, inspect the robot and the cause of the event. Reconnect the controller, select the required mode again, and arm only when the area is clear. Do not repeatedly re-arm while an obstacle or communication fault remains.

The ultrasonic stop depends on sensor readings and coverage. It does not detect every object or replace operator supervision.

## 7. Shutting down

1. Release R2 and stop movement.
2. Disarm with Cross.
3. Turn off the robot's power using the main power switch or physical disconnect.
4. Disconnect the HMI from the BLE device and power down the DualSense as needed.
5. Inspect for loose fasteners, damaged wires, overheated components, or actuator binding before the next use.

## 8. Troubleshooting

| Symptom | Check |
|---|---|
| HMI cannot find `Robot_Agricola_HMI` | Confirm the ESP32 completed setup, BLE is enabled in the firmware, browser Bluetooth permission is available, and the page is opened in a secure context. |
| HMI connects but status fields remain unknown | The HMI only parses messages received after connection. Reconnect before startup or cause the relevant status event safely, then check the raw Serial console. |
| DualSense does not connect | Check controller charge and pairing, the firmware's controller configuration, and whether another device is currently connected to the controller. |
| Hub node reports `ERROR` or CANopen timeout | Stop testing. Check CANH/CANL polarity, transceiver power, common reference, bus termination, bitrate, node IDs, and motor-controller supply. |
| SteadyWin reports a transmit error or no position response | Do not move the chassis. Check CAN wiring, controller IDs 11–14, driver power, and whether the CAN interface is running. |
| Robot stops after controller disconnection or obstacle detection | This is the expected software stop response. Inspect the cause and re-arm only after it is resolved. |
| Height calibration is missing after restart | Repeat the travel calibration; captured limits may not persist across power cycles. |

## 9. Interpreting HMI statuses

The HMI presents the last status messages received from the firmware. A Hub `OK` means that the node responded to configuration; it does not prove the motor is rotating. A SteadyWin status or fault is shown only when the firmware receives and prints the corresponding CAN report. Treat “no report” as unknown, not as confirmed normal operation.
