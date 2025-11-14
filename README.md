<img width="1920" height="1080" alt="image" src="https://github.com/user-attachments/assets/9d2fa50f-a39e-44c0-baf1-7dba50a79bba" /># 8-DOF Quadruped Robot 

![img](https://errouji.vercel.app/content/Robotics/Quadruped_Robot/assembled_robot.png)

## Overview
This project provides the firmware and algorithms driving an 8-degree-of-freedom (DOF) quadruped robot. It delivers:
- Modular code for joint control, inverse kinematics, and trajectory generation
- Coordinated gait cycles for realistic robot walking
- Serial command interface for remote control and parameter tuning

The code was written and adapted by Oussama Errouji for the NRC 2025 competition and is designed for extensibility and clarity, with documentation and professional comments throughout.

---

## Directory & File Structure

- **`ROBOTX-8DOF.ino`**  
  Main entry point and system orchestrator. Handles initialization, state, serial communication, walking cycles, and command interpretation.

- **`includes.h`**  
  Shared definitions, constants, structures, and servo mapping. Required by all modules for correct configuration and robot geometry.

- **`Leg.ino`**  
  Leg control logic, including creation of leg objects, kinematic computations, and servo angle setting. Provides low-level functions for moving individual legs and calculating inverse kinematics.

- **`Trajectory.ino`**  
  Core algorithms for generating and handling leg trajectories—both linear (stance/ground phase) and elliptical (flight/swing phase). Ensures smooth and coordinated gait motion.

- **`Tests.ino`**  
  Contains utility functions for testing and validating kinematic and trajectory elements, supporting calibration and debugging.

---

## Hardware Setup
- 8-DOF quadruped robot (four legs, each with two servos: hip & knee)
- **Adafruit PWM Servo Driver** board for servo actuation
- Microcontroller compatible with Arduino (tested on Arduino Mega)
- 5V power supply recommended for stable servo operation
- Serial communication via USB (default baud: 9600)

### Wiring Guide
- Connect each servo as per the IDs in `includes.h`.
- Hip and knee servos on each leg must match the robot's documented arrangement for correct movement.

---

## Installation & Upload
1. Edit `#define` values in `includes.h` if your geometry differs.
2. Open `ROBOTX-8DOF.ino` in Arduino IDE. Ensure all `.ino` and `.h` files are in the same folder.
3. Install the `Adafruit_PWMServoDriver` library via Library Manager if not already.
4. Select the correct board/port in the Arduino IDE and upload the sketch.

---

## Serial Command Reference
You can control the robot via USB serial using the following commands:

- `MVx` — **Move:**
    - `MV1`: Start walking (play)
    - `MV0`: Stop walking
    - `MV2`: Step forward
    - `MV3`: Stand pose
- `SPxxxx` — **Set Tick Delay:**
    - Example: `SP100` sets delay between steps to 100 ms
- `RXnxxx` / `RYnxxx` — **Set Ellipse Radius:**
    - `RXn`: Set X-radius for leg `n`'s elliptical walking trajectory
    - `RYn`: Set Y-radius for leg `n`'s elliptical walking trajectory
    - Example: `RX260` sets leg 2's ellipse X-radius to 60
- `OXnxxx` / `OYnxxx` — **Set Ellipse Origin:**
    - `OX2-10` sets flight ellipse X origin for leg 2 to -10
    - `OY3120` sets flight ellipse Y origin for leg 3 to 120

**Note:** `n` is a leg index (1–4):
- 1: Back Left
- 2: Front Left
- 3: Front Right
- 4: Back Right

---

## How it Works
- On startup, the robot stands up in default pose.
- Walking is handled by pre-configured gait cycle arrays and dynamic trajectory planning:
    - **Ground phase** uses linear segments for foot placement
    - **Flight phase** uses elliptic arcs for smooth foot lift/placement
- Kinematic calculations determine joint angles from trajectory points
- Serial commands allow you to start/stop walking, adjust speeds and stance, and tune as needed

---

## Advanced Features
- **Kinematics:** Built-in inverse kinematics for error-free motion
- **Trajectory planning:** Easily extend gait cycles or add new motion
- **Testing:** Use `Tests.ino` functions for calibration and debugging
- **Modularity:** Easy to adapt for new gaits, robot shapes, or servo arrangements

---

## Example Usage
1. **Start robot walking:** Send `MV1` over Serial
2. **Change speed:** Send `SP70` (for faster) or `SP150` (slower)
3. **Tune left front leg's flight arc:** Send `RX215` (X-radius) or `OY2100` (Y-origin)
4. **Stop walking:** Send `MV0`

---

## Support
For issues, feature requests, or contributions, please contact Oussama Errouji or submit a pull request!
