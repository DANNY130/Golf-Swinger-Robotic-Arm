# InverseKinematics Arduino 6DOF Robot Arm Controller

This project provides an Arduino/PlatformIO implementation for controlling a 6-DOF robotic arm using [Denavit–Hartenberg (DH) parameters](https://github.com/thomashiemstra/robot_arm_with_vision/blob/master/documentation/animation2.gif), inverse kinematics (IK), and forward kinematics (FK). It is designed for use with an Arduino Uno and the Adafruit PCA9685 PWM servo driver.

## Features

- **Inverse Kinematics:** Calculates joint angles for a desired end-effector position and orientation.
- **Forward Kinematics:** Computes the 3D positions of each joint for verification.
- **Serial Interface:** Accepts target positions via serial input in the format `x,y,z`.
- **Servo Control:** Maps calculated angles to servo positions and actuates servos using the PCA9685 module.
- **Debug Output:** Prints calculated angles and joint positions to the serial monitor for debugging.

## Hardware Requirements

- Arduino Uno (or compatible)
- Adafruit PCA9685 PWM Servo Driver
- 6 servo motors
- 6-DOF robot arm

## Usage

1. **Clone this repository** and open it in PlatformIO or the Arduino IDE.
2. **Update the robot's link lengths** (`D1`, `A2`, `D4`, `D6`) in the code to match your hardware.
3. **Upload the code** to your Arduino board.
4. **Open the serial monitor** at 9600 baud.
5. **Send a target position** in the format `x,y,z` (e.g., `120,50,80`).
6. The robot will move to the calculated position, and the joint angles and positions will be printed for verification.

## File Overview

- `src/main.cpp` — Main Arduino sketch: handles serial input, IK/FK calculations, and servo control.
- `src/IK.cpp`, `include/IK.h` — Inverse and forward kinematics implementation for the robot arm.

## Notes
- Update the robot's link lengths (D1, A2, D4, D6) in the code to match your hardware.
- Angles are calculated in radians and converted to degrees for servo control.
- Add a `.gitignore` file to exclude build files (e.g., `.pio/`).

---

**License:** MIT (or specify your license)  
**Author:** Danny Pham and Vadim Kukhotskiy