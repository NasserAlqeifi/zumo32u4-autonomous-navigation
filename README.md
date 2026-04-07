# Zumo 32U4 Autonomous Navigation

An embedded robotics project implementing autonomous line-following and obstacle avoidance on the Pololu Zumo 32U4 robot. The system integrates multi-sensor fusion, a PD controller, and an orientation filter to enable reliable real-time navigation on a compact robotic platform.

---

## Overview

This project was developed to enhance the navigation capabilities of the Zumo 32U4 robot beyond its out-of-the-box behavior. The robot uses five infrared line sensors, a front-facing proximity sensor, and a 6-axis IMU (accelerometer + gyroscope) to navigate autonomously. When the robot encounters an obstacle it cannot pass, it executes a precise 180-degree heading correction and reverses course.

The project was completed as part of academic coursework at **Hochschule Anhalt** (Anhalt University of Applied Sciences) in 2024.

**Author:** Nasser Al-Qeifi

---

## Features

- **Line following with PD control** - Reads 5 infrared sensors and computes a proportional-derivative adjustment to keep the robot centered on the line
- **Proximity-based speed modulation** - Continuously scales motor speed based on how close an obstacle is, rather than a simple stop/go threshold
- **Obstacle avoidance with 180-degree turn** - When fully blocked, the robot halts, waits 5 seconds, rechecks, and executes a heading-controlled rotation to reverse direction
- **Sensor fusion via MadgwickAHRS** - Fuses accelerometer and gyroscope data into stable roll, pitch, and yaw estimates using an efficient quaternion-based algorithm
- **1 Euro Filter for noise reduction** - Applies adaptive low-pass filtering to raw accelerometer readings before passing them to the orientation filter, reducing jitter without introducing lag
- **Proportional heading control during turns** - The 180-degree turn uses a closed-loop proportional controller to reach the target heading accurately rather than relying on a fixed time delay

---

## Hardware Requirements

| Component | Details |
|-----------|---------|
| **Robot platform** | Pololu Zumo 32U4 Robot |
| **Microcontroller** | ATmega32U4 AVR (onboard) |
| **Line sensors** | 5 x IR reflectance sensors (onboard) |
| **Proximity sensor** | Front-facing IR proximity sensor (onboard) |
| **IMU** | 6-axis accelerometer + gyroscope (onboard) |
| **Motor driver** | Dual H-bridge (onboard) |
| **Interface** | USB via Arduino-compatible bootloader |

The Zumo 32U4 is a self-contained platform. No external wiring or additional hardware is required beyond a USB cable for programming.

---

## Software Requirements

| Requirement | Version / Notes |
|-------------|-----------------|
| **Arduino IDE** | 1.8.x or 2.x |
| **Zumo32U4 library** | Latest from Pololu |
| **MadgwickAHRS library** | Standard Arduino library |
| **1euroFilter library** | Available via Arduino Library Manager |
| **Wire library** | Built into Arduino (I2C communication) |

---

## Architecture

The firmware runs on a single-threaded loop with the following high-level stages:

```
┌─────────────────────────────────────────┐
│              setup()                    │
│  - Calibrate line sensors               │
│  - Initialize IMU                       │
│  - Configure 1 Euro Filters (60 Hz)     │
│  - Start Madgwick filter (25 Hz)        │
└───────────────────┬─────────────────────┘
                    │
                    ▼
┌─────────────────────────────────────────┐
│              loop()                     │
│                                         │
│  1. Read proximity sensor               │
│  2. Calculate proximity factor          │
│     - factor = 0.0 means full stop      │
│     - factor = 1.0 means full speed     │
│                                         │
│  3. If factor == 0:                     │
│     a. Stop motors                      │
│     b. Wait 5 seconds                   │
│     c. Recheck proximity                │
│     d. If still blocked: turn 180 deg   │
│                                         │
│  4. If factor != 0:                     │
│     a. Read line sensor position        │
│     b. Compute PD error and adjustment  │
│     c. Scale speed by proximity factor  │
│     d. Set motor speeds                 │
└─────────────────────────────────────────┘
```

---

## Module Descriptions

### PD Controller (Line Following)

The line sensor array returns a weighted position value from 0 to 4000. The target position (center of the line) is set to `GOAL = 2000`. The controller computes:

```
error      = position - GOAL
adjustment = error / 3.5  +  0.5 * (error - lastError)
```

The proportional term corrects for the current deviation. The derivative term dampens oscillation by reacting to how fast the error is changing. Motor speeds are then:

```
leftSpeed  = speed + adjustment
rightSpeed = speed - adjustment
```

Both values are constrained to `[0, maxSpeed]` (maxSpeed = 400).

### Proximity Factor

The front proximity sensor returns counts from 0 (no obstacle) to 6 (obstacle within ~5-15 cm). The proximity factor is:

```
factor = (6 - min(leftReading, rightReading)) / 6
```

This maps sensor proximity linearly to a `[0.0, 1.0]` range. Both speed and the PD adjustment are multiplied by this factor, so the robot slows down smoothly as it approaches an obstacle rather than stopping abruptly.

### AHRS (Attitude and Heading Reference System)

The `AHRS()` function runs at 25 Hz. It:

1. Reads raw accelerometer and gyroscope values from the IMU
2. Passes raw accelerometer values through three independent 1 Euro Filters (one per axis)
3. Converts filtered accelerometer data to gravity units (`g = raw * 2.0 / 32768.0`)
4. Converts raw gyroscope data to degrees per second (`dps = raw * 245.0 / 32768.0`)
5. Feeds both into the Madgwick filter to update the quaternion state
6. Extracts roll, pitch, and heading (yaw) from the updated quaternion

### 1 Euro Filter

The 1 Euro Filter is an adaptive low-pass filter designed for real-time signal processing. It reduces noise at low signal speeds while allowing fast response to genuine rapid changes. Parameters used:

- **Frequency:** 60 Hz (update rate)
- **Min cutoff:** 3.0 Hz (noise suppression at rest)
- **Beta:** 0.1 (speed adaptation coefficient)

One filter instance is created per accelerometer axis (X, Y, Z).

### Orientation-Controlled Turn

`turnRobotToHeading(float turnAngle)` computes a target heading by adding the turn angle to the current heading and normalizing the result to `[0, 360)`. It then runs a closed control loop:

1. Call `AHRS()` to get the latest heading
2. Compute the signed angular error (shortest path, handling 0/360 wraparound)
3. If `|error| <= 10 degrees`, exit the loop
4. Otherwise compute a proportional motor speed:
   ```
   speed = max(BASE_SPEED, MAX_SPEED - |error| * PROPORTIONAL_GAIN)
   ```
   (BASE_SPEED = 100, MAX_SPEED = 400, PROPORTIONAL_GAIN = 2.0)
5. Apply speed to motors in opposite directions to spin in place
6. Delay 20 ms and repeat

---

## Getting Started

### 1. Install dependencies

Open the Arduino IDE and install the following libraries via **Sketch > Include Library > Manage Libraries**:

- `Zumo32U4` by Pololu
- `MadgwickAHRS`
- `1euroFilter`

### 2. Open the sketch

Open `Code/Code.ino` in the Arduino IDE.

### 3. Connect the robot

Connect the Zumo 32U4 to your computer via USB. Select the correct board and port:

- **Board:** Pololu A-Star 32U4 (or Arduino Leonardo)
- **Port:** the COM port assigned to the robot

### 4. Upload

Click **Upload**. The robot will begin sensor calibration immediately after upload. During calibration it sweeps left and right to read the line. Place it on the track before uploading, or power-cycle it on the track after uploading.

### 5. Run

Place the robot on a line course (dark line on light surface). The robot will begin following the line automatically, slow down near obstacles, and attempt a 180-degree turn if blocked.

---

## Project Structure

```
zumo32u4-autonomous-navigation/
└── Code/
    └── Code.ino       # Main Arduino sketch (all firmware logic)
```

---

## Key Constants Reference

| Constant | Value | Description |
|----------|-------|-------------|
| `maxSpeed` | 400 | Maximum motor speed (0-400 scale) |
| `GOAL` | 2000 | Target line sensor position (center) |
| `turnSpeed` | 200 | Base speed used during turns |
| `NUM_SENSORS` | 5 | Number of active line sensors |
| `FREQUENCY` | 60 Hz | 1 Euro Filter update frequency |
| `MINCUTOFF` | 3.0 Hz | 1 Euro Filter minimum cutoff |
| `BETA` | 0.1 | 1 Euro Filter speed adaptation |
| `HEADING_THRESHOLD` | 10 deg | Tolerance for turn completion |
| `PROPORTIONAL_GAIN` | 2.0 | Gain for heading proportional controller |
| `sensorThreshold` | 6 | Max proximity sensor reading |

---

## References

- Casiez, G. et al. "1 Euro Filter" - https://www.arduino.cc/reference/en/libraries/1eurofilter/
- Madgwick, S. "An efficient orientation filter for inertial and inertial/magnetic sensor arrays" - https://ahrs.readthedocs.io/en/latest/filters/madgwick.html
- Pololu Robotics and Electronics. "Pololu Zumo 32U4 Robot User's Guide" - https://www.pololu.com/docs/0J63/1

---

## License

This project is released under the [MIT License](LICENSE).
