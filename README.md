# Zumo 32U4 Autonomous Navigation

An embedded robotics project implementing autonomous line-following and obstacle avoidance on the Pololu Zumo 32U4 robot. Integrates multi-sensor fusion, a PD controller, and an orientation filter for reliable real-time navigation.

## Overview
Uses five infrared line sensors, a front-facing proximity sensor, and a 6-axis IMU to navigate autonomously. When encountering an impassable obstacle, executes a precise 180-degree heading correction and reverses course. Completed as academic coursework at Hochschule Anhalt, 2024. **Author:** Nasser Al-Qeifi

## Features
- Line following with PD control (5 IR sensors, position 0-4000, target 2000)
- Proximity-based speed modulation (linear scaling, not binary stop/go)
- Obstacle avoidance with 180-degree heading-controlled turn
- Sensor fusion via MadgwickAHRS (quaternion-based, 25 Hz)
- 1 Euro Filter for adaptive noise reduction on accelerometer (60 Hz)
- Proportional heading control during turns (threshold +/-10 degrees)

## Hardware
- **Platform:** Pololu Zumo 32U4
- **MCU:** ATmega32U4
- All sensors (IR line array, proximity, IMU) and motor driver are onboard

## Key Constants
| Constant | Value |
|---|---|
| maxSpeed | 400 |
| GOAL (line center) | 2000 |
| FREQUENCY (1 Euro filter) | 60 Hz |
| HEADING_THRESHOLD | +/-10 degrees |
| PROPORTIONAL_GAIN | 2.0 |

## Dependencies (Arduino)
`Zumo32U4` (Pololu), `MadgwickAHRS`, `1euroFilter`, `Wire`

## Project Structure

```
zumo32u4-autonomous-navigation/
├── Code/
│   └── Code.ino        # Arduino sketch (line following + obstacle avoidance)
└── README.md
```

## License
MIT License
