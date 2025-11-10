# Cargo Bot - Autonomous Line Following Robot

[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

An autonomous cargo delivery robot built with Raspberry Pi Pico, featuring QR code scanning, line following with PID control, and precise box manipulation using a linear actuator. Working in collaboration with my team on the IDP project.

<div align="center">

![Cargo Bot](docs/images/test.gif)

</div>

## Features

- **PID Line Following**: Smooth line tracking with adaptive PID control and post-turn boost for quick recovery.
- **QR Code Navigation**: Scans QR codes to determine delivery destinations dynamically.
- **Precision Actuator Control**: Linear actuator for picking up and placing boxes at different rack heights.
- **Multi-Sensor Integration**: Distance sensors (VL53L0X, TMF8701), line sensors, and QR scanner running concurrently.
- **Autonomous Navigation**: Navigates complex paths with intersection detection and precise turning.

## Technical Implementation

The project is implemented in MicroPython and utilises custom hardware drivers and control algorithms.

| Component | Detail |
|---|---|
| **Microcontroller** | Raspberry Pi Pico |
| **Framework** | MicroPython |
| **Language** | Python |
| **Control Algorithm** | PID with filtered derivative and integral windup prevention |
| **Sensors** | 4x Line sensors, VL53L0X ToF, TMF8701 ToF, Tiny Code Reader (QR) |
| **Actuators** | 2x DC Motors, Linear Actuator |

## Getting Started

### Prerequisites

-   Raspberry Pi Pico with MicroPython installed
-   VS Code with the Pico extension or similar MicroPython development environment

### Setup and Deploy

1.  Clone the repository:
    ```sh
    git clone https://github.com/kfilipekk/Cargo-Bot.git
    cd Cargo-Bot
    ```

2.  Connect your Raspberry Pi Pico to your computer

3.  Upload all project files to the Pico

4.  Run `main.py` on the Pico

## Requirements

-   **Hardware**:
    -   Raspberry Pi Pico
    -   2x DC Motors with H-Bridge driver
    -   Linear Actuator with motor driver
    -   4x IR Line Sensors
    -   VL53L0X Time-of-Flight Distance Sensor
    -   TMF8701 Time-of-Flight Distance Sensor
    -   Tiny Code Reader (QR Scanner)
    -   2x LEDs (status indicators)
    -   Chassis and mounting hardware
    -   Jumper wires and breadboard/PCB
-   **Software**:
    -   MicroPython firmware for Raspberry Pi Pico
    -   VS Code with Pico extension (recommended)

## Project Structure

```
Cargo_Bot/
├── main.py                 # Main navigation and mission control
├── sw/                     # Software modules
│   ├── constants.py        # Robot configuration and PID parameters
│   ├── motor_functions.py  # Motor control and turning functions
│   ├── line_follower.py    # PID line following algorithm
│   ├── sensors.py          # Sensor integration and threading
│   ├── linear_actuator.py  # Actuator control for box manipulation
│   ├── leds.py            # Status LED control
│   └── ultrasonic.py      # Ultrasonic distance sensor
├── libs/                   # Hardware driver libraries
│   ├── tiny_code_reader/   # QR code scanner driver
│   ├── DFRobot_TMF8x01/   # TMF8701 ToF sensor driver
│   └── VL53L0X/           # VL53L0X ToF sensor driver
└── support/               # Documentation and reference materials
```

## Contributors

- [kfilipekk](https://github.com/kfilipekk)
- [aoj23](https://github.com/aoj23)
- [ks2171](https://github.com/ks2171)

<div align="center">

Developed by [kfilipekk](https://github.com/kfilipekk)

</div>
