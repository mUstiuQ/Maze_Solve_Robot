# Maze Navigation and Mapping Robot

![](https://img.shields.io/badge/Arduino) ![](https://img.shields.io/badge/C++-FFDD33) ![](https://img.shields.io/badge/ArduinoIDE)

This project aims to develop a robot capable of autonomously navigating and mapping a maze using an Arduino development board. The robot utilizes an L298N motor driver for controlling the motors, three HC-SR04 ultrasonic sensors for distance measurements (front, left, and right), and WiFi/Bluetooth modules for wireless communication. The navigation logic is based on a Depth-First Search (DFS) algorithm that updates a 2D maze map in real time.

# Features
    Autonomous Navigation:
    The robot dynamically adjusts its path based on sensor readings, selecting the route with the most available space.

# Real-Time Maze Mapping:
    A 2D matrix represents the maze, where:

    2 marks the starting position or the cell where the robot begins.

    1 indicates an open path.

    0 indicates a wall.

    -1 indicates a dead-end.

    -2 indicates an unexplored cell.

## Arduino-Based Control:
    Developed using an Arduino board (e.g., Arduino Nano or Uno) with code written in C++ and compiled in the Arduino IDE.

## Wireless Communication:
    Integrated WiFi/Bluetooth modules allow for remote monitoring and control.

## Modular Design:
    The code is structured to be easily extensible and serves as a starting point for more advanced autonomous robotics projects.

## Hardware Components
    Development Board: Arduino Nano / Arduino Uno (or similar)

    Motor Driver: L298N

    Sensors: 3 x HC-SR04 ultrasonic sensors (for front, left, and right distance measurements)

    Wireless Modules: WiFi and Bluetooth modules (optional)

    Prototyping Tools: Breadboard, connecting wires, voltage regulator, and power supply

## Software Implementation
    The project is written in C++ using the Arduino core libraries. The key components include:

## Hardware Initialization:
    Configures the system clock, GPIOs, PWM outputs for motor control, and other peripherals within the Arduino setup() function.

## Sensor Readings:
    Functions to read distance values from the HC-SR04 ultrasonic sensors using the Trig/Echo protocol.

## Motor Control:
    Functions to move the robot forward, perform left/right turns, and stop using the L298N motor driver.

## Maze Mapping & Navigation:
    Implements a DFS-based exploration algorithm to update a 2D matrix representing the maze as the robot moves. The robot's position is tracked and updated based on movement commands and sensor feedback.

## Debug Interface:
    Debug messages are output via the Serial Monitor, providing real-time feedback on sensor readings and robot actions.

# Getting Started
    Prerequisites
    Hardware:
    Ensure you have an Arduino board (Nano, Uno, etc.), an L298N motor driver, HC-SR04 sensors, optional WiFi/Bluetooth modules, and necessary prototyping tools assembled as per the schematic.

# Software:
    Download and install the Arduino IDE. Familiarity with Arduino programming and basic electronics is beneficial.

# License
This project is licensed under the GPL 3.0 License. See the LICENSE file for more details.


