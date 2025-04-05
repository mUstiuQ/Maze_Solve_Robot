# Maze Navigation and Mapping Robot

![](https://img.shields.io/badge/STM32F407G-8A2BE2) ![](https://img.shields.io/badge/C++-FFDD33) ![](https://img.shields.io/badge/STM32CUBE-F1)

This project aims to develop a robot capable of autonomously navigating and mapping a maze using an STM32F407 development board. The robot utilizes an L298N motor driver for controlling the motors, three HC-SR04 ultrasonic sensors for distance measurements (front, left, and right), and WiFi/Bluetooth modules for wireless communication. The navigation logic is based on a Depth-First Search (DFS) algorithm that updates a 2D maze map in real time.

# Features

  Autonomous Navigation:
   The robot dynamically adjusts its path based on sensor readings, selecting the route with the most available space.

   Real-Time Maze Mapping:
    A 2D matrix represents the maze, where:

     2 marks the starting position or the cell where the robot begins.

        1 indicates an open path.

        0 indicates a wall.

        -1 indicates a dead-end.

        -2 indicates an unexplored cell.

    STM32F407-Based Control:
    Developed on the STM32F407G-DISC1 board, using STM32 HAL libraries for hardware initialization and peripheral management.

    Wireless Communication:
    Integrated WiFi/Bluetooth modules allow for remote monitoring and control.

    Modular Design:
    The code is structured to be easily extensible and serves as a starting point for more advanced autonomous robotics projects.

# Hardware Components

    Development Board: STM32F407G-DISC1

    Motor Driver: L298N

    Sensors: 3 x HC-SR04 ultrasonic sensors (for front, left, and right distance measurements)

    Wireless Modules: WiFi and Bluetooth modules

    Prototyping Tools: Breadboard, connecting wires, voltage regulator, and power supply

# Software Implementation

The project is written in C and uses STM32 HAL libraries. The key components include:

    Hardware Initialization:
    Configures the system clock, GPIOs, PWM channels, and other peripherals. (Often generated using STM32CubeMX.)

    Sensor Readings:
    Functions to read distance values from the HC-SR04 ultrasonic sensors using the Trig/Echo protocol.

    Motor Control:
    Functions to move the robot forward, perform left/right turns, and stop using the L298N motor driver.

    Maze Mapping & Navigation:
    Implements a DFS-based exploration algorithm to update a 2D matrix representing the maze as the robot moves. The robot's position is tracked and updated based on movement commands and sensor feedback.

    Debug Interface:
    Debug messages are output via UART (or another method) to simulate LCD display updates, providing real-time feedback on sensor readings and robot actions.

# Getting Started
Prerequisites

    Hardware:
    Ensure you have the STM32F407G-DISC1 board, L298N motor driver, HC-SR04 sensors, WiFi/Bluetooth modules, and necessary prototyping tools assembled as per the schematic.

    Software:
    STM32CubeIDE is recommended for compiling and flashing the code onto the STM32 board. Familiarity with STM32 HAL libraries is beneficial.

# License

This project is licensed under the GPL 3.0 License. See the LICENSE file for more details.
