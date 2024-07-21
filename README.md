# PGNTA: 17 DOF Humanoid Arduino Robot

## Introduction

During my first-year summer break, I embarked on an ambitious project to create PGNTA, a humanoid Arduino robot. This project was born out of a fascination with robotics and a desire to challenge myself in the fields of electronics, programming, and mechanical design. PGNTA, standing at 53 cm tall, represents my first foray into complex robotics and serves as a testament to what can be achieved with determination and a willingness to learn.

<div align="center">
    <img src="webimages/1.webp" height="300" width="300" alt="PGNTA Version 1">
</div>

## Project Overview

PGNTA is a 17 Degree of Freedom (DOF) humanoid robot powered by an ESP32 microcontroller. Built primarily from wooden plywood, it combines servo motors, sensors, and custom-designed joints to create a functional bipedal structure.

### Key Features:

- 17 DOF using [MG995](webimages/hardware/mg995.webp) and [SG90](webimages/hardware/sg90.webp) servo motors
- [ESP32](webimages/hardware/esp32.webp) a microcontroller for main control
- Custom wooden plywood frame
- Ultrasonic sensor ([HC-SR04](webimages/hardware/hcsr04.webp)) and ([MPU6050](webimages/hardware/mpu6050.webp)) for balance and object detection
- Arduino Cloud integration for live data fetching
- Dimensions: 53 cm height, 25 cm length, 8 cm width

<div align="center">
    <img src="webimages/2.webp" height="500" alt="Overview">
</div>

### Objectives:

1. Design and build a functional humanoid robot using Arduino technology
2. Gain practical experience in servo motor control and power management
3. Explore challenges in balance and movement for bipedal structures
4. Create a platform for future enhancements and learning in robotics
5. Integrate sensors and cloud connectivity for advanced functionality

<div align="center">
    <img src="webimages/4.webp" height="200" alt="Robot during construction">
</div>

## Development Process

### 1. Component Selection

The construction of PGNTA involved careful selection of components to balance functionality, cost, and availability. The key components included:

- Servo Motors: 14 [MG995](webimages/hardware/mg995.webp) A high-torque servos for major joints and 3 [SG90](webimages/hardware/sg90.webp) servos for less demanding movements
- Microcontroller: [ESP 32](webimages/hardware/esp32.webp) for main control
- Sensors: Ultrasonic sensor ([HC-SR04](webimages/hardware/hcsr04.webp)) for object detection and [MPU6050](webimages/hardware/mpu6050.webp) for acceleration and gyroscopic data
- Additional electronics: [PCA 9685](webimages/hardware/pca9685.webp) 16-channel servo driver, Arduino Nano for interfacing with servo control software

<div align="center">
    <img src="webimages/hardware/all.webp" height="300" alt="Component layout">
</div>

### 2. Structural Design

PGNTA's structure was primarily built using 3mm wooden plywood, chosen for its availability, ease of modification, and cost-effectiveness. The robot's frame consists of:

- Central body: 14x11 cm plywood housing core servos
- Limbs: Custom box-like structures using smaller plywood pieces
- Joints: C-shaped hinge joints created from plywood for flexibility
- Head: 6x3x5 cm cube-shaped structure housing sensors

The frame was constructed using the following techniques:

<div align="center">
    <img src="webimages/0.webp" height="400" alt="Structural Design">
</div>

- Rectangular plywood pieces were cut and joined using adhesive
- Each servo was secured with four 2mm screws and nuts
- 7mm holes were drilled for servo horns
- Additional support was provided using nails on the opposite side of servo horns

### 3. Servo Configuration

The 17 servos were strategically placed to mimic human joint movements:

- Legs (10 servos): LL1-LL5 and RL1-RL5
- Arms (6 servos): LH1-LH3 and RH1-RH3
- Head (1 servo): For rotational movement

### 4. Power Management

Initial power supply using 9V batteries proved insufficient. The final solution involved:

- Six 18650 2000mAh 3.7V batteries
- Configuration: 2 parallel sets of 3 batteries in series
- Output: Approximately 11.5V with higher current capacity
- DC-DC Buck converter providing 5V output

<div align="center">
    <img src="webimages/hardware/battery.webp" height="200" alt="Power Management">
</div>

### 5. Programming and Control

- Arduino IDE was used for programming
- PWM control implemented for servo movements
- [Tim's Servo x16 Controller](https://tims-pc-applications.blogspot.com/2020/05/tim-servo-x16-controller.html) utilized for fine-tuning servo positions
- Integration with [Arduino Cloud](https://app.arduino.cc) for remote monitoring and control

<div align="center">
    <img src="webimages/iot.png" height="300" alt="Arduino Cloud">
</div>

## Circuit Documentation

### ESP32 Version

This circuit manages precise servo motor movements using an Adafruit PCA9685 PWM Servo Breakout board, interfaced with an ESP32 microcontroller.

#### Component List:

- ESP32: Interfaces with the servo control software and manages PWM signals.
- 18650 Li-ion Batteries: Rechargeable batteries providing the power source.
- Adafruit PCA9685 PWM Servo Breakout: 16-channel, 12-bit PWM I2C-controlled servo driver.
- Servos: Actuators precisely controlled for position.
- Terminal PCB 2 Pin: Connectors for power supply.
- Electrolytic Capacitor: 25V 100μF capacitor for voltage smoothing.
- Ultrasonic Sensor (HC-SR04): For object detection.
- MPU6050: For acceleration and gyroscopic data.
- Circular LED: Connected to the ESP32 for visual indication.

#### Wiring Details:

(Detailed wiring information for ESP32, sensors, and servos) [Click here](webimages/circuit/circuitesp32.webp)  
Circuit Documentation for ESP32 [Click here](code/esp32.html)

### Arduino Nano Version

This circuit is an alternative version using an Arduino Nano microcontroller.

#### Component List:

- Arduino Nano: Based on the ATmega328, interfacing with the servo control software.
- 18650 Li-ion Batteries: Rechargeable batteries providing the power source.
- Adafruit PCA9685 PWM Servo Breakout: 16-channel, 12-bit PWM I2C-controlled servo driver.
- Servos: Actuators precisely controlled for position.
- Terminal PCB 2 Pin: Connectors for power supply.
- Electrolytic Capacitor: 25V 100μF capacitor for voltage smoothing.

#### Wiring Details:

(Detailed wiring information for Arduino Nano, sensors, and servos) [Click here](webimages/circuit/circuitnano.jpg)  
Circuit Documentation for Arduino Nano [Click here](code/nano.html)

<div align="center">
    <img src="webimages/circuit/commoncircuit.webp" height="250" width="400" alt="Circuit diagram">
</div>

## Code
- [ESP 32 Version](code/esp32.md)
- [Arduino Nano Version](code/nano.md)

## Challenges & Solutions

1. **Servo Movement Limitations**: 
   - Challenges: MG995 servos mechanically limited to 0-180 degrees
   - Solutions: Used Tim's Servo x16 Controller software to fine-tune servo positions and created preset configurations and scripts for movements

2. **Power Supply Issues**: 
   - Challenge: Initial power supply (9V batteries) insufficient for high current draw
   - Solution: Switched to 18650 Li-ion batteries for higher current capacity and implemented a 2S3P configuration for balanced voltage and current

3. **Weight Management**: 
   - Challenge: Excessive weight due to 3mm plywood frame
   - Solution: Future versions to consider lighter materials and ongoing optimization of design for weight reduction

4. **Assembly Mistakes**: 
   - Challenges: Servos joined to body without prior testing, restricting some motions
   - Solution: Learned the importance of component testing before final assembly and future iterations to include modular design for easier modifications

5. **Wiring and Connections**: 
   - Challenge: Improper soldering of pins in PWM 9685 boards caused connection issues
   - Solutions: Re-soldered connections using a 30-watt soldering iron and gained hands-on soldering experience

## Future Improvements (Planned Phases)

### Version 2

- Implement web-based control interface
- Enhance movement fluidity and range

### Version 3

- Integrate ESP32-CAM module for image detection
- Implement real-time movement following human actions

### Version 4

- Integrate AI API for advanced interactions
- Implement voice control and natural language processing


##### If you have any questions, suggestions, feel free to contact below


<table> 
   <tr>
      <td><a href="mailto:pgnta.1385@gmail.com" >
      <img src="webimages/logo/g.png" height="40" width ="">
      </a></td>
      <td><a href="https://www.linkedin.com/posts/prashant7579_arduinoproject-robotics-humanoidrobot-activity-7220639796921085952-nnRn?utm_source=share&utm_medium=member_desktop">
      <img src="webimages/logo/ld.png" height="40" width="50">
      </a></td>
      <td><a href="https://github.com/rebelpg/project" >
      <img src="webimages/logo/git.png" height="40" width ="">
      </a></td>
      <td><a href="https://www.youtube.com/@rebelpg7579" >
         <img src="webimages/logo/yt.png" height="40" width ="">
      </a></td>
      <td><a href="https://www.instagram.com/rebel.pg/" >
      <img src="webimages/logo/ig.png" height="40" width ="">
      </a></td>
   </tr>
</table>

<div align="center">
    <h1>&copy; 2024 PGNTA. All rights reserved.</b></h1>
    </div>
