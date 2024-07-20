# Circuit Documentation for Arduino Nano

## Summary

This circuit manages precise servo motor movements using an Adafruit PCA9685 PWM Servo Breakout board, interfaced with an Arduino Nano microcontroller. The servos are powered by 18650 Li-ion batteries, with a rocker switch to manage the power supply. The Adafruit PCA9685 allows independent control of up to 16 servos through the I2C interface with the Arduino Nano. The circuit features a capacitor for voltage smoothing and terminal PCBs for connecting the batteries and servos. The Arduino Nano handles interfacing with the servo control software, managing PWM signals for accurate positioning and coordination of the servos.

## Component List

### Microcontroller
- **Arduino Nano**: Based on the ATmega328P, interfaces with the servo control software and manages PWM signals.

### Power Supply
- **18650 Li-ion Batteries**: Rechargeable batteries providing the power source.
- **Rocker Switch**: Controls the power flow to the circuit.

### Servo Control
- **Adafruit PCA9685 PWM Servo Breakout**: 16-channel, 12-bit PWM I2C-controlled servo driver for precise control of multiple servos.

### Servos
- **Servos**: Actuators precisely controlled for position, receiving PWM signals from the Adafruit PCA9685.

### Power Distribution
- **Terminal PCB 2 Pin**: Connectors for making quick and reversible connections of the power supply.

### Passive Components
- **Electrolytic Capacitor**: Capacitor for voltage smoothing in the power supply.

## Wiring Details

### Arduino Nano
- **5V**: → 5.0V pin of the Adafruit PCA9685
- **GND**: → GND pin of the Adafruit PCA9685
- **A4 (SDA)**: → SDA pin of the Adafruit PCA9685
- **A5 (SCL)**: → SCL pin of the Adafruit PCA9685

### Adafruit PCA9685 PWM Servo Breakout
- **5.0V**: → 5V pin of the Arduino Nano and VCC of the servos
- **GND**: → GND pin of the Arduino Nano and servos
- **SDA**: → A4 pin of the Arduino Nano
- **SCL**: → A5 pin of the Arduino Nano
- **PWM0 - PWM15**: → Pulse pin of corresponding servo

### Servos
- **VCC**: → 5.0V pin of the Adafruit PCA9685
- **GND**: → GND pin of the Adafruit PCA9685
- **Pulse**: → Corresponding PWM pin on the Adafruit PCA9685

### 18650 Li-ion Batteries
- **+**: → Rocker switch and other batteries as required for desired voltage and capacity
- **-**: → Ground distribution via terminal PCBs

### Rocker Switch
- **Input**: → Positive terminal of the battery pack
- **Output**: → Power distribution network and Adafruit PCA9685

### Terminal PCB 2 Pin
- Connects the power supply to the Adafruit PCA9685 and servos.

### Electrolytic Capacitor
- **+**: → Positive side of the power supply
- **-**: → Ground side of the power supply

## Documented Code

- [Download Tims_PCA_9685_Controller](https://drive.google.com/file/d/1oVNN0d4eN6NiO2mBfU_Uc30aRuDH89zs/view?usp=sharing) 

- I prefer you go through this --> [Tim's PC Applications Blogspot](https://tims-pc-applications.blogspot.com/2020/05/tim-servo-x16-controller.html) 


## Initial Setup Mistake
- Assembled without testing components.
- Caused issues with MG995 servo's mechanical range (0-180 degrees).

## Another Problems Arised
- Tim PCA controller did not support ESP32.
- Had to use Arduino Nano connected to PCA9685 and then to PC.
- Adjusted servo settings using the software to map required servo angles to pulses.

## PWM Control Process
- Set frequency using the slide bar to control servo angle.
- Mapped current PWM values to the script field.
- Added delays between scripts using delay button.
- Set minimum and maximum pulse width.
- Uploaded X file to Arduino using XLoader.
- Saved the PWM script file for controlling robot movement.

## Movement Scripts
- Provided sample scripts for walking and hand dance movements.

## Future Plans

- Implement machine learning for movement patterns.




