#Project-VAJRA---Smart-IOT-Car

This repository contains the code and documentation for the BMS Car, a 4-wheeler autonomous and remote-controlled car developed as a final year project. The project uses an Arduino Uno, L298N motor driver, ESP32 for MQTT communication, ultrasonic sensor, servo, LEDs, LDR, and buzzer to implement various functionalities. Below are the completed or attempted milestones.


Milestones:

Milestone 1: Remote Control (RC)
Purpose: Enable wireless control of the car via MQTT.
Description: Implemented remote control using an ESP32 module communicating over MQTT (topic: car/control). Commands include FORWARD, BACKWARD, LEFT, RIGHT, and STOP, controlling the L298N motor driver (pins 9, 7, 6, 3, 5, 4). The car responds to commands sent from an MQTT client (e.g., MQTT Dash).

Milestone 2: Self-Driving Mode (Attempted and not commited to github)
Description: Attempted to implement self-driving using a backward-facing ultrasonic sensor (TRIG 11, ECHO 10) on a servo (pin 2, angles 180°, 270°, 350°). The car checks rear distance; moves forward if >20cm, else stops, scans right/left, and turns toward the clearer side or backs up. This feature is incomplete due to navigation issues.

Milestone 3: Environment Sensing
Description: Integrated an LDR (pin A0) to detect light levels. If LDR value <300, the white LED (pin A5) turns on, and red LEDs (pins A1, A2, PWM 100) activate. Manual override is supported via MQTT commands (LIGHTS_ON, LIGHTS_OFF).

Milestone 4: Auto-Braking
Description: Implemented auto-braking in FORWARD and FOLLOW modes using the ultrasonic sensor. If an obstacle is <15cm, the car stops, red LEDs (pins A1, A2) switch to PWM 255, and the buzzer (pin A4) sounds for 1 second.

Milestone 5: Driving Modes
Description: Added three modes via MQTT: STRADA (150 PWM, 5s ramp), SPORT (200 PWM, 3s ramp), and CORSA (255 PWM, instant). Modes adjust motor speed using the L298N driver, set by MQTT commands (STRADA, SPORT, CORSA).
