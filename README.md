# RoboCup - Third Place 2024

Welcome to the RoboCup Project repository! This project was designed and competed in the annual RoboCup competition for University of Canterbury third Yr Mechatronics, securing a **third-place finish**. This repository aims to maintain a well-documented structure for future enhancements, iterations, and collaboration in autonomous robotics.

## Table of Contents
1. [Project Overview](#project-overview)
2. [CAD](#setup-instructions)
3. [Code Structure](#code-structure)
6. [License](#license)

---

### Project Overview
This project is built around a robot designed to compete in RoboCup. There are three reports within this git that outlines the Robot in detail CDR , DDR and Competition report. Recommended reading is the Competition report as it outlines what worked and did not work in the competition. 


The robot is controlled by a Teensy 4.0 microcontroller with an IO expansion board. The teensy has 40 pins but 24 in the given configuration. Having a IO expander allows for the additional IO to be plugged in and multiplexed to. It also allows for standard connector types, serial, digital, i2c etc.
The code is developed and built in VS code as a platformIO project. This allows for greater functionality as the code written is C++ not Arduino C++. 



### CAD 
The robot has been designed to maximise simplicity and reliability. 
It uses sensors to gather information from the world. From this information it moves throughout area to gather weights and returns them to base.
To do this the robot must have primary motion system, the system implemented is a ‘tank drive’ system. The robot has ToF sensors that return distances from walls and obstacles which allow it to navigate throughout the area. 
The robot uses ultrasound sensors to identify all weights, it then uses the navigation system to rotate and drive towards the weight. The weight is collected then is kept on board with a “sheep gate” system using an inductive proximity sensor and servo. The inductive proximity sensor determines if the weight is a target or dummy. The robot then continues forward searching for more weights until it holds 3 on board where it returns home. 

See CAD for solidworks parts and refer to assembly to complete design

### Code Structure

The robot is controlled by a Teensy 4.0 microcontroller with an IO expansion board. The teensy has 40 pins but 24 in the given configuration. Having a IO expander allows for the additional IO to be plugged in and multiplexed to. It also allows for standard connector types, serial, digital, i2c etc.
The code is developed and built in VS code as a platformIO project. This allows for greater functionality as the code written is C++ not Arduino C++. 

The code is broken up into modules:
-	main.cpp: task scheduler set up and run. 
-	finite_state_machine.cpp: all states and logic to switch between them.
-	return_to_base.cpp: calculates where home is, if home and return home.
-	weight_collection.cpp: detects weights and functions to pick weight up.
-	smartmove.cpp: moves the robot based on sensors, move while avoiding walls etc.
-	motors.cpp: call to set motor/actuators values. 
-	sensors.cpp: takes in data from sensors.

### License
This project is licensed under the MIT License. See LICENSE for more details.
