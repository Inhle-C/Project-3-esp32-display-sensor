# Project 3: Display Humidity and Temperature with ESP32

**Part 3 of my IoT Lab Series using ESP32 and Raspberry Pi 4**

## Overview

This project builds on the previous labs by adding a display interface and integrating sensor data output in real time.  
It involves building a custom C++ library for controlling the I2C RGB display on the ESP32-C3 board and displaying temperature and humidity readings from the SHTC3 sensor.

The goal is to develop skills in writing hardware-level C++ libraries and integrating I2C peripherals.

## Objectives

- ✅ Solder pin headers to ESP32 board for I2C display and Vdd control
- ✅ Develop a C++ library to control the RGB LCD display using ESP32 I2C calls
- ✅ Respect the API of DFRobot_LCD.h but re-implement it for ESP32
- ✅ Display "Hello CSE121!" and surname on the LCD
- ✅ Read temperature and humidity data from SHTC3 sensor
- ✅ Update temperature in Celsius on the first line of the LCD every second

## Project Structure

esp32-lab3-display-sensor/  
├── report.pdf # Lab report (required)  
├── lab3_2/ # Lab 3.2: Display Library  
│ ├── sdkconfig  
│ ├── CMakeLists.txt  
│ ├── README.md  
│ └── main/  
│ ├── CMakeLists.txt  
│ ├── main.cpp  
│ └── lcd_display.h  
├── lab3_3/ # Lab 3.3: Integrate Sensor and Display  
│ ├── sdkconfig  
│ ├── CMakeLists.txt  
│ ├── README.md  
│ └── main/  
│ ├── CMakeLists.txt  
│ ├── main.cpp  
│ └── lcd_display.h  
│ └── sensor.h  


## Setup Instructions

### 🛠️ Lab 3.1: Soldering Pins

- Solder pin headers to the ESP32 board
- Ensure proper connections for I2C and Vdd for the display

### 💻 Lab 3.2: Display Library

1. Clone the Arduino library as a reference:  
https://github.com/DFRobot/DFRobot_RGBLCD1602  

2. Create a new C++ project for ESP32
- Do not use Arduino's Print or Wire classes
- Use ESP32 native I2C APIs
- Maintain the API of DFRobot_LCD.h but re-implement internals

3. Expected output on the display:
Hello CSE121!  
YourLastName  

4. Recommended main loop:
while (true) {
  lcd.init();
  lcd.setRGB("Hello CSE121!");
  lcd.setCursor(0, 1);
  lcd.printstr("YourLastName");
}

### 🌡️ Lab 3.3: Integrate Humidity and Temperature

1. Read sensor data from SHTC3 every second

2. Update display:
- Line 1: Temperature in Celsius
- Line 2: Humidity percentage

3. Verify live updates on the display during project check-off

## Notes

- Exclude build/ directories when zipping the project.
- Submit the required directories only: lab3_2/* and lab3_3/*
- Document issues or learnings in report.pdf and subfolder README.md.
- All external code must follow APACHE or BSD-like licenses.
- Reference any helpful resources properly in report.pdf (No StackOverflow, Reddit).

## What I Learned

- Soldering and hardware setup for ESP32 peripherals
- Developing C++ libraries for embedded systems
- Using ESP32 I2C drivers to control hardware
- Displaying dynamic sensor data on an RGB LCD
- Integrating multiple peripherals (display + sensor) on shared I2C bus

## Future Improvements

- Improve display readability with custom characters or formatting
- Add Fahrenheit output alongside Celsius
- Display timestamp or logging data on LCD
- Expand to display additional sensors or messages

## License

This project is for educational purposes.

Previous Project: [ESP32 Debugging and Sensor Integration](https://github.com/Inhle-C/Project-2-esp32-lab2-debugging-sensor)  
(Part 2 of the series)

Next Project: ESP32 Sensor Data Logger 🔗  
(To be uploaded as Part 4 of the series)
