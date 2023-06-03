# Inclinometer-using-BNO055-and-ESP8266
This repository contains the Arduino code for an inclinometer project based on the BNO055 sensor and the ESP8266 module. The purpose of this project is to measure the inclination or tilt of an object with respect to gravity and send this data to ThingSpeak for visualizing and monitoring.

Key features:

•Uses BNO055, a 9-DOF sensor with onboard fusion algorithms.

•Calculates the pitch and roll angles to measure the inclination.

•Employs a basic form of sensor fusion, combining accelerometer and gyroscope data, to increase the accuracy of the measurements.

•Utilizes the SHT2x sensor to measure temperature and humidity for additional environmental information.

•Connects to the Wi-Fi network using the ESP8266 module and sends the data to ThingSpeak for real-time visualization and analysis.

•This code is perfect for hobbyists and professionals looking to integrate an inclinometer into their projects, or for those interested in learning more about sensor fusion and IoT data transmission.

Note: Always ensure to enter your Wi-Fi and ThingSpeak channel details in the given variables. Be mindful to calibrate the BNO055 sensor for optimal performance.

The code is well commented, and we welcome any contributions or suggestions for improvements. Enjoy measuring your world!

Note: This repository only contains the source code. Make sure to install all necessary libraries and board definitions in your Arduino IDE or PlatformIO setup. Please check the comments in the code for further instructions.

General view of the inclinometer:

![image](https://github.com/GeodeZister/Inclinometer-using-BNO055-and-ESP8266/assets/97829206/cb0049b6-0d53-4f92-885b-2a3b3b15f21c)


![image](https://github.com/GeodeZister/Inclinometer-using-BNO055-and-ESP8266/assets/97829206/8482cff5-7dae-4ad9-82f8-1a40cd405149)



