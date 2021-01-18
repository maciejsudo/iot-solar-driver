# IoT Solar Panel Driver
# Introduction
This project is made for maintaining and monitoring low voltage solar panel charging process including position optimalization.
Device measures the voltages and other parameters of the system then sends it via MQTT protocol so graphical dashborad with data can be presented to the user.
Additionaly device optimalize the position of the panel using servo motor, so the best charging angle can be set.
# General technical info
Microcontroller measures the voltages on the output of the solar panel, battery which is being charged by the panel and the light balance level. Microcontroller also manages servo position of the solar panel based on light source position in automatic mode.
All of the measurements listed above are done using ADC channels of the microcontroller.
Then the microcontroller sends the measured data via Serial protocol to the Espresiff ESP.
ESP sends the data via MQTT protocol to the MQTT broker set along with in Node-RED server.
Node-RED also provides graphical dashboard to user with visualised data over the time and simple control of the servo position in case of manual mode choosen.
# More technical info  
In this project STM32F407VG or STM32F429ZIT can be used as microcontroller, serial to mqtt relay can be provided by ESP32 or ESP8266. Node-Red along with MQTT broker can be set both on any  Raspberry Pi model on Node-Red compatibile AWS EC2 instance. Block diagram which is ilustrating entire system can be found below:   
  
<img src="https://github.com/bielakjacek/iot-solar-driver/blob/main/block-diagram/iot-solar-driver-block-diagram.svg">  

# To be continued...
