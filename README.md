# IoT Solar Panel Driver
# Introduction
This project is made for maintaining and monitoring low voltage solar panel charging process including position optimalization.  
Device measures the voltages and other parameters of the system then sends it via MQTT protocol so graphical dashborad with data can be presented to the user.  
Additionaly device optimalize the position of the panel using servo motor, so the best charging angle can be set.
# More technical info
Microcontroller measures the voltages on the output of the solar panel, battery which is being charged by the panel and the light level.
All of the measurements listed above are done using ADC channels of the microcontroller. In this case - STM32 with Cortex-M core.
Then the microcontroller sends the measured data via Serial protocol to the Espresiff ESP - in this project we used both the 8266 and 32.
ESP sends the data via MQTT protocol to the MQTT broker set in Node-RED server which is provided by Raspberry Pi.
Node-RED also provides graphical dashboard to user with visualised data over the time and simple controls of the system.
