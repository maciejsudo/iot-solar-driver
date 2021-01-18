**Work progress report**

5.12.2020 Update
- Subject of project was choosen.  
- This repository was created. 

21.12.2020 Update
- ESP Serial to MQTT relay was sucesfully created. 
- Prototype of node-red dashboard was created.  
- Prototype of node-red flow with mqtt data formatting ability was created.  

29.12.2020 Update  
 on STM32:  
 - 3 ADC channels with use of DMA were added 
 - Prototype of UART communication with basic data frame containing ADC conversion parameters was created 
  
07.01.2021 Update  
 on STM32:  
 - UART receiveing function was added (servo data from ESP)
 - prototype of PWM signal to control servo - with data received from ESP was added
 - Added feedback UART transmittion to ensure that servo is in the right position
  
12.01.2021 Update  
 on STM32:
 - Added light leading algorithm, with PWM generation to set the servo properly  
 - prototype of sleep mode function was added  
 - correction of UART Receiving function to set the right servo position based on received data   
 
15.01.2021 Update  
 - STM32F407VG code has been ported to the STM32F429ZIT so we can now work more time independly on the STM hardware side.  
 - Also during code port, serial data recieving ability was repaired so at this moment it works only on STM32F429ZIT.
 - Node Red prototype is no longer prototype, new funcions are added and most of the repairs and calibrations are done at this moment.  
 
16.01.2021 Update  
 - Independent Watchdog configuration was added to prevent software freeze  
