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
 
