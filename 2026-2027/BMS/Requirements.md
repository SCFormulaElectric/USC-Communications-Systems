**Motherboard**



Discharge

\- If any daughterboard reports a fault, save to EEPROM the details of the fault \& signal the relay board

\- Sense current \& report over CAN

\- Report voltage 

\- Calculate battery health

\- Report min max avg temps



Charge

\- If CAN requests charging, talk to ELCON

\- Monitor and report SOC





&#x20;- UART
 - CAN

&#x20;- EXT OSC

&#x20;**-** SPI, communicate with BMS chips

**Daughterboard**



&#x20;- TI's proprietary Daisy Chain Interface with SPI (ref: https://www.ti.com/lit/an/sluaa17a/sluaa17a.pdf?ts=1787015738028)

&#x20;

