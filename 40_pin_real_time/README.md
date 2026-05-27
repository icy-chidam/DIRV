This file contains the working Python codes for transmitting the configuration file to radar sensor without using the USB cable
Alternatively jumper wires are used for the transmission of configuration and the data is taken out from MSS LOGGER.

USB TO TTL ADAPTER To MMWAVEICBOOST Pin Connections:

1. DURING CONFIGURATION 

TXD ---- J5.7 (RS232_RX)
RXD ---- J5.5 (RS232_TX)
GND ---- J5.4 (GND)
3.3V ---- J5.1 (3V3) (Not compulsory)

2. DURING VISUALIZING DATA FROM MSSLOGGER 

First do configuration, while the configuration takes place and the data is collected in real time - Switch the J5.5 pin to MSSLOGGER

TXD ---- J5.7 (RS232_RX)
RXD ---- J6.9 (MSS_LOGGER)
GND ---- J5.4 (GND)
3.3V ---- J5.1 (3V3) (Not compulsory)

**WARNING**: Once the configuration is sent do not press reset, if reset is pressed cut off the power supply to radar 
and start the configuration transmission process from the start.

