# s-one
Arduino sketch for Supla home automation.

The sketch is rewritten to the library in version 2.3.1 (originally 1.6.1) and still being developed. 
Tested on Wemos D1 mini and ESP-01S with relay board v.4.0.

The most important features:
* Configuration in AP mode with the server at http://194.168.4.1 and save configuration in EEPROM.
* Possibility to update/upload the .bin file via the website at http://194.168.4.1/update
* Possibility to download/upload the configuration.
* One  relay channel  with two (on ESP-01S with one) configurable inputs:
  - first input: NO sensor, touch sensor, motion sensor (HC-SR501 tested) can work only at night - based on 
  time synchronized by NTP the program calculates summer/winter time and sunrise/sunset.
  - second input (PULL UP): NO sensor, monostable button, bistable switch
  On Wemos you can use both. 
* 433MHz remote control (rc-switch library).
* 433MHz temperature and humidity signal received from three external weather station sensors (RFControl library).
* Possibility to save the last relay state in EEPROM.
* TELNET - viewing the logs, entering to configuration mode, resetting via e.g. PUTTY (telnetspy library).
  I don't know how safe it is, so you can use only the serial port.
* DHT and/or DS18B20 temperature measurement.

I use the RXB14 module to receive the 433MHz signal, but others will probably work too.
Not all functionalities are available at the same time. Before compilation it is necessary to comment/uncomment 
the correct macros in "compil_opt.h" or use script "s-build.py".

The sources of libraries needed for compilation should be downloaded from github or installed in the libraries manager.
Sketch intended mainly for fun :D, but I hope it can be useful to controlling lighting.
