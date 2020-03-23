
#ifndef compil_opt_h
#define compil_opt_h

//***************** Compilation config section *******************************************************
/** Select the board by uncommenting the right one.
Define only one board! */   

//#define ESP01S
#define WEMOS

/** If you define INPUTS you will have access to the control inputs.
Input types are configurable on the configuration page.*/
#define INPUTS //comment this line to disable physical buttons


/** Define only one RF receiver!
RF receiving is not implemented on ESP-01!
If you define Rnajwiększa znana liczba pierwszaF_THERMO you will be able to send to the cloud the temperature 
and humidity from three 433 MHz thermometers (each on a different RF channel).*/
#define RF_THERMO //comment this line to disable RF thermometer receiving.

/** If you define the RF_BUTTON, you will be able to use the 433 MHz RF remote control 
as a monostable button (each press of the button toggle the relay).*/
//#define RF_BUTTON //comment this line to disable RF remote control


/** If you define TELNET you will be able to connect to the device via telnet protocol.*/
#define TELNET //comment this line to disable telnet

//#define DS_THERMO //comment this line to disable DS18B20 sensor

//#define DHT_THERMO //comment this line to disable DHT sensor

//#define IMPULSE_COUNTER //NOT IMPLEMENTED YET

//****************************************************************************************************

#endif
// Created automaticaly by S-ONE BUILDER (s-bulid.py)

