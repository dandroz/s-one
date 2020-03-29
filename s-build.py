#!/usr/bin/env python
# encoding: utf-8
#import PySimpleGUI as sg
import re
import sys
import subprocess
if sys.version_info[0] >= 3:
	import PySimpleGUI as sg
else:
	import PySimpleGUI27 as sg

sg.ChangeLookAndFeel('DarkAmber')

ino_file = ""
h_file = ""

if len(sys.argv) == 2:
	ino_file = sys.argv[1]
	try:
		f = open(ino_file, "r")
	except:
		print("file:", ino_file, "not found")
		sg.Popup("file:", ino_file, "not found")
		ino_file = ""
		
	if ino_file.split("/")[-1].split(".")[-1] != "ino":
		print("input file must be Arduino sketch *.ino file")
		sg.Popup("input file must be Arduino sketch *.ino file")
		ino_file = ""
		
fqbns = {"WEMOS" :
		"esp8266com:esp8266:d1_mini:xtal=80,vt=flash,exception=disabled,ssl=all,eesz=4M,ip=lm2f,dbg=Disabled,lvl=None____,wipe=none,baud=921600",
		 "ESP01S" : "esp8266com:esp8266:generic:xtal=80,vt=flash,exception=disabled,ssl=all,ResetMethod=ck,CrystalFreq=26,FlashFreq=40,FlashMode=dout,eesz=1M,led=2,sdk=nonosdk221,ip=lm2f,dbg=Disabled,lvl=None____,wipe=none,baud=115200"}
	
compil_opt_h_head = """
#ifndef compil_opt_h
#define compil_opt_h

//***************** Compilation config section *******************************************************
/** Select the board by uncommenting the right one.
Define only one board! */   

"""
compil_opt_h_foot = """
//****************************************************************************************************

#endif
// Created automaticaly by S-ONE BUILDER (s-bulid.py)

"""

def disableContent(istrue):
	for key in values:
		window.FindElement(key).Update(disabled=istrue)
	window.FindElement('compile').Update(disabled=istrue)
	window.FindElement('upload').Update(disabled=istrue)
	window.FindElement('cancel').Update(disabled=istrue)
   
layout = [      
	[sg.Text('Select sketch:', size=(15, 1)), sg.InputText(ino_file, key='file'), sg.FileBrowse()],      
	[sg.Text('Choose board:       '), sg.Radio('WEMOS', "RADIO1", default=True, key='wemos', enable_events=True), sg.Radio('ESP01S', "RADIO1", key='esp01s', enable_events=True)],
	[sg.Text('Select options:')],     
	[sg.Checkbox('INPUTS', key='inputs'), sg.Checkbox('TELNET', default=True, key='telnet'),sg.Checkbox('DHT_THERMO', key='dht_thermo', enable_events=True), sg.Checkbox('DS_THERMO', key='ds_thermo', enable_events=True)],     
	[sg.Checkbox('RF_THERMO', key='rf_thermo', enable_events=True ), sg.Checkbox('RF_BUTTON', key='rf_button', enable_events=True), sg.Checkbox('IMPULSE_COUNTER', key='impulse_counter', enable_events=True), sg.Checkbox('SSL_CONNECTION', default=True, key='ssl_connection', enable_events=True)],
	[sg.Output(size=(80, 10), key='status')],
	[sg.Submit("Compile", key='compile'), sg.Submit("Upload", key='upload'), sg.Cancel(key='cancel')]]


window = sg.Window('s-one binaries builder', default_element_size=(40, 1)).Layout(layout).Finalize()
window['upload'].set_tooltip('via serial port') 
window['ssl_connection'].set_tooltip('When you enable ssl connection,\nthe sketch uses a lot of memory.\nThis may cause a crash.')


while True:
	event, values = window.Read()
	
	if event in (None, 'exit'):
		break
	if values['esp01s']:
		window.FindElement('rf_thermo').Update(disabled=True)
		window.FindElement('rf_button').Update(disabled=True)
		window.FindElement('impulse_counter').Update(disabled=True)
		if values['ds_thermo']:
			window.FindElement('ds_thermo').Update(disabled=False)
			window.FindElement('dht_thermo').Update(disabled=True)
		elif values['dht_thermo']:
			window.FindElement('ds_thermo').Update(disabled=True)
			window.FindElement('dht_thermo').Update(disabled=False)
		else:
			window.FindElement('ds_thermo').Update(disabled=False)
			window.FindElement('dht_thermo').Update(disabled=False)
	else:
		if values['rf_thermo']:
			window.FindElement('rf_thermo').Update(disabled=False)
			window.FindElement('rf_button').Update(disabled=True)
			window.FindElement('impulse_counter').Update(disabled=True)	
		elif values['rf_button']:
			window.FindElement('rf_thermo').Update(disabled=True)
			window.FindElement('rf_button').Update(disabled=False)
			window.FindElement('impulse_counter').Update(disabled=True)
		elif values['impulse_counter']:
			window.FindElement('rf_thermo').Update(disabled=True)
			window.FindElement('rf_button').Update(disabled=True)
			window.FindElement('impulse_counter').Update(disabled=False)
		else:
			window.FindElement('rf_thermo').Update(disabled=False)
			window.FindElement('rf_button').Update(disabled=False)
			window.FindElement('impulse_counter').Update(disabled=False)
	
	if event == 'compile':
		disableContent(True)
		ino_file = values['file']
		if ino_file.split("/")[-1].split(".")[-1] != "ino":
			print("input file must be Arduino sketch *.ino file")
			sg.Popup("input file must be Arduino sketch *.ino file")
			ino_file = ""
			h_file = ""
		else:
			compil_opt_h = compil_opt_h_head
			if values['wemos']:
				fqbn = "WEMOS"
				compil_opt_h += """//#define ESP01S
#define WEMOS
"""
			else:
				fqbn = "ESP01S"
				compil_opt_h += """#define ESP01S
//#define WEMOS
"""
			comp_flag = fqbn + "."
			if values['inputs']:
				comp_flag += "INPUTS."
				compil_opt_h += """
/** If you define INPUTS you will have access to the control inputs.
Input types are configurable on the configuration page.*/
#define INPUTS //comment this line to disable physical buttons

"""
			else:
					compil_opt_h += """
/** If you define INPUTS you will have access to the control inputs.
Input types are configurable on the configuration page.*/
//#define INPUTS //comment this line to disable physical buttons

"""
			if values['rf_thermo']:
				comp_flag += "RF_THERMO."
				compil_opt_h += """
/** Define only one RF receiver!
RF receiving is not implemented on ESP-01!
If you define RF_THERMO you will be able to send to the cloud the temperature 
and humidity from three 433 MHz thermometers (each on a different RF channel).*/
#define RF_THERMO //comment this line to disable RF thermometer receiving.
"""
			else:
					compil_opt_h += """
/** Define only one RF receiver!
RF receiving is not implemented on ESP-01!
If you define RF_THERMO you will be able to send to the cloud the temperature 
and humidity from three 433 MHz thermometers (each on a different RF channel).*/
//#define RF_THERMO //comment this line to disable RF thermometer receiving.
"""				
			if values['rf_button']:
				comp_flag += "RF_BUTTON."
				compil_opt_h += """
/** If you define the RF_BUTTON, you will be able to use the 433 MHz RF remote control 
as a monostable button (each press of the button toggle the relay).*/
#define RF_BUTTON //comment this line to disable RF remote control

"""
			else:
				compil_opt_h += """
/** If you define the RF_BUTTON, you will be able to use the 433 MHz RF remote control 
as a monostable button (each press of the button toggle the relay).*/
//#define RF_BUTTON //comment this line to disable RF remote control

"""
			if values['telnet']:
				comp_flag += "TELNET."
				compil_opt_h += """
/** If you define TELNET you will be able to connect to the device via telnet protocol.*/
#define TELNET //comment this line to disable telnet
"""
			else:
				compil_opt_h += """
/** If you define TELNET you will be able to connect to the device via telnet protocol.*/
//#define TELNET //comment this line to disable telnet
"""
			if values['ds_thermo']:
				comp_flag += "DS_THERMO."
				compil_opt_h += """
#define DS_THERMO //comment this line to disable DS18B20 sensor
"""
			else:
				compil_opt_h += """
//#define DS_THERMO //comment this line to disable DS18B20 sensor
"""			
			if values['dht_thermo']:
				comp_flag += "DHT_THERMO."
				compil_opt_h += """
#define DHT_THERMO //comment this line to disable DHT sensor
"""
			else:
				compil_opt_h += """
//#define DHT_THERMO //comment this line to disable DHT sensor
"""			
			if values['impulse_counter']:
				comp_flag += "IMPULSE_COUNTER."
				compil_opt_h += """
#define IMPULSE_COUNTER //NOT IMPLEMENTED YET
"""
			else:
				compil_opt_h += """
//#define IMPULSE_COUNTER //NOT IMPLEMENTED YET
"""
			if values['ssl_connection']:
				comp_flag += "SSL."
				compil_opt_h += """
/**When you enable ssl connection, the sketch uses a lot of memory. 
This may cause a crash.*/
#define SSL_CONNECTION
"""
			else:
				compil_opt_h += """
/**When you enable ssl connection, the sketch uses a lot of memory. 
This may cause a crash.*/
//#define SSL_CONNECTION
"""
			
			compil_opt_h += compil_opt_h_foot
			h_file = re.sub(ino_file.split("/")[-1], "compil_opt.h", ino_file)
			hf = open(h_file, "w")
			hf.write(compil_opt_h)
			hf.close()
			bin_file = ino_file + "." + comp_flag + "bin"
			cmd = "~/bin/arduino-cli compile -b " + fqbns[fqbn] + " " + ino_file + " -o " + bin_file 
			#print("prepared files:", ino_file, h_file, "compilation options:", comp_flag)
			try:
				print(cmd)
				print('Wait for finish compiling...')
				window.refresh()
				res_ar = subprocess.call(cmd, shell=True)
				if res_ar > 0:
					print('**** ERROR ****\n')
					sg.popup_error('Something went wrong')
					disableContent(False)
				else:
					print('**** DONE ****\n')
					disableContent(False)
			except:
				print('**** ERROR ****\n')
				sg.popup_error('Something went wrong')
				disableContent(False)
			
	elif event == 'upload':
		disableContent(True)
		ino_file = values['file']
		if ino_file.split("/")[-1].split(".")[-1] != "ino":
			print("input file must be Arduino sketch *.ino file")
			sg.Popup("input file must be Arduino sketch *.ino file")
			ino_file = ""
			h_file = ""
		else:
			compil_opt_h = compil_opt_h_head
			if values['wemos']:
				fqbn = "WEMOS"
				compil_opt_h += """//#define ESP01S
#define WEMOS
"""
			else:
				fqbn = "ESP01S"
				compil_opt_h += """#define ESP01S
//#define WEMOS
"""
			comp_flag = fqbn + "."
			if values['inputs']:
				comp_flag += "INPUTS."
				compil_opt_h += """
/** If you define INPUTS you will have access to the control inputs.
Input types are configurable on the configuration page.*/
#define INPUTS //comment this line to disable physical buttons

"""
			else:
					compil_opt_h += """
/** If you define INPUTS you will have access to the control inputs.
Input types are configurable on the configuration page.*/
//#define INPUTS //comment this line to disable physical buttons

"""
			if values['rf_thermo']:
				comp_flag += "RF_THERMO."
				compil_opt_h += """
/** Define only one RF receiver!
RF receiving is not implemented on ESP-01!
If you define Rnajwiększa znana liczba pierwszaF_THERMO you will be able to send to the cloud the temperature 
and humidity from three 433 MHz thermometers (each on a different RF channel).*/
#define RF_THERMO //comment this line to disable RF thermometer receiving.
"""
			else:
					compil_opt_h += """
/** Define only one RF receiver!
RF receiving is not implemented on ESP-01!
If you define RF_THERMO you will be able to send to the cloud the temperature 
and humidity from three 433 MHz thermometers (each on a different RF channel).*/
//#define RF_THERMO //comment this line to disable RF thermometer receiving.
"""				
			if values['rf_button']:
				comp_flag += "RF_BUTTON."
				compil_opt_h += """
/** If you define the RF_BUTTON, you will be able to use the 433 MHz RF remote control 
as a monostable button (each press of the button toggle the relay).*/
#define RF_BUTTON //comment this line to disable RF remote control

"""
			else:
				compil_opt_h += """
/** If you define the RF_BUTTON, you will be able to use the 433 MHz RF remote control 
as a monostable button (each press of the button toggle the relay).*/
//#define RF_BUTTON //comment this line to disable RF remote control

"""
			if values['telnet']:
				comp_flag += "TELNET."
				compil_opt_h += """
/** If you define TELNET you will be able to connect to the device via telnet protocol.*/
#define TELNET //comment this line to disable telnet
"""
			else:
				compil_opt_h += """
/** If you define TELNET you will be able to connect to the device via telnet protocol.*/
//#define TELNET //comment this line to disable telnet
"""
			if values['ds_thermo']:
				comp_flag += "DS_THERMO."
				compil_opt_h += """
#define DS_THERMO //comment this line to disable DS18B20 sensor
"""
			else:
				compil_opt_h += """
//#define DS_THERMO //comment this line to disable DS18B20 sensor
"""			
			if values['dht_thermo']:
				comp_flag += "DHT_THERMO."
				compil_opt_h += """
#define DHT_THERMO //comment this line to disable DHT sensor
"""
			else:
				compil_opt_h += """
//#define DHT_THERMO //comment this line to disable DHT sensor
"""			
			if values['impulse_counter']:
				comp_flag += "IMPULSE_COUNTER."
				compil_opt_h += """
#define IMPULSE_COUNTER //NOT IMPLEMENTED YET
"""
			else:
				compil_opt_h += """
//#define IMPULSE_COUNTER //NOT IMPLEMENTED YET
"""
			if values['ssl_connection']:
				comp_flag += "SSL."
				compil_opt_h += """
/**When you enable ssl connection, the sketch uses a lot of memory. 
This may cause a crash.*/
#define SSL_CONNECTION
"""
			else:
				compil_opt_h += """
/**When you enable ssl connection, the sketch uses a lot of memory. 
This may cause a crash.*/
//#define SSL_CONNECTION
"""
			compil_opt_h += compil_opt_h_foot
			h_file = re.sub(ino_file.split("/")[-1], "compil_opt.h", ino_file)
			hf = open(h_file, "w")
			hf.write(compil_opt_h)
			hf.close()
			bin_file = ino_file + "." + comp_flag + "bin"
			cmd = "~/bin/arduino-cli compile" + " -u -p /dev/ttyUSB0 -b " + fqbns[fqbn] + " " + ino_file + " -o " + bin_file 
			try:
				print(cmd)
				print('Wait for finish compiling and uploading...')
				window.refresh()
				res_ar = subprocess.call(cmd, shell=True)
				if res_ar > 0:
					print('**** ERROR ****\n')
					sg.popup_error('Something went wrong')
					disableContent(False)
				else:
					print('**** DONE ****\n')
					disableContent(False)
			except:
				print('**** ERROR ****\n')
				sg.popup_error('Something went wrong')
				disableContent(False)
				
	elif event == 'cancel':
		# sg.Popup('Title',
         # 'The results of the window.',
         # 'The button clicked was "{}"'.format(event),
         # 'The values are', values)
		window.close()
		sys.exit(0)
	else:
		pass
	
	
