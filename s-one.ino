/*
  s-one - Arduino IDE sketch for ESP8266

  Based on SuplaDevice library https://github.com/SUPLA/arduino Copyright (C)
  AC SOFTWARE SP. Z O.O.
 
  +---------------------------------------------------------------------------------+
  |Thanks to colleagues from the forum.supla.org in particular: @pzygmunt and @klew |
  |without whom this program would probably not have been created.                  |
  +---------------------------------------------------------------------------------+

  Tested with ESP8266 core ver. 2.6.3 on Wemos D1 mini V3.0.0 and ESP-01S with
  relay board v.4.0

  For some time written in Geany (https://www.geany.org/) and compiled
  by arduino-cli (https://arduino.github.io/arduino-cli/installation/).

  Author: @daniel

  ** Use at your own risks! **

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
*/

/*******************************************************
 ** PLEASE EDIT compil_opt.h FILE BEFORE COMPILATION! **
 **           or use script s-build.py                **
 *******************************************************/


/**
  WEMOS D1 mini pinout
  Pin Function                     ESP-8266 Pin
  TX  TXD                          TXD GPIO1
  RX  RXD                          RXD GPIO3
  A0  Analog input                 A0
  D0  IO, Wake                     GPIO16
  D1  IO, SCL                      GPIO5
  D2  IO, SDA                      GPIO4
  D3  IO, 10k Pull-up              GPIO0
  D4  IO, 10k Pull-up BUILTIN_LED  GPIO2
  D5  IO, SCK                      GPIO14
  D6  IO, MISO                     GPIO12
  D7  IO, MOSI                     GPIO13
  D8  IO,10k Pull-down, SS         GPIO15
  G   Ground                       GND
  5V  5V                           -
  3V3 3.3V                         3.3V
  RST Reset                        RST
*/

//#define SUPLA_COMM_DEBUG

#define ver "1.58"

#include "compil_opt.h"

#if defined(ESP01S) && defined(WEMOS) //
#error "Please define only one board."
#endif

#if defined(RF_BUTTON) && defined(RF_THERMO) //only one RF instance is allowed
#error "Only one RF instance is allowed. Please define only one of them."
#endif

#if defined(ESP01S) && (defined(RF_BUTTON) || defined(RF_THERMO))
#error "RF is not implemented yet on ESP-01!"
#endif

#if defined(ESP01S) && (defined(DHT_THERMO) && defined(DS_THERMO)) //only one type of themperature  sensor is allowed
#error "On ESP-01 only one type of temperature sensor is allowed on.\n\rPlease define only one of them."
#endif

#if (defined(RF_BUTTON) || defined(RF_THERMO)) && defined(IMPULSE_COUNTER)
#error "RF and IMPULSE_COUNTER cannot be used at the same time.\n\rPlease define only one of them."
#endif

#include <SuplaDevice.h> //https://github.com/klew/arduino/tree/master (GPL)
#include <supla/network/esp_wifi.h>
#include <supla/sensor/normally_open.h>
#include <supla/control/relay.h>
#ifdef INPUTS
#include <supla/control/button.h>
#endif

#include <ESP8266WiFi.h> // https://github.com/esp8266/Arduino (GPL)
#include <WiFiUdp.h>
#include <EEPROM.h>
#include <SPI.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPUpdateServer.h>

#include <ESP8266TrueRandom.h> // https://github.com/marvinroger/ESP8266TrueRandom (GPL)

#include <DoubleResetDetector.h> // https://github.com/datacute/DoubleResetDetector (MIT)

#include <SunSet.h> //https://github.com/buelowp/sunset (GPL)

#include <Time.h> // https://github.com/PaulStoffregen/Time (GPL)

#ifdef TELNET
#include <TelnetSpy.h>  // modified: https://github.com/dandroz/telnetspy
                        // original: https://github.com/yasheena/telnetspy (MIT)
#endif

#ifdef RF_THERMO
#include <RFControl.h> // original: https://github.com/pimatic/RFControl (GPL)
                       // modified: https://github.com/dandroz/RFControl
#include <supla/sensor/therm_hygro_meter.h>
#endif

#ifdef DHT_THERMO
#include <supla/sensor/DHT.h>
#endif

#ifdef RF_BUTTON
#include <RCSwitch.h> //https://github.com/sui77/rc-switch (GPL)
RCSwitch mySwitch = RCSwitch();
#include <supla/element.h>
#endif

#ifdef DS_THERMO
#include <supla/sensor/DS18B20.h>
#endif

#define E_EEPROM_SIZE 1024
#define E_EEPROM_OFFSET 10

#define E_RELAY_STATE E_EEPROM_OFFSET
#define E_MODE_ADRESS (E_RELAY_STATE   + 1)
#define E_CONF_STATE (E_MODE_ADRESS + 1)

#define E_LED_ADRESS (E_CONF_STATE + 1)
#define E_LED_MAXSIZE 2
#define E_SAVE_ADRESS (E_LED_ADRESS + E_LED_MAXSIZE)
#define E_SAVE_MAXSIZE 2
#define E_APSSID_ADRESS (E_SAVE_ADRESS + E_SAVE_MAXSIZE)
#define E_APSSID_MAXSIZE 33
#define E_APPASS_ADRESS (E_APSSID_ADRESS + E_APSSID_MAXSIZE)
#define E_APPASS_MAXSIZE 65
#define E_INPUT_1_ADRESS (E_APPASS_ADRESS + E_APPASS_MAXSIZE)
#define E_INPUT_1_MAXSIZE 2
#define E_INPUT_2_ADRESS (E_INPUT_1_ADRESS + E_INPUT_1_MAXSIZE)
#define E_INPUT_2_MAXSIZE 2
#define E_RF_BTN_VAL_ADRESS (E_INPUT_2_ADRESS + E_INPUT_2_MAXSIZE)
#define E_RF_BTN_VAL_MAXSIZE 17
#define E_COORD_ADRESS (E_RF_BTN_VAL_ADRESS + E_RF_BTN_VAL_MAXSIZE)
#define E_COORD_MAXSIZE 66
#define E_PIR_TIMER_ADRESS (E_COORD_ADRESS + E_COORD_MAXSIZE)
#define E_PIR_TIMER_MAXSIZE 10

#define E_NAME_ADRESS (E_PIR_TIMER_ADRESS + E_PIR_TIMER_MAXSIZE)
#define E_NAME_MAXSIZE 202
#define E_SSID_ADRESS (E_NAME_ADRESS + E_NAME_MAXSIZE)
#define E_SSID_MAXSIZE 33
#define E_WPASS_ADRESS (E_SSID_ADRESS + E_SSID_MAXSIZE)
#define E_WPASS_MAXSIZE 65
#define E_SERVER_ADRESS (E_WPASS_ADRESS + E_WPASS_MAXSIZE)
#define E_SERVER_MAXSIZE 66
#define E_EMAIL_ADRESS (E_SERVER_ADRESS + E_SERVER_MAXSIZE)
#define E_EMAIL_MAXSIZE 257

#define E_AUTHK_ADRESS (E_EMAIL_ADRESS + E_EMAIL_MAXSIZE)
#define E_AUTHK_MAXSIZE 82
#define E_GUID_ADRESS (E_AUTHK_ADRESS + E_AUTHK_MAXSIZE)
#define E_GUID_MAXSIZE 82

#define E_EEPROM_USAGE (E_GUID_ADRESS + E_GUID_MAXSIZE - E_EEPROM_OFFSET)


//*********************** Modules configuration section *****************************************
#ifdef ESP01S
#define RELAY_PIN 0            // Relay pin on ESP-01S
#else
#define RELAY_PIN D1           // Relay pin on WEMOS
#endif

#ifdef INPUTS

#ifdef ESP01S
#define INPUT_2_PIN 2         // input pin on ESP-01S
int config_btn_pin = INPUT_2_PIN;     // Pin of a config button. Push it until 10 second.
#else
#define INPUT_1_PIN D2        // first input pin on WEMOS
#define INPUT_2_PIN D7         // second input pin on WEMOS
int config_btn_pin = INPUT_1_PIN;     // Pin of a config button. Push it until 10 second.
bool pir_on_day = false;              // if true, PIR continue detecting in the day
#endif
#endif //INPUTS

#define led_pin 2                  // LED pin

#ifdef DS_THERMO
#ifdef ESP01S
#define DS_PIN 3                  // DS_THERMO pin on ESP01
#else
#define DS_PIN D5                 // DS_THERMO pin on WEMOS
#endif
#endif

#if defined(RF_THERMO) || defined(RF_BUTTON)
#define RF_PIN D6                 // RF receiver pin
#endif

#ifdef DHT_THERMO
#define DHTPIN 3                  // DHT pin
#define DHTTYPE DHT11             // DHT type
#endif

String name_prefix = "s-one_";
String str_apssid = name_prefix + String(ESP.getChipId());  // Defalut AP name
String str_appass = "supla2018";  // Defalut AP password
int pir_timer = 120;   // Defalut PIR sensor timer (seconds)
float coord[3] = {52.2297, 21.01178, 1}; // Defalut geographical coordinates (Warsaw), time zone offset
static const char ntpServerName[] = "tempus1.gum.gov.pl";    // NTP servver
#define DRD_TIMEOUT 5 // Number of seconds after reset during which a subseqent reset will be considered a double reset.
//***********************************************************************************************

#ifdef RF_THERMO
#define RF_RECEIVER_PIN      D6
#define RF_ZERO_PULSE        1700
#define RF_ONE_PULSE         3800
#define RF_AVG_PULSE         (RF_ZERO_PULSE + RF_ONE_PULSE) / 2
#define RF_RAW_LENGTH        92
int rf_bin[RF_RAW_LENGTH / 2];
unsigned int rf_cb_last_time = 0;

typedef struct {
  int channel;
  float temp;
  float humi;
  unsigned long last_time;
} _rf_data_t;
_rf_data_t rf_data[3];
#endif

#define DRD_ADDRESS 0
const String SAVE_RELAY_STATE_MODES[3] = {"ALWAYS_OFF", "ALWAYS_ON", "LAST_STATE"};

#ifdef TELNET
TelnetSpy SerialAndTelnet;
#define serial SerialAndTelnet
#else
#define serial Serial
#endif

uint8_t mac[6];
char GUID[SUPLA_GUID_SIZE];
char AUTH_KEY[SUPLA_GUID_SIZE];
int save_state;
char supla_name[E_NAME_MAXSIZE];
char ssid[E_SSID_MAXSIZE];
char password[E_APPASS_MAXSIZE];
char serv[E_SERVER_MAXSIZE];
char apssid[E_APSSID_MAXSIZE];
char appassword[E_APPASS_MAXSIZE];
char email[E_EMAIL_MAXSIZE];
String str_guid;
String str_auth_key;
int relay_channel;
int led;
bool server_started = false;
int last_led_time = 0;
bool normal_mode = true;
unsigned long last_connect_time = 0;
bool configured = false;
unsigned int start_time;
unsigned int start_online_time;
bool NTP_updated = false;
bool set_start_time = false;
unsigned int config_mode_start_time;
bool config_server_connected = false;
int relay_last_state;
unsigned long relay_last_time;
int last_status;

#ifdef INPUTS
int input_1_last_state;
unsigned long input_1_last_time;
unsigned long pir_start_time;
int input_2_mode;
int input_1_mode;
unsigned long config_btn_time = 0;
bool on_by_pir = false;
#define MODE_PULLUP_SENSOR_NO 0
#define MODE_PULLUP_MONOSTABLE 1
#define MODE_TOUCH_SENSOR 2
#define MODE_PULLUP_BISTABLE 3
#define MODE_PIR_SENSOR 4
#define MODE_NIGHT_PIR_SENSOR 5
const String INPUTS_MODES[6] = {"MODE_PULLUP_SENSOR_NO", "MODE_PULLUP_MONOSTABLE", "MODE_TOUCH_SENSOR", "MODE_PULLUP_BISTABLE", "MODE_PIR_SENSOR", "MODE_NIGHT_PIR_SENSOR" };
Supla::Control::Button *input_1 = nullptr;
Supla::Control::Button *input_2 = nullptr;
#else
bool on_by_pir = false;
#endif

#ifdef RF_BUTTON
int rf_btn_value = 0;
bool rf_switch = false;
unsigned int rf_switch_time;
bool rf_new_value = false;
bool get_rf_button = false;
#endif

unsigned int localPort = 8888;  // local port to listen for UDP packets
ADC_MODE(ADC_VCC);
WiFiUDP ntpUdp;
DoubleResetDetector drd(DRD_TIMEOUT, DRD_ADDRESS);
Supla::ESPWifi *wifi = nullptr;
Supla::Control::Relay *relay = nullptr;
SunSet *sun = nullptr;
ESP8266WebServer *server = nullptr;
ESP8266HTTPUpdateServer *httpUpdater = nullptr;

String getCompilOptions() {
  String comp_opt = "";
#ifdef ESP01S
  comp_opt += "ESP01S.";
#endif
#ifdef WEMOS
  comp_opt += "WEMOS.";
#endif
#ifdef INPUTS
  comp_opt += "INPUTS.";
#endif
#ifdef RF_THERMO
  comp_opt += "RF_THERMO.";
#endif
#ifdef RF_BUTTON
  comp_opt += "RF_BUTTON.";
#endif
#ifdef TELNET
  comp_opt += "TELNET.";
#endif
#ifdef DS_THERMO
  comp_opt += "DS_THERMO.";
#endif
#ifdef DHT_THERMO
  comp_opt += "DHT_THERMO.";
#endif
#ifdef IMPULSE_COUNTER
  comp_opt += "IMPULSE_COUNTER.";
#endif

  return comp_opt;
}

#ifdef RF_THERMO
class RFthermHygro: public Supla::Sensor::ThermHygroMeter {
  public:
    RFthermHygro(int index) {
      myIndex = index;
    }

    void onInit() {
      RFControl::startReceiving(RF_RECEIVER_PIN);
      //channel.setNewValue(getTemp(), getHumi());
    }

    double getTemp() {
      if ((millis() > rf_data[myIndex].last_time + 120000) && rf_data[myIndex].temp != -275 ) {
        return -275;
      }
      else {
        return rf_data[myIndex].temp; 
      }
    }
    
     void onTimer() {
      rfThermHygroReceive();
    }

    double getHumi() {
      if ((millis() > rf_data[myIndex].last_time + 120000) && rf_data[myIndex].humi != -1 ) {
        return -1;
      }
      else {
        return rf_data[myIndex].humi;
      }
    }
    
    void iterateAlways() {
     if (lastReadTime + 10000 < millis()) {
        lastReadTime = millis();
        channel.setNewValue(getTemp(), getHumi());
      }
    }
    
    int myIndex;
};
#endif

class NTPtimeUpdater : public Supla::Element {
  public:
  NTPtimeUpdater() {}
  
  void onInit() {
    ntpUdp.begin(localPort);
  }
  
  void onTimer() {
  }
  
  void iterateAlways () {
    now();
    if (!NTP_updated && wifi->isReady()) {
      setSyncProvider(getNtpTime);
      setSyncInterval(21600);
      serial.print("NTP update status: ");
      if (timeStatus() == timeSet) {
        NTP_updated = true;
        set_start_time = true;
        serial.print("updated --> ");
        timeStamp();
      }
      else {
        serial.println("not updated");
      }
    }
    if (set_start_time && NTP_updated) {
      set_start_time = false;
      start_time = now();
    }
  }
};

#ifdef RF_BUTTON
class RFbutton : public Supla::Element {
  public:
    RFbutton() {}

    void iterateAlways() {
      if (rf_switch) {
        rf_switch_time = millis();
        rf_switch = false;
        timeStamp();
        relay->toggle();
        serial.println("relay state changed by rf remote control");
      }
    }
    void onInit() {}
};
#endif

class SaveRelayState : public Supla::Element {
  public:
    SaveRelayState() {}

    void iterateAlways() {
      now = millis();
      relay_pin_value = digitalRead(RELAY_PIN);
      if (!on_by_pir && (relay_pin_value != relay_last_state) && (now > relay_last_time + 1000)) {
        EEPROM.write(E_RELAY_STATE, relay_pin_value);
        timeStamp();
        serial.println("relay state: " + String(relay_pin_value) + " saved in EEPROM");
        relay_last_state = relay_pin_value;
        relay_last_time = now;
        on_by_pir = false;
        EEPROM.commit();
      }
    }

    void onInit() {}

    unsigned long now;
    int relay_pin_value;

};

class StatusLed : public Supla::Element {
  public:
    StatusLed() {}

    void iterateAlways() {
      cloud_connected = wifi->connected();
      now = millis();
      if (cloud_connected) {
        if (led == 1 && (now > last_led_time + 20)) {
          analogWrite(led_pin, fade);
          if (fade >= 500) add = -1;
          if (fade <= 0) add = 1;
          fade = fade + add;
          last_led_time = now;
        }
      }
      else {
        if ((led) == 1 && (now > last_led_time + 500)) {
          digitalWrite(led_pin, (digitalRead(led_pin) == HIGH ? LOW : HIGH));
          last_led_time = now;
        }
      }
    }

    void onInit() {
      if (led == 1) {
        analogWriteRange(500);
        pinMode(led_pin, OUTPUT);
        add = 1;
        fade = 0;
      }
    }

    bool cloud_connected;
    unsigned long now;
    int add;
    int fade;
};

#ifdef INPUTS
#ifndef ESP01S
class PIRsensor : public Supla::Element {
  public:
    PIRsensor() {}

    void iterateAlways() {
      now = millis();
      relay_pin_value = digitalRead(RELAY_PIN);
      if (input_1_mode >= MODE_PIR_SENSOR) {
        input_1_pin_value = digitalRead(INPUT_1_PIN);
        if ((input_1_pin_value != input_1_last_state) && (now > input_1_last_time)) {
          input_1_last_state = input_1_pin_value;
          input_1_last_time = now;
          if (pirTurnOn()) {
            if (input_1_pin_value == HIGH && relay_pin_value != HIGH) {
              relay->turnOn();
              timeStamp();
              serial.println("relay ON by PIR");
              pir_start_time = now;
              on_by_pir = true;
            }
            if (input_1_pin_value == HIGH && on_by_pir) {
              serial.println("added time to PIR timer");
              pir_start_time = now;
            }
          }
        }
      }
      if (!relay->isOn()) {
        on_by_pir = false;
      }
      if ((now > (pir_start_time + pir_timer * 1000)) && on_by_pir && relay_pin_value == HIGH && (input_1_mode >= MODE_PIR_SENSOR)) {
        timeStamp();
        relay->turnOff();
        serial.println("relay off by PIR timer");
        on_by_pir = false;
      }
    }
    void onInit() {}
    unsigned long now;
    int relay_pin_value;
    int input_1_pin_value;
};
#endif
#endif

void showInfo() {
  String comp_opt = getCompilOptions();
  timeStamp();
  serial.println("Device name: \033[1m" + String(supla_name) +  "\n\r\033[0mSoftware version: \033[1m" + String(ver) + "\033[0m");
  serial.print("Compilation: ");
  serial.println(comp_opt);
  if (normal_mode) {
    unsigned int n = now() - start_time;
    unsigned int d = n / (24 * 3600);
    n = n % (24 * 3600);
    unsigned int h = n / 3600;
    n %= 3600;
    unsigned int m = n / 60;
    serial.println( "up time: " + String(d) + " days, " + String(h) + " hours, " + String(m) + " minutes");
    serial.print("cloud is: ");
    if (wifi->connected()) {
      n = now() - start_online_time;
      d = n / (24 * 3600);
      n = n % (24 * 3600);
      h = n / 3600;
      n %= 3600;
      m = n / 60;
      serial.println("connected");
      serial.println( "online time: " + String(d) + " days, " + String(h) + " hours, " + String(m) + " minutes");
    } 
    else {
      serial.println("disconnected");
    }
  }
  serial.print("local IP: ");
  serial.println(WiFi.localIP());
  serial.print("subnet mask: ");
  serial.println(WiFi.subnetMask());
  serial.print("gateway IP: ");
  serial.println(WiFi.gatewayIP());
  serial.print("host: ");
  serial.println(WiFi.hostname());
  serial.print("RSSI: ");
  serial.print(WiFi.RSSI());
  serial.println(" dBm");
  WiFi.macAddress(mac);
  serial.print("MAC: ");
  serial.print(mac[5], HEX);
  serial.print(":");
  serial.print(mac[4], HEX);
  serial.print(":");
  serial.print(mac[3], HEX);
  serial.print(":");
  serial.print(mac[2], HEX);
  serial.print(":");
  serial.print(mac[1], HEX);
  serial.print(":");
  serial.println(mac[0], HEX);
#ifdef INPUTS
#ifndef ESP01S
  serial.print("input 1 mode: ");
  serial.println(INPUTS_MODES[input_1_mode]);
#endif
  serial.print("input 2 mode: ");
  serial.println(INPUTS_MODES[input_2_mode]);
#endif
  serial.print("relay pin state: ");
  serial.println(digitalRead(RELAY_PIN) ? "HI" : "LOW");
  serial.print("relay save state mode: ");
  serial.println(SAVE_RELAY_STATE_MODES[save_state]);
  serial.print("Vcc: ");
  serial.println((float)ESP.getVcc() / 1000);
  serial.print("ESP core version: ");
  serial.print(esp8266::coreVersionMajor());
  serial.print(".");
  serial.print(esp8266::coreVersionMinor());
  serial.print(".");
  serial.print(esp8266::coreVersionRevision());
  serial.println(esp8266::coreVersionSubRevision());
  uint32_t free;
  uint16_t max;
  uint8_t frag;
  ESP.getHeapStats(&free, &max, &frag);
  serial.printf("Heap stats - free: %5d; max: %5d; frag: %3d%%\n\r", free, max, frag);
  serial.print("chip ID: ");
  serial.println(ESP.getChipId());
  serial.print("cpu freq: ");
  serial.println(ESP.getCpuFreqMHz());
  serial.print("flash chip ID: ");
  serial.println(ESP.getFlashChipId());
  serial.print("flash chip Real Size: ");
  serial.println(ESP.getFlashChipRealSize());
  serial.print("flash chip Size: ");
  serial.println(ESP.getFlashChipSize());
  serial.print("flash chip Speed: ");
  serial.println(ESP.getFlashChipSpeed());
  serial.print("flash chip Mode: ");
  serial.println(ESP.getFlashChipMode());
  serial.print("sketch size: ");
  serial.println(ESP.getSketchSize());
  serial.print("free sketch space: ");
  serial.println(ESP.getFreeSketchSpace());
}

#if !defined(ESP01S) && defined(INPUTS)
bool pirTurnOn() {
  if (pir_on_day) {
    return true;
  }
  else {
    if (isDark()) {
      return true;
    }
    else {
      return false;
    }
  }
}
#endif

bool isSummertime(int y, int m, int d, int h) {
  if (m < 3 || m > 10) {
    return false;
  }
  if (m > 3 && m < 10) {
    return true;
  }
  int last_sun_mar =  (31 - (5 * y / 4 + 4) % 7);
  int last_sun_oct = (31 - (5 * y / 4 + 1) % 7);
  if (m == 3) {
    if ( d > last_sun_mar)
      return true;
    if ( d < last_sun_mar)
      return false;
    if (h < 1)
      return false;
    if (h >= 1)
      return true;
  }
  if (m == 10) {
    if ( d < last_sun_oct)
      return true;
    if ( d > last_sun_oct)
      return false;
    if (h >= 1)
      return false;
    if (h < 1)
      return true;
  }
}

void timeStamp() {
  if (NTP_updated) {
    int summertime_offset;
    int e = now();
    int y = year(e);
    int m = month(e);
    int d = day(e);
    int h = hour(e);
    int mi = minute(e);
    int s = second(e);
    bool summertime = isSummertime(y, m, d, h);
    if (summertime) {
      summertime_offset = 60;
    }
    else {
      summertime_offset = 0;
    }
    serial.println("<" + (d < 10 ? "0" + String(d) : String(d)) + "-" + (m < 10 ? "0"
                   + String(m) : String(m)) + "-" + String(y) + "|" + (h < 10 ? "0"
                       + String(h) : String(h)) + ":" + (mi < 10 ? "0" + String(mi) : String(mi))
                   + ":" + (s < 10 ? "0" + String(s) : String(s)) + "(UTC)+" + String(int(coord[2])
                       + (summertime_offset / 60)) + ":00>");
  }
}

#ifndef ESP01S
bool isDark() {
  int summertime_offset;
  int e = now();
  int y = year(e);
  int m = month(e);
  int d = day(e);
  int h = hour(e);
  int mi = minute(e);
  int s = second(e);
  bool summertime = isSummertime(y, m, d, h);
  if (summertime) {
    summertime_offset = 60;
  }
  else {
    summertime_offset = 0;
  }
  sun->setCurrentDate(y, m, d);
  int min_after_midnight = (h * 60) + (coord[2] * 60) + mi + summertime_offset;
  int sunrise = sun->calcSunrise();
  int sunset = sun->calcSunset();
  if ((min_after_midnight < sunrise) || (min_after_midnight > sunset)) {
    return true;
  }
  else {
    return false;
  }
}
#endif

String getHex(byte number) {
  int topDigit = number >> 4;
  int bottomDigit = number & 0x0f;
  return String("0123456789ABCDEF"[topDigit]) + String("0123456789ABCDEF"[bottomDigit]);
}

String getUuid(byte * uuidNumber) {
  int i;
  String g;
  g = "{0x";
  for (i = 0; i < 16; i++) {
    g = g + getHex(uuidNumber[i]);
    if (i < 15) g = g + ",0x";
  }
  g = g + "}";
  return g;
}

String readEEPROM(int l, int p) {
  String temp;
  char ch;
  for (int n = p; n < l + p; n++) {
    ch = EEPROM.read(n);
    if (char(ch) != ';') {
      temp += String(char(ch));
      serial.print(".");
    }
    else {
      n = l + p;
    }
  }
  serial.println();
  return temp;
}

void readConfig() {
  serial.println("reading configuration...");
  String str_ssid = readEEPROM(E_SSID_MAXSIZE, E_SSID_ADRESS);
  String str_wpass = readEEPROM(E_WPASS_MAXSIZE, E_WPASS_ADRESS);
  String str_server = readEEPROM(E_SERVER_MAXSIZE, E_SERVER_ADRESS);
  str_auth_key = readEEPROM(E_AUTHK_MAXSIZE, E_AUTHK_ADRESS);
  str_guid = readEEPROM(E_GUID_MAXSIZE, E_GUID_ADRESS);
  String str_name = readEEPROM(E_NAME_MAXSIZE, E_NAME_ADRESS);
  String str_led = readEEPROM(E_LED_MAXSIZE, E_LED_ADRESS);
  String str_save = readEEPROM(E_SAVE_MAXSIZE, E_SAVE_ADRESS);
  String str_apssid = readEEPROM(E_APSSID_MAXSIZE, E_APSSID_ADRESS);
  String str_appass = readEEPROM(E_APPASS_MAXSIZE, E_APPASS_ADRESS);

#ifdef INPUTS
  String str_ch2_btn_mode = readEEPROM(E_INPUT_2_MAXSIZE, E_INPUT_2_ADRESS);
  input_2_mode = str_ch2_btn_mode.toInt();
  String str_ch1_btn_mode = readEEPROM(E_INPUT_1_MAXSIZE, E_INPUT_1_ADRESS);
  input_1_mode = str_ch1_btn_mode.toInt();
  String str_pir_timer = readEEPROM(E_PIR_TIMER_MAXSIZE, E_PIR_TIMER_ADRESS);
  pir_timer = str_pir_timer.toInt();
  String str_coord = readEEPROM(E_COORD_MAXSIZE, E_COORD_ADRESS);
  String tmp[3];
  int a = 0;
  for (int n = 0; n < str_coord.length() - 1; n++) {
    if (String(str_coord[n]) == "{" or String(str_coord[n]) == "}" or String(str_coord[n]) == " " or String(str_coord[n]) == " ") {
    }
    else {
      if (String(str_coord[n]) == ",") {
        a++;
      }
      else {
        tmp[a] = String(tmp[a]) + String(str_coord[n]);
      }
      coord[a] = tmp[a].toFloat();
    }
  }
#endif

#ifdef RF_BUTTON
  String str_rf_btn_value = readEEPROM(E_RF_BTN_VAL_MAXSIZE, E_RF_BTN_VAL_ADRESS);
  rf_btn_value = str_rf_btn_value.toInt();
#endif
  String str_supla_email = readEEPROM(E_EMAIL_MAXSIZE, E_EMAIL_ADRESS);
  str_apssid.toCharArray(apssid, E_APSSID_MAXSIZE);
  str_appass.toCharArray(appassword, E_APPASS_MAXSIZE);
  str_ssid.toCharArray(ssid, E_SSID_MAXSIZE);
  str_wpass.toCharArray(password, E_WPASS_MAXSIZE);
  str_server.toCharArray(serv, E_SERVER_MAXSIZE);
  str_supla_email.toCharArray(email, E_EMAIL_MAXSIZE);
  String guid[SUPLA_GUID_SIZE];
  int c = 0;
  for (int n = 0; n < str_guid.length() - 1; n++) {
    if (String(str_guid[n]) == "{" or String(str_guid[n]) == "}" or String(str_guid[n]) == " " or String(str_guid[n]) == " ") {
    }
    else {
      if (String(str_guid[n]) == ",") {
        c++;
      }
      else {
        guid[c] = String(guid[c]) + String(str_guid[n]);
      }
      GUID[c] = byte(strtoul(guid[c].c_str(), NULL, 16));
    }
  }
  String auth_key[SUPLA_GUID_SIZE];
  int d = 0;
  for (int n = 0; n < str_auth_key.length() - 1; n++) {
    if (String(str_auth_key[n]) == "{" or String(str_auth_key[n]) == "}" or String(str_auth_key[n]) == " " or String(str_auth_key[n]) == " ") {
    }
    else {
      if (String(str_auth_key[n]) == ",") {
        d++;
      }
      else {
        auth_key[d] = String(auth_key[d]) + String(str_auth_key[n]);
      }
      AUTH_KEY[d] = byte(strtoul(auth_key[c].c_str(), NULL, 16));
    }
  }
  str_name.toCharArray(supla_name, E_NAME_MAXSIZE);
  led = str_led.toInt();
  save_state = str_save.toInt();
  serial.println("configuration was read");
}

#ifdef TELNET
void telnetConnected() {
  timeStamp();
  String menu = "";
  menu += "Enter \033[32m\033[1mh\033[0m\ for help.";
  serial.println(menu);
}

void telnetDisconnected() {
  timeStamp();
  serial.println("telnet connection closed");
}
#endif

unsigned int test_last_time;
void test(int index) {
  if (millis() > test_last_time + 250) {
    serial.print(index);
    serial.print(":");
    serial.println(millis() - test_last_time);
    test_last_time = millis();
  }
}


#ifdef RF_THERMO
void rfThermHygroReceive() {
  if (RFControl::hasData()) {
    if (millis() > rf_cb_last_time + 1000) {
        unsigned int *timings;
        unsigned int timings_size;
        unsigned int pulse_length_divider = RFControl::getPulseLengthDivider();
        RFControl::getRaw(&timings, &timings_size);
        int i = 0;
        //serial.println("          1         2         3         4         5");
        //serial.println("012345678901234567890123456789012345678901234567890");
        if (timings_size == RF_RAW_LENGTH) {
          for (int x = 1; x < timings_size; x += 2) {
            unsigned long timing = timings[x] * pulse_length_divider;
            if (timing > RF_AVG_PULSE) {
              rf_bin[i++] = 1;
              //serial.print(1);
            } else {
              rf_bin[i++] = 0;
              //serial.print(0);
            }
          }
          //serial.println();
          int c = rfChannel() - 1;
          /**TODO: calculate the checksum and battery status if exist*/
          serial.print("rf channel ");
          serial.print(c + 1);
          serial.println(" data received");
          rf_data[c].temp = rfTemperature();
          rf_data[c].humi = rfHumidity();
          rf_data[c].channel = rfChannel();
          rf_data[c].last_time = millis();
        }
      }
    RFControl::continueReceiving();
    rf_cb_last_time = millis();
  }
}

void rfDataInit() {
  for (int a = 0; a < 3; a++) {
    rf_data[a].temp = -275;
    rf_data[a].humi = -1;
    rf_data[a].channel = (a + 1) * -1;
    rf_data[a].last_time = millis();
  }
}


int bin2dec(int *binary, int first, int last) {
  int value = 0;
  for (int i = first; i < last; i++)
  {
    value *= 2;
    if (binary[i] == 1) value++;
  }
  return value;
}

int rfHumidity() {
  int b = (bin2dec(rf_bin, 33, 37) * 10) + bin2dec(rf_bin, 37, 41);
  if (b < 1 || b > 100) {
    return -1;
  }
  return b;
}

double rfTemperature() {
double b = (double)bin2dec(rf_bin, 21, 33) / 10;
  b = (b - 122) * (double)0.555555556;
  if (b < -40.0 || b > 70.0) {
    return -275;
  }
  return b;
}

int rfChannel() {
  return bin2dec(rf_bin, 41, 45);
}

int rfId() {
  return bin2dec(rf_bin, 5, 13);
}
#endif

// source https://github.com/PaulStoffregen/Time/blob/f11f6fc1de7f08ce8c3ce9946be0836b037a58b2/examples/TimeNTP_ESP8266WiFi/TimeNTP_ESP8266WiFi.ino#L99
/*-------- NTP code ----------*/
const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

time_t getNtpTime() {
  IPAddress ntpServerIP; // NTP server's ip address
  while (ntpUdp.parsePacket() > 0) ; // discard any previously received packets
  serial.println("Transmit NTP Request");
  // get a random server from the pool
  WiFi.hostByName(ntpServerName, ntpServerIP);
  serial.print(ntpServerName);
  serial.print(": ");
  serial.println(ntpServerIP);
  sendNTPpacket(ntpServerIP);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = ntpUdp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      serial.println("Receive NTP Response");
      ntpUdp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      serial.print("NTP epoch time: ");
      serial.println(secsSince1900 - 2208988800UL);
      return secsSince1900 - 2208988800UL; //+ timeZone * SECS_PER_HOUR;
    }
  }
  serial.println("No NTP Response :-(");
  return 0; // return 0 if unable to get the time
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address) {
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  ntpUdp.beginPacket(address, 123); //NTP requests are to port 123
  ntpUdp.write(packetBuffer, NTP_PACKET_SIZE);
  ntpUdp.endPacket();
}

void suplaDeviceStatus(int status, const char *msg) {
  if (last_status != status) {
    timeStamp();
    serial.print("SuplaDevice status: [");
    serial.print(status);
    serial.print("] ");
    if (status == 17){
      if (NTP_updated) {
        start_online_time = now();
      }
      serial.print("\033[32m");
    }
    if (status == 9  || status == 21) {
      serial.print("\033[31m");
    }
    serial.println(msg);
    serial.print("\033[0m");
    uint32_t free;
    uint16_t max;
    uint8_t frag;
    ESP.getHeapStats(&free, &max, &frag);
    serial.printf("heap stats - free: %5d; max: %5d; frag: %3d%%\n\r", free, max, frag);
    last_status = status;
  }
}

void setup() {
  int set;
  serial.begin(9600);
  serial.setDebugOutput(false);

#ifdef TELNET
  String logo = "";
  logo += "\n\r\033[32m\033[1m";
  logo += "Based on SuplaDevice library Copyright (C) AC SOFTWARE SP. Z O.O.";
  logo += "\033[0m\n\r";
  int len = logo.length() + 1;
  char logo_ch[len];
  logo.toCharArray(logo_ch, len);
  SerialAndTelnet.setWelcomeMsg(logo_ch);
  SerialAndTelnet.setCallbackOnConnect(telnetConnected);
  SerialAndTelnet.setCallbackOnDisconnect(telnetDisconnected);
#endif

  serial.println();
  serial.println("begin setup...");
  serial.println("begin EEPROM...");
  EEPROM.begin(E_EEPROM_SIZE);
  serial.print("EEPROM size: ");
  serial.println(E_EEPROM_SIZE);
  serial.print("EEPROM offset: ");
  serial.println(E_EEPROM_OFFSET);
  serial.print("EEPROM usage: ");
  serial.println(E_EEPROM_USAGE);

#ifdef RF_BUTTON
  mySwitch.enableReceive(RF_PIN);
  serial.println("setup RF remote...");
#endif

  if (drd.detectDoubleReset()) {
    serial.println("double Reset Detected");
    serial.println("go to configuration mode...");
    normal_mode = false;
  }
  else {
    if (int(EEPROM.read(E_MODE_ADRESS)) != 1) {
      serial.println("running in configuration mode...");
      normal_mode = false;
    }
  }
  if (EEPROM.read(E_CONF_STATE) == 1) {
    configured = true;
    serial.println("device is configured...");
    readConfig();
  }
  else {
    serial.println("device is not configured");
    configured = false;
    normal_mode = false;
  }

  if (normal_mode) {
    WiFi.persistent(false);
    WiFi.softAPdisconnect(true);
    wifi = new Supla::ESPWifi(ssid, password);
    serial.println("wifi inicialized...");

#ifdef RF_THERMO
    serial.println("setup RF thermometers...");
    memset(rf_data, 0, sizeof(rf_data));
    rfDataInit();
    new RFthermHygro(0);
    new RFthermHygro(1);
    new RFthermHygro(2);
#endif

#ifdef ESP01S
    bool hi_turn_on = false;
#else
    bool hi_turn_on = true;
#endif

#ifdef INPUTS
#ifndef ESP01S
    input_1 = new Supla::Control::Button(INPUT_1_PIN);
    serial.print("added input 1 on GPIO");
    serial.println(INPUT_1_PIN);
#endif
    input_2 = new Supla::Control::Button(INPUT_2_PIN, true, true);
    serial.print("added input 2 on GPIO");
    serial.println(INPUT_2_PIN);
#endif

#ifdef RF_BUTTON
    new RFbutton();
    serial.println("start RF button receiving...");
#endif

    relay = new Supla::Control::Relay(RELAY_PIN, hi_turn_on);
    serial.print("added relay on GPIO");
    serial.println(RELAY_PIN);

#ifdef INPUTS
#ifndef ESP01S
    if (input_1_mode > 0) {
      switch (input_1_mode) {
        case MODE_TOUCH_SENSOR: {
            input_1->willTrigger(*relay, Supla::Control::Button::ON_PRESS, Supla::Control::Relay::TOGGLE);
            serial.println("input 1: set to touch sensor mode");
            break;
          }
        case MODE_PIR_SENSOR: {
            serial.println("input 1: set to PIR mode");
            pir_on_day = true;
            new PIRsensor();
            break;
          }
        case MODE_NIGHT_PIR_SENSOR: {
            serial.println("setup geographical coordinates...");
            sun = new SunSet(coord[0], coord[1], coord[2]);
            serial.println("input 1: set to night PIR mode");
            new PIRsensor();
            break;
          }
      }
    }
    else {
      new Supla::Sensor::NormallyOpen(INPUT_1_PIN);
      serial.print("added SENSOR_NO on GPIO");
      serial.println(INPUT_1_PIN);
    }
#endif

    if (input_2_mode > 0) {
      switch (input_2_mode) {
        case MODE_PULLUP_MONOSTABLE: {
            input_2->willTrigger(*relay, Supla::Control::Button::ON_PRESS, Supla::Control::Relay::TOGGLE);
            serial.println("input 2: set to pull up monostable mode");
            break;
          }
        case MODE_PULLUP_BISTABLE: {
            input_2->willTrigger(*relay, Supla::Control::Button::ON_CHANGE, Supla::Control::Relay::TOGGLE);
            serial.println("input 2: set to pull up bistable mode");
            break;
          }
      }
    }
    else {
      new Supla::Sensor::NormallyOpen(INPUT_2_PIN);
      serial.print("added sensor NO on GPIO");
      serial.println(INPUT_2_PIN, true);

    }
#endif

#ifdef DHT_THERMO
    new Supla::Sensor::DHT(DHTPIN, DHTTYPE);
    serial.print("added DHT thermometer on GPIO");
    serial.println(DHTPIN);
#endif

#ifdef DS_THERMO
    new Supla::Sensor::DS18B20(DS_PIN);
    serial.print("added DS18B20 thermometer on GPIO");
    serial.println(DS_PIN);
#endif

    if (led == 1) {
      serial.println("setup status LED");
      new StatusLed();
    }

    if (save_state > 1) {
      serial.println("setup saving relay state in EEPROM");
      new SaveRelayState();
    }
    new NTPtimeUpdater();
    SuplaDevice.setName(supla_name);
    SuplaDevice.setStatusFuncImpl(&suplaDeviceStatus);
    SuplaDevice.begin(GUID,              // Global Unique Identifier
                      serv,              // SUPLA server address
                      email,             // Email address
                      AUTH_KEY);         // Authentication key

    WiFi.hostname(apssid);

    if (save_state > 1) {
      set = int(EEPROM.read(E_RELAY_STATE));
      (set == 1) && hi_turn_on ? relay->turnOn() : relay->turnOff();
      serial.println("restore relay state to: " + String(set == 1 ? 1 : 0));
    }
    else {
      (save_state == 1) && hi_turn_on ? relay->turnOn() : relay->turnOff();
    }
    relay_last_state = digitalRead(RELAY_PIN);
    relay_last_time = millis();
    drd.stop();
    serial.println("setup ended");
  }
  else {
    drd.stop();
    configMode();
  }
}

void configMode() {
  serial.println("run device in configuration mode");
  server = new ESP8266WebServer(80);
  httpUpdater = new ESP8266HTTPUpdateServer;
  IPAddress ap_local_IP(192, 168, 4, 1);
  IPAddress ap_gateway(192, 168, 4, 254);
  IPAddress ap_subnet(255, 255, 255, 0);
  if (configured) {
    config_mode_start_time = millis();
  }
  else {
    getRandomGuidAndAuthkey();
    str_apssid.toCharArray(apssid, E_APSSID_MAXSIZE);
    str_appass.toCharArray(appassword, E_APPASS_MAXSIZE);
  }
  serial.println(apssid);
  serial.println(appassword);
  WiFi.disconnect(true);
  pinMode(led_pin, OUTPUT);
  serial.print("setting AP config: ");
  serial.println(WiFi.softAPConfig(ap_local_IP, ap_gateway , ap_subnet) ? "Ready" : "Failed!");
  serial.print("setting up AP: ");
  serial.println(WiFi.softAP(apssid, appassword) ? "Ready" : "Failed!");
  serial.print("soft AP IP address: ");
  serial.println(WiFi.softAPIP());
  server->on("/", handleRoot);
  server->on("/restart", handleRestart);
#ifdef RF_BUTTON
  server->on("/getrfbutton", handleGetRFbutton);
#endif
  server->on("/scan", handleScan);
  server->on("/clear", handleClear);
  server->on("/saveconfig", handleSaveConfigFile);
  server->on("/uploadconfig", handleUploadConfigFile);
  server->on("/uploadfile", HTTP_POST, []() {
    server->sendHeader("Connection", "close");
    server->send(200, "text/plain", "upload");
    ESP.reset();
  }, []() {
    HTTPUpload& upload = server->upload();
    int c;
    if (upload.status == UPLOAD_FILE_START) {
      WiFiUDP::stopAll();
      String filename = upload.filename;
      serial.print("start uploading: ");
      serial.println(filename);
    }
    else if (upload.status == UPLOAD_FILE_WRITE) {
      for (int i =  0; i < sizeof(upload.buf); i++) {
        c++;
        if (c > 50) {
          c = 0;
          serial.println();
        }
        serial.print(".");
        delay(1);
        EEPROM.write(i, upload.buf[i]);
      }
      EEPROM.commit();
      delay(3000);
    }
    else if (upload.status == UPLOAD_FILE_END) {
      serial.print("uploaded: ");
      serial.print(upload.totalSize);
      serial.println(" bytes");
      server->sendHeader("Connection", "close");
      String res = "<!DOCTYPE html>";
      res += "<html>";
      res += "<head>";
      res += "<meta http-equiv=\'Refresh\' content=\'5; url=/\' />";
      res += "</head>";
      res += "<body>";
      res += "<p><h2>Upload succcess!<br>Reboot...</h2></p>";
      res += "</body>";
      res += "</html>";
      server->send(200, "text/html", res);
    }
    else {
      server->send(500, "text/plain", "500: couldn't upload file");
    }
  });

  server->onNotFound(handleNotFound);
  httpUpdater->setup(&*server);
  server->begin();
  serial.println("HTTP server started...");
  server_started = true;
  EEPROM.write(E_MODE_ADRESS, 1);
  EEPROM.commit();
}

void handleRestart() {
  serial.println("restarting...");
  server->send(200, "text/html", "<p><h2>Restarting...</h2><p>");
  EEPROM.write(E_MODE_ADRESS, 1);
  EEPROM.commit();
  delay(3000);
  ESP.reset();
}

void handleClear() {
  serial.println("erasing EEPROM...");
  int c;
  String res = "<!DOCTYPE html>";
  res += "<html>";
  res += "<head>";
  res += "<meta http-equiv=\'Refresh\' content=\'5; url=/\' />";
  res += "</head>";
  res += "<body>";
  res += "<p><h2>Earasing EEPROM...<br>Please wait...</h2></p>";
  res += "</body>";
  res += "</html>";
  server->send(200, "text/html", res);
  delay(500);
  for (int i = 0; i < E_EEPROM_SIZE; ++i) {
    c++;
    if (c > 50) {
      c = 0;
      serial.println();
    }
    serial.print(".");
    EEPROM.write(i, 255);
    delay(2);
  }
  EEPROM.commit();
  delay(3000);
  ESP.reset();
}

void handleSaveConfigFile() {
  if (configured) {
    char ch;
    int c;
    String file_name = "";
    file_name += "attachment; filename=";
    file_name += apssid;
    file_name += ".";
    String comp_opt = getCompilOptions();
    file_name += comp_opt;
    file_name += "conf.bin";
    serial.println("saving config file...");
    server->setContentLength(E_EEPROM_SIZE);
    server->sendHeader("Content-Disposition", file_name);
    server->send(200, "application/octet-stream", "");
    for (int i = 0; i < E_EEPROM_SIZE; ++i) {
      c++;
      if (c > 50) {
        c = 0;
        serial.println();
      }
      serial.print(".");
      delay(1);
      ch = EEPROM.read(i);
      server->sendContent_P(&ch, 1);
    }
  }
  else {
    server->send(200, "text/html", "<script>confirm('no configuration in EEPROM')</script>");
    serial.println("device not configured");
  }
}

void handleUploadConfigFile() {
  server->sendHeader("Connection", "close");
  const char* res = "<form method='POST' action='/uploadfile' enctype='multipart/form-data'><input type='file' name='update'><input type='submit' value='Upload'></form>";
  server->send(200, "text/html", res);
}

void handleRoot() {
#if defined(RF_BUTTON) && defined(INPUTS) && !defined(ESP01S)//
  if (server->hasArg("crd") && server->hasArg("tmr") && server->hasArg("sid") && server->hasArg("wpw") && server->hasArg("srv") && server->hasArg("eml") && server->hasArg("auk") && server->hasArg("gid") && server->hasArg("nam") && server->hasArg("aid") && server->hasArg("apw") && server->hasArg("c2b") && server->hasArg("c1b") && server->hasArg("rfb")) {
    handleSubmit();
  }
#endif
#if defined(INPUTS) && !defined(RF_BUTTON) && !defined(ESP01S)//
  if (server->hasArg("crd") && server->hasArg("tmr") && server->hasArg("sid") && server->hasArg("wpw") && server->hasArg("srv") && server->hasArg("eml") && server->hasArg("auk") && server->hasArg("gid") && server->hasArg("nam") && server->hasArg("aid") && server->hasArg("apw") && server->hasArg("c2b") && server->hasArg("c1b")) {
    handleSubmit();
  }
#endif
#if defined(INPUTS) && defined(ESP01S)//
  if (server->hasArg("sid") && server->hasArg("wpw") && server->hasArg("srv") && server->hasArg("eml") && server->hasArg("auk") && server->hasArg("gid") && server->hasArg("nam") && server->hasArg("aid") && server->hasArg("apw") && server->hasArg("c2b")) {
    handleSubmit();
  }
#endif
#if defined(RF_BUTTON) && !defined(INPUTS)
  if (server->hasArg("sid") && server->hasArg("wpw") && server->hasArg("srv") && server->hasArg("eml") && server->hasArg("auk") && server->hasArg("gid") && server->hasArg("nam") && server->hasArg("aid") && server->hasArg("apw") && server->hasArg("rfb")) {
    handleSubmit();
  }
#endif
#if !defined(RF_BUTTON) && !defined(INPUTS)
  if (server->hasArg("sid") && server->hasArg("wpw") && server->hasArg("srv") && server->hasArg("eml") && server->hasArg("auk") && server->hasArg("gid") && server->hasArg("nam") && server->hasArg("aid") && server->hasArg("apw")) {
    handleSubmit();
  }
#endif

  else {
    serial.println("loading main page...");
    //source https://github.com/SUPLA/supla-espressif-esp/blob/master/src/html/cfg-template1.html
    String main_page = "";
    main_page += "<DOCTYPE HTML>";
    main_page += "<html>";
    main_page += "<head>";
    main_page += "<meta content = \"text/html; charset=ISO-8859-1\"";
    main_page += "http-equiv=\"content-type\">";
    main_page += "<meta name = \"viewport\" content = \"width = device-width, initial-scale = 1.0, maximum-scale = 1.0, user-scalable=0\">";
    main_page += "<title>s-one</title>";
    main_page += "<style>";
    main_page += "body {font-size: 14px;font-family: \"HelveticaNeue\", \"Helvetica Neue\", \"HelveticaNeueRoman\", \"HelveticaNeue-Roman\", \"Helvetica Neue Roman\", 'TeXGyreHerosRegular', \"Helvetica\", \"Tahoma\", \"Geneva\", \"Arial\", sans-serif; font-weight:400; font-stretch:normal;background: #00D151;color: #fff; line-height: 20px; padding: 0;}";
    main_page += ".s {width: 460px;margin: 0 auto;margin-top: calc(50vh - 340px); border: solid 3px #fff; padding: 0 10px 10px; border-radius: 3px;}";
    main_page += "#l {display: block; max-width: 150px; height: 155px; margin: -80px auto 20px; background:#00D151;padding-right: 5px;}";
    main_page += "#l path {fill:#000;}";
    main_page += ".w {margin: 3px 0 16px;padding: 5px 0px; border-radius: 3px; background: #fff; box-shadow: 0 1px 3px rgba(0,0,0,.3)}";
    main_page += "h1, h3 {margin: 10px 8px;font-family:  \"HelveticaNeueLight\", \"HelveticaNeue-Light\", \"Helvetica Neue Light\", \"HelveticaNeue\", \"Helvetica Neue\", 'TeXGyreHerosRegular', \"Helvetica\", \"Tahoma\", \"Geneva\", \"Arial\", sans-serif; font-weight:300; font-stretch:normal; color: #000; font-size: 23px;}";
    main_page += "h1 {margin-bottom: 14px;color:#fff}";
    main_page += "span {display: block;margin: 10px 7px 14px;}";
    main_page += "i {display: block;font-style:normal; position: relative;border-bottom: solid 1px #00D151;height: 42px;}";
    main_page += "i:last-child {border: none;}";
    main_page += "label {position: absolute;display: inline-block;top: 0px; left: 8px;color: #00D151; line-height: 41px;pointer-events: none;}";
    main_page += "input, select {width: calc(100% - 145px);border: none;font-size: 16px;line-height: 40px;border-radius: 0; letter-spacing: -.5px; background: #fff; color: #000;padding-left: 144px;-webkit-appearance: none;";
    main_page += "-moz-appearance: none; appearance: none;outline: none!imain_pageortant;height: 40px;}";
    main_page += "select {padding: 0px;float: right;margin: 1px 3px 1px 2px;}";
    main_page += "button {width: 100%; border: 5;background: #000;padding: 5px 10px;font-size: 16px;line-height: 40px;color: white;border-radius: 3px; box-shadow: 0 1px 3px rgba(0,0,0,.3); cursor: pointer;}";
    main_page += ".c {background: #FFE836; position: fixed; width: 100%; line-height: 80px; color: #000; top: 0; left: 0;box-shadow: 0 1px 3px rgba(0,0,0,.3); text-align: center; font-size: 26px; z-index: 100}";
    main_page += "@media all and (max-height: 920px) {";
    main_page += ".s {margin-top: 80px;}";
    main_page += "}";
    main_page += "@media all and (max-width: 900px) {";
    main_page += ".s {width: calc(100% - 20px);margin-top: 40px; border: none; padding: 0 8px; border-radius: 0px;}";
    main_page += "#l {max-width: 80px; height: auto; margin: 10px auto 20px;}";
    main_page += "h1, h3 {font-size: 19px;}";
    main_page += "i {border: none;height: auto}";
    main_page += "label {display: block;margin: 4px 0 12px;color: #00D151; font-size: 13px;position:relative; line-height:18px;}";
    main_page += "input, select {width: calc(100% - 10px);font-size: 16px;line-height: 28px;padding: 0px 5px ;border-bottom: solid 1px #00D151; }";
    main_page += "select {width: 100%;float: none;margin:0}";
    main_page += "}";
    main_page += "</style>";
    main_page += "</head>";
    main_page += "<body>";
    main_page += "<div class=\"s\">";
    main_page += "<svg version=\"1.1\" id=\"l\" x=\"0\" y=\"0\" viewBox=\"0 0 200 200\" xml:space=\"preserve\">";
    main_page += "<path d=\"M59.3,2.5c18.1,0.6,31.8,8,40.2,23.5c3.1,5.7,4.3,11.9,4.1,18.3c-0.1,3.6-0.7,7.1-1.9,10.6c-0.2,0.7-0.1,1.1,0.6,1.5c12.8,7.7,25.5,15.4,38.3,23c2.9,1.7,5.8,3.4,8.7,5.3c1,0.6,1.6,0.6,2.5-0.1c4.5-3.6,9.8-5.3,15.7-5.4c12.5-0.1,22.9,7.9,25.2,19c1.9,9.2-2.9,19.2-11.8,23.9c-8.4,4.5-16.9,4.5-25.5,0.2c-0.7-0.3-1-0.2-1.5,0.3c-4.8,4.9-9.7,9.8-14.5,14.6c-5.3,5.3-10.6,10.7-15.9,16c-1.8,1.8-3.6,3.7-5.4,5.4c-0.7,0.6-0.6,1,0,1.6c3.6,3.4,5.8,7.5,6.2,12.2c0.7,7.7-2.2,14-8.8,18.5c-12.3,8.6-30.3,3.5-35-10.4c-2.8-8.4,0.6-17.7,8.6-22.8c0.9-0.6,1.1-1,0.8-2c-2-6.2-4.4-12.4-6.6-18.6c-6.3-17.6-12.7-35.1-19-52.7c-0.2-0.7-0.5-1-1.4-0.9c-12.5,0.7-23.6-2.6-33-10.4c-8-6.6-12.9-15-14.2-25c-1.5-11.5,1.7-21.9,9.6-30.7C32.5,8.9,42.2,4.2,53.7,2.7c0.7-0.1,1.5-0.2,2.2-0.2C57,2.4,58.2,2.5,59.3,2.5z M76.5,81c0,0.1,0.1,0.3,0.1,0.6c1.6,6.3,3.2,12.6,4.7,18.9c4.5,17.7,8.9,35.5,13.3,53.2c0.2,0.9,0.6,1.1,1.6,0.9c5.4-1.2,10.7-0.8,15.7,1.6c0.8,0.4,1.2,0.3,1.7-0.4c11.2-12.9,22.5-25.7,33.4-38.7c0.5-0.6,0.4-1,0-1.6c-5.6-7.9-6.1-16.1-1.3-24.5c0.5-0.8,0.3-1.1-0.5-1.6c-9.1-4.7-18.1-9.3-27.2-14c-6.8-3.5-13.5-7-20.3-10.5c-0.7-0.4-1.1-0.3-1.6,0.4c-1.3,1.8-2.7,3.5-4.3,5.1c-4.2,4.2-9.1,7.4-14.7,9.7C76.9,80.3,76.4,80.3,76.5,81z M89,42.6c0.1-2.5-0.4-5.4-1.5-8.1C83,23.1,74.2,16.9,61.7,15.8c-10-0.9-18.6,2.4-25.3,9.7c-8.4,9-9.3,22.4-2.2,32.4c6.8,9.6,19.1,14.2,31.4,11.9C79.2,67.1,89,55.9,89,42.6z M102.1,188.6c0.6,0.1,1.5-0.1,2.4-0.2c9.5-1.4,15.3-10.9,11.6-19.2c-2.6-5.9-9.4-9.6-16.8-8.6c-8.3,1.2-14.1,8.9-12.4,16.6C88.2,183.9,94.4,188.6,102.1,188.6z M167.7,88.5c-1,0-2.1,0.1-3.1,0.3c-9,1.7-14.2,10.6-10.8,18.6c2.9,6.8,11.4,10.3,19,7.8c7.1-2.3,11.1-9.1,9.6-15.9C180.9,93,174.8,88.5,167.7,88.5z\"/>";
    main_page += "</svg>";
    main_page += "<h1>s-one config page</h1>";
    main_page += "<span>";
    main_page += "version: ";
    main_page += ver;
    main_page += " <br> compilation: ";
    String comp_opt = getCompilOptions();
    main_page += comp_opt;
    main_page += "<br>";
    main_page += "Based on SuplaDevice library Copyright(C) AC SOFTWARE SP.Z O.O.";
    //main_page += " ver: 2.3.1";
    main_page += "<br><br>GPIO configuration:<br>";
    main_page += "GPIO" + String(RELAY_PIN) + " - relay<br>";
#ifdef INPUTS
#ifndef ESP01S
    main_page += "GPIO" + String(INPUT_1_PIN) + " - INPUT 1<br>";
#endif
    main_page += "GPIO" + String(INPUT_2_PIN) + " - INPUT 2<br>";
#endif
#if defined(RF_BUTTON) || defined(RF_THERMO)
    main_page += "GPIO" + String(RF_PIN) + " - RF433 receiver<br>";
#endif
#ifdef DS_THERMO
    main_page += "GPIO" + String(DS_PIN) + " - DS_THERMO<br>";
#endif
#ifdef DHT_THERMO
    main_page += "GPIO" + String(DHTPIN) + " - DHT11<br>";
#endif
    main_page += "</span>";
    main_page += "<FORM action=\"/\" method=\"post\">";
    main_page += "<P>";
    main_page += "<div class=\"w\">";
    main_page += "<h3>Wi-Fi Settings</h3>";
    main_page += "<i>";
    main_page += "<input type=\"text\" name=\"sid\" value=\"";
    if (configured) {
      main_page += String(ssid);
    }
    main_page += "\">";
    main_page += "<label>Network name</label>";
    main_page += "</i>";
    main_page += "<i>";
    main_page += "<input type=\"text\" name=\"wpw\" value=\"";
    if (configured) {
      main_page += String(password);
    }
    main_page += "\">";
    main_page += "<label>Password</label>";
    main_page += "</i>";
    main_page += "</div>";
    main_page += "<div class=\"w\">";
    main_page += "<h3>Supla Settings</h3>";
    main_page += "<i>";
    main_page += "<input type=\"text\" name=\"srv\" value=\"";
    if (configured) {
      main_page += String(serv);
    }
    main_page += "\">";
    main_page += "<label>Server</label>";
    main_page += "</i>";
    main_page += "<i>";
    main_page += "<input type=\"text\" name=\"eml\" value=\"";
    if (configured) {
      main_page += String(email);
    }
    main_page += "\">";
    main_page += "<label>Email</label>";
    main_page += "</i>";
    main_page += "<i>";
    main_page += "<input type=\"text\" name=\"nam\" value=\"";
    if (configured) {
      main_page += String(supla_name);
    }
    main_page += "\">";
    main_page += "<label>Device name</label>";
    main_page += "</i>";
    main_page += "<i>";
    main_page += "<input type=\"text\" name=\"gid\" value=\"";
    main_page += str_guid;
    main_page += "\">";
    main_page += "<label>Device GUID</label>";
    main_page += "</i>";
    main_page += "<i>";
    main_page += "<input type=\"text\" name=\"auk\" value=\"";
    main_page += str_auth_key;
    main_page += "\">";
    main_page += "<label>Device AUTH KEY</label>";
    main_page += "</i>";
    main_page += "</div>";
    main_page += "<div class=\"w\">";
    main_page += "<h3>AP mode Settings</h3>";
    main_page += "<i>";
    main_page += "<input type=\"text\" name=\"aid\" value=\"" ;
    main_page += String(apssid) ;
    main_page += "\">";
    main_page += "<label>Network name</label>";
    main_page += "</i>";
    main_page += "<i>";
    main_page += "<input type=\"text\" name=\"apw\" value=\"";
    main_page += String(appassword);
    main_page += "\">";
    main_page += "<label>Password</label>";
    main_page += "</i>";
    main_page += "</div>";
    main_page += "<div class=\"w\">";
    main_page += "<h3>Additional Settings</h3>";
#ifdef RF_BUTTON
    main_page += "<i>";
    main_page += "<input type=\"number\" name=\"rfb\" value=\"";
    if (configured) {
      main_page += String(rf_btn_value);
    }
    main_page += "\">";
    main_page += "<label>RF button value</label>";
    main_page += "</i>";
#endif
    main_page += "<i>";
    main_page += "<select name=\"led\">";
    String l;
#ifndef ESP01S
    l = ((led == 1) ? "selected" : "");
    main_page += "  <option " + l + " value=\"1\">LED pulse</option>";
#endif
    l = ((led == 0) ? "selected" : "");
    main_page += "  <option " + l + " value=\"0\">LED OFF</option>";
    main_page += "</select>";
    main_page += "<label>Status - connected</label>";
    main_page += "</i>";
    main_page += "<i>";
    main_page += "<select name=\"sav\">";
    String r = ((save_state == 0) ? "selected" : "");
    main_page += "  <option " + r + " value=\"0\">Always LOW</option>";
    r = ((save_state == 1) ? "selected" : "");
    main_page += "  <option " + r + " value=\"1\">Always HI</option>";
    r = ((save_state == 2) ? "selected" : "");
    main_page += "  <option  " + r + " value=\"2\">Last state</option>";
    main_page += "</select>";
    main_page += "<label>Boot relay state</label>";
    main_page += "</i>";
#ifdef INPUTS
#ifndef ESP01S
    main_page += "<i>";
    main_page += "<select name=\"c1b\">";
    String b1 = ((input_1_mode == 0) ? "selected" : "");
    main_page += "<option " + b1 + " value=\"0\">Sensor NO</option>";
    b1 = ((input_1_mode == 2) ? "selected" : "");
    main_page += "<option " + b1 + " value=\"2\">HIGH trigger touch sensor</option>";
    b1 = ((input_1_mode == 4) ? "selected" : "");
    main_page += "<option " + b1 + "value=\"4\">HC-SR501 PIR sensor</option>";
    b1 = ((input_1_mode == 5) ? "selected" : "");
    main_page += "<option " + b1 + " value=\"5\">HC-SR501 PIR sensor NIGHT MODE</option>";
    main_page += "</select>";
    main_page += "<label>GPIO" + String(INPUT_1_PIN) + " INPUT 1</label>";
    main_page += "</i>";
#endif
    main_page += "<i>";
    main_page += "<select name=\"c2b\">";
    String b2 = ((input_2_mode == 0) ? "selected" : "");
    main_page += "<option " + b2 + " value=\"0\">Sensor NO</option>";
    b2 = ((input_2_mode == 1) ? "selected" : "");
    main_page += "<option " + b2 + " value=\"1\">PULL UP monostable</option>";
    b2 = ((input_2_mode == 3) ? "selected" : "");
    main_page += "<option " + b2 + " value=\"3\">PULL UP bistable </option>";
    main_page += "</select>";
    main_page += "<label>GPIO" + String(INPUT_2_PIN) + " INPUT ";
#ifndef ESP01S
    main_page += "2";
#endif
    main_page += "</label>";
    main_page += "</i>";
#ifndef ESP01S
    main_page += "<i>";
    main_page += "<input type=\"text\" name=\"crd\" value=\"";
    main_page += "{" + String(coord[0]) + ", " + String(coord[1]) + ", " + String(coord[2]) + "}";
    main_page += "\">";
    main_page += "<label>Coord.{lat,long,zone}</label>";
    main_page += "</i>";
    main_page += "<i>";
    main_page += "<input type=\"number\" name=\"tmr\" value=\"";
    main_page += String(pir_timer);
    main_page += "\">";
    main_page += "<label>PIR time (seconds)</label>";
    main_page += "</i>";
#endif
#endif
    main_page += "</div>";
    main_page += "<button type=\"submit\">SAVE AND RESTART</button>";
#ifdef RF_BUTTON
    main_page += "<button type=\"button\" onclick=\"window.location.href='/getrfbutton'\">Get RF button value</button>";
#endif
    main_page += "<button type=\"button\" onclick=\"window.location.href='/scan'\">Scan WiFi networks</button>";
    main_page += "<button type=\"button\" onclick=\"window.location.href='/update'\">Update firmware</button>";
    main_page += "<button type=\"button\" onclick=\"window.location.href='/saveconfig'\">Save config file - NOT ENCRYPTED!</button>";
    main_page += "<button type=\"button\" onclick=\"window.location.href='/uploadconfig'\">Upload config file</button>";
    main_page += "<button type=\"button\" onclick=\"window.location.href='/restart'\">RESTART WITHOUT SAVING</button>";
    main_page += "<button type=\"button\" onclick=\"window.location.href='/clear'\">! CLEAR EEPROM !</button>";
    main_page += "</P>";
    main_page += "</FORM>";
    main_page += "</div>";
    main_page += "</body>";
    main_page += "</html>";
    server->send(200, "text/html", main_page);
    config_server_connected = true;
  }
}

void getRandomGuidAndAuthkey() {
  byte uuidNumber[16];
  serial.println("generating guid...");
  ESP8266TrueRandom.uuid(uuidNumber);
  str_guid = getUuid(uuidNumber);
  serial.println(str_guid);
  ESP8266TrueRandom.uuid(uuidNumber);
  serial.println("generating authkey...");
  str_auth_key = getUuid(uuidNumber);
  serial.println(str_auth_key);
}

void handleScan() {
  serial.println("scan wifi networks...");
  int n = WiFi.scanNetworks();
  String res = "";
  res += "<!DOCTYPE HTML>";
  res += "<html>";
  res += "<head>";
  res += "<meta content=\"text/html; charset=ISO-8859-1\"";
  res += " http-equiv=\"content-type\">";
  res += "<meta name = \"viewport\" content = \"width = device-width, initial-scale = 1.0, maximum-scale = 1.0, user-scalable=0\">";
  res += "<title>Supla</title>";
  res += "<style>";
  res += "body {font-size: 14px;font-family: \"HelveticaNeue\", \"Helvetica Neue\", \"HelveticaNeueRoman\", \"HelveticaNeue-Roman\", \"Helvetica Neue Roman\", 'TeXGyreHerosRegular', \"Helvetica\", \"Tahoma\", \"Geneva\", \"Arial\", sans-serif; font-weight:400; font-stretch:normal;background: #00D151;color: #fff; line-height: 20px; padding: 0;}";
  res += ".s {width: 460px;margin: 0 auto;margin-top: calc(50vh - 340px); border: solid 3px #fff; padding: 0 10px 10px; border-radius: 3px;}";
  res += "#l {display: block; max-width: 150px; height: 155px; margin: -80px auto 20px; background:#00D151;padding-right: 5px;}";
  res += "#l path {fill:#000;}";
  res += ".w {margin: 3px 0 16px;padding: 5px 0px; border-radius: 3px; background: #fff; box-shadow: 0 1px 3px rgba(0,0,0,.3)}";
  res += "h1, h3 {margin: 10px 8px;font-family:  \"HelveticaNeueLight\", \"HelveticaNeue-Light\", \"Helvetica Neue Light\", \"HelveticaNeue\", \"Helvetica Neue\", 'TeXGyreHerosRegular', \"Helvetica\", \"Tahoma\", \"Geneva\", \"Arial\", sans-serif; font-weight:300; font-stretch:normal; color: #000; font-size: 23px;}";
  res += "h1 {margin-bottom: 14px;color:#fff}";
  res += "span {display: block;margin: 10px 7px 14px;}";
  res += "i {display: block;font-style:normal; position: relative;border-bottom: solid 1px #00D151;height: 42px;}";
  res += "i:last-child {border: none;}";
  res += "label {position: absolute;display: inline-block;top: 0px; left: 8px;color: #00D151; line-height: 41px;pointer-events: none;}";
  res += "input, select {width: calc(100% - 145px);border: none;font-size: 16px;line-height: 40px;border-radius: 0; letter-spacing: -.5px; background: #fff; color: #000;padding-left: 144px;-webkit-appearance: none;";
  res += "-moz-appearance: none; appearance: none;outline: none!important;height: 40px;}";
  res += "select {padding: 0px;float: right;margin: 1px 3px 1px 2px;}";
  res += "button {width: 100%; border: 5;background: #000;padding: 5px 10px;font-size: 16px;line-height: 40px;color: white;border-radius: 3px; box-shadow: 0 1px 3px rgba(0,0,0,.3); cursor: pointer;}";
  res += ".c {background: #FFE836; position: fixed; width: 100%; line-height: 80px; color: #000; top: 0; left: 0;box-shadow: 0 1px 3px rgba(0,0,0,.3); text-align: center; font-size: 26px; z-index: 100}";
  res += "@media all and (max-height: 920px) {";
  res += ".s {margin-top: 80px;}";
  res += "}";
  res += "@media all and (max-width: 900px) {";
  res += ".s {width: calc(100% - 20px);margin-top: 40px; border: none; padding: 0 8px; border-radius: 0px;}";
  res += "#l {max-width: 80px; height: auto; margin: 10px auto 20px;}";
  res += "h1, h3 {font-size: 19px;}";
  res += "i {border: none;height: auto}";
  res += "label {display: block;margin: 4px 0 12px;color: #00D151; font-size: 13px;position:relative; line-height:18px;}";
  res += "input, select {width: calc(100% - 10px);font-size: 16px;line-height: 28px;padding: 0px 5px ;border-bottom: solid 1px #00D151; }";
  res += "select {width: 100%;float: none;margin:0}";
  res += "}";
  res += "</style>";
  res += "</head>";
  res += "<body>";
  res += "<div class=\"s\">";
  res += "<svg version=\"1.1\" id=\"l\" x=\"0\" y=\"0\" viewBox=\"0 0 200 200\" xml:space=\"preserve\">";
  res += "<path d=\"M59.3,2.5c18.1,0.6,31.8,8,40.2,23.5c3.1,5.7,4.3,11.9,4.1,18.3c-0.1,3.6-0.7,7.1-1.9,10.6c-0.2,0.7-0.1,1.1,0.6,1.5c12.8,7.7,25.5,15.4,38.3,23c2.9,1.7,5.8,3.4,8.7,5.3c1,0.6,1.6,0.6,2.5-0.1c4.5-3.6,9.8-5.3,15.7-5.4c12.5-0.1,22.9,7.9,25.2,19c1.9,9.2-2.9,19.2-11.8,23.9c-8.4,4.5-16.9,4.5-25.5,0.2c-0.7-0.3-1-0.2-1.5,0.3c-4.8,4.9-9.7,9.8-14.5,14.6c-5.3,5.3-10.6,10.7-15.9,16c-1.8,1.8-3.6,3.7-5.4,5.4c-0.7,0.6-0.6,1,0,1.6c3.6,3.4,5.8,7.5,6.2,12.2c0.7,7.7-2.2,14-8.8,18.5c-12.3,8.6-30.3,3.5-35-10.4c-2.8-8.4,0.6-17.7,8.6-22.8c0.9-0.6,1.1-1,0.8-2c-2-6.2-4.4-12.4-6.6-18.6c-6.3-17.6-12.7-35.1-19-52.7c-0.2-0.7-0.5-1-1.4-0.9c-12.5,0.7-23.6-2.6-33-10.4c-8-6.6-12.9-15-14.2-25c-1.5-11.5,1.7-21.9,9.6-30.7C32.5,8.9,42.2,4.2,53.7,2.7c0.7-0.1,1.5-0.2,2.2-0.2C57,2.4,58.2,2.5,59.3,2.5z M76.5,81c0,0.1,0.1,0.3,0.1,0.6c1.6,6.3,3.2,12.6,4.7,18.9c4.5,17.7,8.9,35.5,13.3,53.2c0.2,0.9,0.6,1.1,1.6,0.9c5.4-1.2,10.7-0.8,15.7,1.6c0.8,0.4,1.2,0.3,1.7-0.4c11.2-12.9,22.5-25.7,33.4-38.7c0.5-0.6,0.4-1,0-1.6c-5.6-7.9-6.1-16.1-1.3-24.5c0.5-0.8,0.3-1.1-0.5-1.6c-9.1-4.7-18.1-9.3-27.2-14c-6.8-3.5-13.5-7-20.3-10.5c-0.7-0.4-1.1-0.3-1.6,0.4c-1.3,1.8-2.7,3.5-4.3,5.1c-4.2,4.2-9.1,7.4-14.7,9.7C76.9,80.3,76.4,80.3,76.5,81z M89,42.6c0.1-2.5-0.4-5.4-1.5-8.1C83,23.1,74.2,16.9,61.7,15.8c-10-0.9-18.6,2.4-25.3,9.7c-8.4,9-9.3,22.4-2.2,32.4c6.8,9.6,19.1,14.2,31.4,11.9C79.2,67.1,89,55.9,89,42.6z M102.1,188.6c0.6,0.1,1.5-0.1,2.4-0.2c9.5-1.4,15.3-10.9,11.6-19.2c-2.6-5.9-9.4-9.6-16.8-8.6c-8.3,1.2-14.1,8.9-12.4,16.6C88.2,183.9,94.4,188.6,102.1,188.6z M167.7,88.5c-1,0-2.1,0.1-3.1,0.3c-9,1.7-14.2,10.6-10.8,18.6c2.9,6.8,11.4,10.3,19,7.8c7.1-2.3,11.1-9.1,9.6-15.9C180.9,93,174.8,88.5,167.7,88.5z\"/>";
  res += "</svg>";
  res += "<h1>S-ONE config page</h1>";
  res += "<span>";
  res += "</span>";
  res += "<div class=\"w\">";
  res += "<h3>Aviable wireless networks:</h3>";
  for (int i = 0; i < n; ++i) {
    res += "<i>";
    res += "<label>";
    res += i + 1;
    res += ": ";
    res += WiFi.SSID(i);
    res += " (RSSI: ";
    res += WiFi.RSSI(i);
    res += " dBm)";
    res += (WiFi.encryptionType(i) == ENC_TYPE_NONE) ? " [no encrypted] " : " ";
    res += "</label>";
    res += "</i>";
  }
  res += "</div>";
  res += "<button type=\"button\" onclick=\"window.location.href='/scan'\">Refresh</button>";
  res += "<button type=\"button\" onclick=\"window.location.href='/'\">Return main page</button>";
  res += "</div>";
  res += "</body>";
  res += "</html>";
  server->send(200, "text/html", res);
}

void handleSubmit() {
  serial.println("submit config...");
  String res = "<p>ssid: ";
  String sid = server->arg("sid");
  res += sid;
  res += "<br>";
  res += "password: ";
  String wpw = server->arg("wpw");
  res += wpw;
  res += "<br>";
  res += "server: ";
  String srv = server->arg("srv");
  res += srv;
  res += "<br>";
  res += "email: ";
  String eml = server->arg("eml");
  res += eml;
  res += "<br>";
  res += "authkey: ";
  String auk = server->arg("auk");
  res += auk;
  res += "<br>";
  res += "GUID: ";
  String gid = server->arg("gid");
  res += gid;
  res += "<br>";
  res += "name: ";
  String nam = server->arg("nam");
  res += nam;
  res += "<br>";
  res += "AP: ";
  String aid = server->arg("aid");
  res += aid;
  res += "<br>";
  res += "password: ";
  String apw = server->arg("apw");
  res += apw;
  res += "<br>";
  res += "LED: ";
  String led = server->arg("led");
  res += led;
  res += "<br>";
  res += "save state: ";
  String sav = server->arg("sav");
  res += sav;
  res += "<br>";
#ifdef INPUTS
  res += "channel 2 button: ";
  String c2b = server->arg("c2b");
  res += c2b;
  res += "<br>";
#ifndef ESP01S
  res += "channel 1 button: ";
  String c1b = server->arg("c1b");
  res += c1b;
  res += "<br>";
  res += "cordinates: ";
  String crd = server->arg("crd");
  res += crd;
  res += "<br>";
  res += "PIR time: ";
  String tmr = server->arg("tmr");
  res += tmr;
  res += "<br>";
#endif
#endif
#ifdef RF_BUTTON
  res += "rf button: ";
  String rfb = server->arg("rfb");
  res += rfb;
  res += "<br>";
#endif
  res += "<br>";
  res += "</P><BR>";
  res += "<p><h2> Config saved. Restarting...</h2><p>";
  server->send(200, "text/html", res);
#if defined(RF_BUTTON) && defined(INPUTS) && !defined(ESP01S)
  writeToMemory(sid, wpw, srv, eml, auk, gid, nam, led, sav, aid, apw, c2b, c1b, rfb, crd, tmr);
#endif
#if defined(INPUTS) && !defined(RF_BUTTON) && !defined(ESP01S)
  writeToMemory(sid, wpw, srv, eml, auk, gid, nam, led, sav, aid, apw, c2b, c1b, crd, tmr);
#endif
#if defined(INPUTS) && defined(ESP01S)
  writeToMemory(sid, wpw, srv, eml, auk, gid, nam, led, sav, aid, apw, c2b);
#endif
#if defined(RF_BUTTON) && !defined(INPUTS)
  writeToMemory(sid, wpw, srv, eml, auk, gid, nam, led, sav, aid, apw, rfb);
#endif
#if !defined(RF_BUTTON) && !defined(INPUTS)
  writeToMemory(sid, wpw, srv, eml, auk, gid, nam, led, sav, aid, apw);
#endif
  server->send(200, "text/html", res);
}

#ifdef RF_BUTTON

void handleGetRFbutton() {
  serial.println("get rf button...");
  String res = "<!DOCTYPE html>";
  res += "<html>";
  res += "<head>";
  res += "<meta http-equiv=\"Refresh\" content=\"10; url=/\" />";
  res += "</head>";
  res += "<body>";
  res += "<p><h2>Receiving...<br>Please press the button on remote control.<br>This page closes automatically after a few seconds.</h2></p>";
  res += "</body>";
  res += "</html>";
  server->send(200, "text/html", res);
  get_rf_button = true;
}
#endif

#if defined(RF_BUTTON) && defined(INPUTS) && !defined(ESP01S)
void writeToMemory(String sid, String wpw, String srv, String eml, String auk,
                   String gid, String nam, String led, String sav, String aid, String apw, String c2b,
                   String c1b, String rfb, String crd, String tmr) {
  serial.println("write to memmory...");
  sid += ";";
  writeEEPROM(sid, E_SSID_ADRESS);
  wpw += ";";
  writeEEPROM(wpw, E_WPASS_ADRESS);
  srv += ";";
  writeEEPROM(srv, E_SERVER_ADRESS);
  eml += ";";
  writeEEPROM(eml, E_EMAIL_ADRESS);
  auk += ";";
  writeEEPROM(auk, E_AUTHK_ADRESS);
  gid += ";";
  writeEEPROM(gid, E_GUID_ADRESS);
  nam += ";";
  writeEEPROM(nam, E_NAME_ADRESS);
  led += ";";
  writeEEPROM(led, E_LED_ADRESS);
  sav += ";";
  writeEEPROM(sav, E_SAVE_ADRESS);
  aid += ";";
  writeEEPROM(aid, E_APSSID_ADRESS);
  apw += ";";
  writeEEPROM(apw, E_APPASS_ADRESS);
  c2b += ";";
  writeEEPROM(c2b, E_INPUT_2_ADRESS);
  c1b += ";";
  writeEEPROM(c1b, E_INPUT_1_ADRESS);
  rfb += ";";
  writeEEPROM(rfb, E_RF_BTN_VAL_ADRESS);
  crd += ";";
  writeEEPROM(crd, E_COORD_ADRESS);
  tmr += ";";
  writeEEPROM(tmr, E_PIR_TIMER_ADRESS);
  EEPROM.write(E_MODE_ADRESS, 1);
  EEPROM.write(E_CONF_STATE, 1);
  delay(500);
  EEPROM.commit();
  serial.println("saved (1)");
  delay(3000);
  ESP.reset();
}
#endif

#if defined(INPUTS) && !defined(RF_BUTTON) && !defined(ESP01S)
void writeToMemory(String sid, String wpw, String srv, String eml, String auk,
                   String gid, String nam, String led, String sav, String aid, String apw,
                   String c2b, String c1b, String crd, String tmr) {
  serial.println("write to memmory...");
  sid += ";";
  writeEEPROM(sid, E_SSID_ADRESS);
  wpw += ";";
  writeEEPROM(wpw, E_WPASS_ADRESS);
  srv += ";";
  writeEEPROM(srv, E_SERVER_ADRESS);
  eml += ";";
  writeEEPROM(eml, E_EMAIL_ADRESS);
  auk += ";";
  writeEEPROM(auk, E_AUTHK_ADRESS);
  gid += ";";
  writeEEPROM(gid, E_GUID_ADRESS);
  nam += ";";
  writeEEPROM(nam, E_NAME_ADRESS);
  led += ";";
  writeEEPROM(led, E_LED_ADRESS);
  sav += ";";
  writeEEPROM(sav, E_SAVE_ADRESS);
  aid += ";";
  writeEEPROM(aid, E_APSSID_ADRESS);
  apw += ";";
  writeEEPROM(apw, E_APPASS_ADRESS);
  c2b += ";";
  writeEEPROM(c2b, E_INPUT_2_ADRESS);
  c1b += ";";
  writeEEPROM(c1b, E_INPUT_1_ADRESS);
  crd += ";";
  writeEEPROM(crd, E_COORD_ADRESS);
  tmr += ";";
  writeEEPROM(tmr, E_PIR_TIMER_ADRESS);
  EEPROM.write(E_MODE_ADRESS, 1);
  EEPROM.write(E_CONF_STATE, 1);
  delay(500);
  EEPROM.commit();
  serial.println("saved (2)");
  delay(3000);
  ESP.reset();
}
#endif

#if defined(INPUTS) && defined(ESP01S)
void writeToMemory(String sid, String wpw, String srv, String eml, String auk,
                   String gid, String nam, String led, String sav, String aid, String apw,
                   String c2b) {
  serial.println("write to memmory...");
  sid += ";";
  writeEEPROM(sid, E_SSID_ADRESS);
  wpw += ";";
  writeEEPROM(wpw, E_WPASS_ADRESS);
  srv += ";";
  writeEEPROM(srv, E_SERVER_ADRESS);
  eml += ";";
  writeEEPROM(eml, E_EMAIL_ADRESS);
  auk += ";";
  writeEEPROM(auk, E_AUTHK_ADRESS);
  gid += ";";
  writeEEPROM(gid, E_GUID_ADRESS);
  nam += ";";
  writeEEPROM(nam, E_NAME_ADRESS);
  led += ";";
  writeEEPROM(led, E_LED_ADRESS);
  sav += ";";
  writeEEPROM(sav, E_SAVE_ADRESS);
  aid += ";";
  writeEEPROM(aid, E_APSSID_ADRESS);
  apw += ";";
  writeEEPROM(apw, E_APPASS_ADRESS);
  c2b += ";";
  writeEEPROM(c2b, E_INPUT_2_ADRESS);
  EEPROM.write(E_MODE_ADRESS, 1);
  EEPROM.write(E_CONF_STATE, 1);
  delay(500);
  EEPROM.commit();
  serial.println("saved (3)");
  delay(3000);
  ESP.reset();
}
#endif


#if defined(RF_BUTTON) && !defined(INPUTS)
void writeToMemory(String sid, String wpw, String srv, String eml, String auk, String gid, String nam, String led, String sav, String aid, String apw, String rfb) {
  serial.println("write to memmory...");
  sid += ";";
  writeEEPROM(sid, E_SSID_ADRESS);
  wpw += ";";
  writeEEPROM(wpw, E_WPASS_ADRESS);
  srv += ";";
  writeEEPROM(srv, E_SERVER_ADRESS);
  eml += ";";
  writeEEPROM(eml, E_EMAIL_ADRESS);
  auk += ";";
  writeEEPROM(auk, E_AUTHK_ADRESS);
  gid += ";";
  writeEEPROM(gid, E_GUID_ADRESS);
  nam += ";";
  writeEEPROM(nam, E_NAME_ADRESS);
  led += ";";
  writeEEPROM(led, E_LED_ADRESS);
  sav += ";";
  writeEEPROM(sav, E_SAVE_ADRESS);
  aid += ";";
  writeEEPROM(aid, E_APSSID_ADRESS);
  apw += ";";
  writeEEPROM(apw, E_APPASS_ADRESS);
  rfb += ";";
  writeEEPROM(rfb, E_RF_BTN_VAL_ADRESS);
  crd += ";";
  writeEEPROM(crd, E_COORD_ADRESS);
  tmr += ";";
  writeEEPROM(tmr, E_PIR_TIMER_ADRESS);
  EEPROM.write(E_MODE_ADRESS, 1);
  EEPROM.write(E_CONF_STATE, 1);
  delay(500);
  EEPROM.commit();
  serial.println("saved (4)");
  delay(3000);
  ESP.reset();
}
#endif

#if !defined(RF_BUTTON) && !defined(INPUTS)
void writeToMemory(String sid, String wpw, String srv, String eml, String auk, String gid, String nam, String led, String sav, String aid, String apw) {
  serial.println("write to memmory...");
  sid += ";";
  writeEEPROM(sid, E_SSID_ADRESS);
  wpw += ";";
  writeEEPROM(wpw, E_WPASS_ADRESS);
  srv += ";";
  writeEEPROM(srv, E_SERVER_ADRESS);
  eml += ";";
  writeEEPROM(eml, E_EMAIL_ADRESS);
  auk += ";";
  writeEEPROM(auk, E_AUTHK_ADRESS);
  gid += ";";
  writeEEPROM(gid, E_GUID_ADRESS);
  nam += ";";
  writeEEPROM(nam, E_NAME_ADRESS);
  led += ";";
  writeEEPROM(led, E_LED_ADRESS);
  sav += ";";
  writeEEPROM(sav, E_SAVE_ADRESS);
  aid += ";";
  writeEEPROM(aid, E_APSSID_ADRESS);
  apw += ";";
  writeEEPROM(apw, E_APPASS_ADRESS);
  EEPROM.write(E_MODE_ADRESS, 1);
  EEPROM.write(E_CONF_STATE, 1);
  delay(500);
  EEPROM.commit();
  serial.println("saved(5)");
  delay(3000);
  ESP.reset();
}
#endif

void writeEEPROM(String x, int pos) {
  for (int n = pos; n < x.length() + pos; n++) {
    EEPROM.write(n, x[n - pos]);
    serial.print(".");
  }
  serial.println();
}

void handleNotFound() {
  server->send(404, "text/plain", "Not Found");
}

void gotoNormalMode() {
  serial.println("go to normal mode...");
  serial.println("quit from telnet...");
  delay(100);
  serial.end();
  EEPROM.write(E_MODE_ADRESS, 1);
  EEPROM.commit();
  delay(500);
  ESP.reset();
}

void gotoConfigMode() {
  timeStamp();
  serial.println("go to config mode...");
  serial.println("quit from telnet...");
  delay(100);
  serial.end();
  normal_mode = false;
#ifdef INPUTS
  config_btn_time = 0;
#endif
  EEPROM.write(E_MODE_ADRESS, 0);
  EEPROM.commit();
  WiFi.softAPdisconnect(false);
  delay(500);
  ESP.reset();
}

void loop() {
  if (normal_mode) {
    SuplaDevice.iterate();
  }
  else {
    unsigned long n = millis();
    if (n > last_led_time + 100) {
      digitalWrite(led_pin, (digitalRead(led_pin) == HIGH ? LOW : HIGH));
      last_led_time = n;
    }
    if (server_started) {
      server->handleClient();
    }
    if (!config_server_connected && millis() > config_mode_start_time + 300000) {
      serial.println("not config page opened, rebooting...");
      delay(1000);
      ESP.reset();
    }
  }

#ifdef RF_BUTTON
  if (mySwitch.available()) {
    int value = mySwitch.getReceivedValue();
    if (value == 0) {
      serial.println("RF unknown encoding");
    }
    else {
      if (!normal_mode && !rf_new_value && get_rf_button) {
        rf_btn_value = value;
        rf_new_value = true;
        serial.println(rf_btn_value);
      }
      if (normal_mode) {
        if (value == rf_btn_value) {
          if (millis() > rf_switch_time + 1000) {
            rf_switch = true;
            serial.println("RF code recived");
          }
        }
      }
      mySwitch.resetAvailable();
    }
  }
#endif

#ifdef TELNET
  SerialAndTelnet.handle();
  const char *menu = "\n\r\033[1mc\033[0m - configuration mode\n\r\033[1mn\033[0m - normal mode\n\r\033[1mi\033[0m - device info\n\r\033[1mq\033[0m - quit from telnet\n\r\033[1mscroll up\033[0m to view the logs\n\r";
  if (serial.available() > 0) {
    char c = serial.read();
    switch (c) {
      case 'c':
        gotoConfigMode();
        break;
      case 'n':
        gotoNormalMode();
        break;
      case 'i':
        showInfo();
        break;
      case 'h':
        serial.println(menu);
        break;
      case 'q':
        timeStamp();
        serial.println("quit from telenet...");
        delay(500);
        serial.end();
        delay(500);
        serial.begin(9600);
        break;
      default:
        serial.print("");
        break;
    }
  }
#endif
}
