/*
  FSWebServer - Example WebServer with SPIFFS backend for esp8266
  Copyright (c) 2015 Hristo Gochkov. All rights reserved.
  This file is part of the ESP8266WebServer library for Arduino environment.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  upload the contents of the data folder with MkSPIFFS Tool ("ESP8266 Sketch Data Upload" in Tools menu in Arduino IDE)
  or you can upload the contents of a folder if you CD in that folder and run the following command:
  for file in `ls -A1`; do curl -F "file=@$PWD/$file" esp8266fs.local/edit; done

  access the sample web page at http://esp8266fs.local
  edit the page by going to http://esp8266fs.local/edit
*/
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <DNSServer.h>
#include <WiFiManager.h>
#include <SPI.h>
#include <WebSocketsServer.h>
#include <Hash.h>
#include "OneButton.h"

#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>

//#include "max6675.h"
#include "Adafruit_MAX31855.h"
#include "RTClib.h"
#include "DHT12_I2C.h"
#include <Wire.h>

#include <FS.h>


#define LED2        15
#define LED1        1
#define SELECT_KEY    13   // SW2
#define START_KEY   0      // SW3

#define DBG_OUTPUT_PORT Serial
#define USE_SERIAL Serial

#define DHTPIN 0    // what digital pin we're connected to
#define DHTTYPE DHT11   // DHT 11

#define MAX31855_CS_PIN     2

#define LOGGER_STRUCT_SIZE 8

struct logger_struct {
  uint32_t timestamp;
  uint16_t mSec;
  int16_t MAX31855;
};

// the struct much be 4, 8,  12.... byte for file writting.
//struct logger_struct {
//  uint32_t timestamp;
//  int8_t temperature;
//  uint8_t humidity;
//  int16_t ntc;
//  float MAX6675;
//};

//typedef union {
//  uint32_t regValue;
//  struct {
//    unsigned temperature : 14;
//    unsigned reserved1 : 1;
//    unsigned fault : 1;
//    unsigned internalTemperature : 12;
//    unsigned reserved2 : 1;
//    unsigned faultMode: 3;
//  };
//} MAX31855_REG;

struct systemConfig {
  uint32_t samplingInterval = 1000;
  uint8_t logDataON = 0;
  uint8_t diskFull = 0;
};



uint8_t wsConnection = 0;

ADC_MODE(ADC_VCC);
//MAX6675 thermocouple(MAX6675_CS_PIN);
//Adafruit_MAX31855 thermocouple(MAX31855_CS_PIN);





DHT12_I2C dht12;
RTC_DS1307 rtc;
Ds1307SqwPinMode modes[] = {OFF, ON, SquareWave1HZ, SquareWave4kHz, SquareWave8kHz, SquareWave32kHz};

// Setup a new OneButton on SELECT_KEY
OneButton selectKey(SELECT_KEY, true);
OneButton startKey(START_KEY, true);

const char* updatehost = "pchunter.synology.me";
const char* ssid = "xx";
const char* password = "xx";
const char* host = "logger";
const uint32_t interval = 1000;     // Sampling interval
const uint8_t LOG_BUFF_SIZE = 10;
const size_t DATA_FILE_SIZE = 1500000;
const size_t DATA_FILE_WARNING_SIZE = 1400000;
// 8,

logger_struct logData;
systemConfig sysConfig;
uint32_t startTime;
unsigned long previousMillis = 0;
unsigned long sysCheckPreviousMillis = 0;

bool recordStop = false;
bool probeError = false;

uint8_t buffCount = 0;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
uint32_t LED1_Blink_Speed = 0x300;

ESP8266WebServer server(80);
ESP8266HTTPUpdateServer httpUpdater;
WebSocketsServer webSocket = WebSocketsServer(81);

//holds the current upload
File fsUploadFile;


void setup(void) {

  max31855Setup();

  DBG_OUTPUT_PORT.begin(115200);
  DBG_OUTPUT_PORT.print("\n");
  DBG_OUTPUT_PORT.setDebugOutput(true);
  SPIFFS.begin();
  {
    Dir dir = SPIFFS.openDir("/");
    while (dir.next()) {
      String fileName = dir.fileName();
      size_t fileSize = dir.fileSize();
      DBG_OUTPUT_PORT.printf("FS File: %s, size: %s\n", fileName.c_str(), formatBytes(fileSize).c_str());
    }
    DBG_OUTPUT_PORT.printf("\n");
  }


  // load system config value
  File sys = SPIFFS.open("/config.sys", "r");
  if (sys) {
    sys.readBytes((char*) &sysConfig, sizeof(sysConfig));
    sys.close();
  }





  pinMode(SELECT_KEY, INPUT);
  pinMode(START_KEY, INPUT);

  if ( !digitalRead(SELECT_KEY)) {
    delay(500);
    for (uint8_t i = 0 ; i < 20 ; i++) {
      rtc.writeSqwPinMode(ON);
      delay(50);
      rtc.writeSqwPinMode(OFF);
      delay(50);
    }
    if ( digitalRead(SELECT_KEY)) {
      WiFi.mode(WIFI_AP);
      WiFi.softAP("LoggerDirect", "12345678");
    } else {
      WiFiManager wifiManager;
      wifiManager.startConfigPortal("LoggerSetup");
      delay(2000);
      ESP.reset();
    }
  } else {
    WiFi.mode(WIFI_STA);
    WiFi.begin();
  }

  //  //WIFI INIT
  DBG_OUTPUT_PORT.printf("Connecting to %s\n", ssid);
  //  if (String(WiFi.SSID()) != String(ssid)) {
  //    WiFi.begin(ssid, password);
  //  }


  uint8_t count = 3;
  while (!count) {
    if (WiFi.status() == WL_CONNECTED)
      break;
    delay(500);
    DBG_OUTPUT_PORT.print(".");
    count--;
  }

  rtc.begin();
  rtc.writeSqwPinMode(ON);

  if (! rtc.isrunning()) {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));


    startTime  = getNtpTime() - millis() / 1000;


  } else {
    startTime = rtc.now().unixtime() - millis() / 1000;
  }





  //  DBG_OUTPUT_PORT.println("");
  //  DBG_OUTPUT_PORT.print("Connected! IP address: ");
  //  DBG_OUTPUT_PORT.println(WiFi.localIP());

  MDNS.begin(host);
  //  DBG_OUTPUT_PORT.print("Open http://");
  //  DBG_OUTPUT_PORT.print(host);
  //  DBG_OUTPUT_PORT.println(".local/edit to see the file browser");


  //SERVER INIT
  //list directory
  server.on("/list", HTTP_GET, handleFileList);
  //load editor
  server.on("/edit", HTTP_GET, []() {
    if (!handleFileRead("/edit.htm")) server.send(404,  F( "text/plain"), "FileNotFound");
  });
  //create file  server.on("/edit", HTTP_PUT, handleFileCreate);
  //delete file
  server.on("/edit", HTTP_DELETE, handleFileDelete);
  //first callback is called after the request has ended with all parsed arguments
  //second callback handles file uploads at that location
  server.on("/edit", HTTP_POST, []() {
    server.send(200,  F( "text/plain"), "");
  }, handleFileUpload);

  //reboot
  server.on("/reboot", HTTP_GET, []() {
    server.send(200,  F( "text/plain"), "Logger rebooting...");
    rtc.writeSqwPinMode(ON);
    delay(2000);
    ESP.reset();
  });

  // adjust RTC
  server.on("/rtc", HTTP_GET, adjustRTC);

  // set Config
  server.on("/setconfig", HTTP_GET, setConfig);

  // get Config
  server.on("/sysconfig", HTTP_GET, getConfig);

  // clean all data
  server.on("/clean", HTTP_GET, []() {
    recordStop = true;
    deleteData();
    server.send(200,  F( "text/plain"), "OK");
    recordStop = false;
  });

  //http update
  server.on("/httpupdate", HTTP_GET, httpUpdate);

  //called when the url is not defined here
  //use it to load content from SPIFFS
  server.onNotFound([]() {
    if (!handleFileRead(server.uri()))
      server.send(404,  F( "text/plain"), "FileNotFound");
  });

  // Get system data
  server.on("/sysinfo", HTTP_GET, sysinfo );




  httpUpdater.setup(&server);
  server.begin();
  DBG_OUTPUT_PORT.println("HTTP server started");

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);


  ArduinoOTA.setHostname(host);
  ArduinoOTA.begin();

  MDNS.addService("http", "tcp", 80);


  //  pinMode(13, OUTPUT);
  //  pinMode(15 , OUTPUT);
  //  pinMode(0, OUTPUT);

  //  digitalWrite(13, HIGH);
  //  digitalWrite(15, HIGH);
  //  digitalWrite(0, HIGH);

  selectKey.attachLongPressStart(selectLongPress);
  startKey.attachLongPressStart(startLongPress);



  delay(1000); //delay 3s for Sensor stability

  digitalWrite(LED1, HIGH);
  pinMode(LED1, OUTPUT);

}


void loop(void) {

  //  GPOS = 0xA001;
  server.handleClient();
  webSocket.loop();
  ArduinoOTA.handle();
  selectKey.tick();
  startKey.tick();


  //  GPOC = 0xA001;

  //  digitalWrite(13, !digitalRead(13));
  //  digitalWrite(15, !digitalRead(15));
  //  digitalWrite(16, !digitalRead(16));

  logger_struct LogDataBuff[LOG_BUFF_SIZE];

  unsigned long currentMillis = millis();

  // Recording LED1 Blinking.
  digitalWrite(LED1,  !((currentMillis & LED1_Blink_Speed) && sysConfig.logDataON || sysConfig.diskFull));

  if (currentMillis - previousMillis >= sysConfig.samplingInterval || previousMillis == 0) {
    previousMillis = currentMillis;

    //DateTime now = rtc.now();
    //yield();
    //logData.timestamp = now.unixtime();

    uint32_t m = millis();
    logData.timestamp = startTime + m / 1000;
    logData.mSec = m % 1000;

    // dht12.read(logData.DHT12_temp, logData.DHT12_hum);
    // logData.MAX6675 = thermocouple.readC();

    //float tc_temp = thermocouple.readCelsius();


    int16_t tc_temp = max31855_read();

    if (probeError &&  (0xffffffff == tc_temp) ) {
      delay(500);
      tc_temp = max31855_read();
      probeError = false;
    }

    if ( (0xffffffff != tc_temp) ) {
      logData.MAX31855 = tc_temp;
      if (sysConfig.logDataON && !recordStop ) {
        LogDataBuff[buffCount] = logData;
        buffCount++;
        if (buffCount >= LOG_BUFF_SIZE) {
          buffCount = 0;
          File binfile = SPIFFS.open("/data.bin", "a+");
          if (binfile) {
            binfile.write((uint8_t *)&LogDataBuff, LOGGER_STRUCT_SIZE * LOG_BUFF_SIZE);
            // binfile.close();
          }
        }

      }

    } else {
      //if reading thermocouple error send higher error value for javascript handle
      logData.MAX31855 = 5000;
      probeError = true;
    }

    if (wsConnection != 0)

      webSocket.broadcastBIN((uint8_t *)&logData, LOGGER_STRUCT_SIZE);

    else {
      // check wifi status turn on led
      if (WiFi.status() == WL_CONNECTED)
        rtc.writeSqwPinMode(OFF);
      else
        rtc.writeSqwPinMode(ON);
    }

    //check system status
    if ((millis() - sysCheckPreviousMillis  > 60000 ) )
      checkSystemStatus();

  }

  yield();

  //  ESP.deepSleep(60000, WAKE_RF_DEFAULT);
  // will put the chip into deep sleep. mode is one of
  // WAKE_RF_DEFAULT, WAKE_RFCAL, WAKE_NO_RFCAL, WAKE_RF_DISABLED.

}





void checkSystemStatus( ) {

  sysCheckPreviousMillis = millis();
  webSocket.broadcastTXT("checking");

  File f = SPIFFS.open("/data.bin", "r");
  String  s = String(f.size());

  webSocket.broadcastTXT(s);
  if (f.size() > DATA_FILE_SIZE   ) {
    webSocket.broadcastTXT("diskFull");
    LED1_Blink_Speed = 0x1; //alway on
    sysConfig.logDataON = 0  ;
    sysConfig.diskFull = 1;
    webSocket.broadcastTXT("rec-off");
    saveSysConfig();
  }
  else if (f.size() > DATA_FILE_WARNING_SIZE) {
    LED1_Blink_Speed = 0x40; //fast blanking
    webSocket.broadcastTXT("spaceWarning");
  } else {
    LED1_Blink_Speed = 0x300; //slow blanking
    webSocket.broadcastTXT("spaceOK");
  }
  f.close();
}

void startLongPress() {
  if (digitalRead(SELECT_KEY)) {

    if (sysConfig.logDataON) {
      sysConfig.logDataON = 0  ;
      webSocket.broadcastTXT("rec-off");
    } else {
      sysConfig.logDataON = 1 ;
      webSocket.broadcastTXT("rec-on");
    }

    buffCount = 0;
    saveSysConfig();
    checkSystemStatus();
  }
}


void selectLongPress() {
  if (digitalRead(START_KEY))
    return;
  deleteData();

}


void deleteData() {
  recordStop = true;

  //remove the data file
  SPIFFS.remove("/data.bin");
  //create new data file
  File file = SPIFFS.open("/data.bin", "w");
  if (file) {
    file.close();
  }
  // blinking LED
  for (uint8_t i = 0 ; i < 20 ; i++) {
    digitalWrite(LED1,  i & 0x1 );
    rtc.writeSqwPinMode( modes[i & 0x1]);
    delay(50);
  }
  sysConfig.diskFull = 0;
  LED1_Blink_Speed = 0x300;

  checkSystemStatus();
  // resume LED status
  if (wsConnection != 0)
    rtc.writeSqwPinMode(SquareWave1HZ);
  delay(100);
  recordStop = false;
}


void saveSysConfig() {

  File sys = SPIFFS.open("/config.sys", "w");
  if (sys) {
    sys.write((uint8_t*) &sysConfig, sizeof(sysConfig));
    sys.close();
  }
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t lenght) {

  switch (type) {
    case WStype_DISCONNECTED:
      USE_SERIAL.printf("[%u] Disconnected!\n", num);

      wsConnection--;
      if (wsConnection == 0)
        rtc.writeSqwPinMode(ON);     //no websocket connection off the LED
      break;

    case WStype_CONNECTED:
      {
        rtc.writeSqwPinMode(SquareWave1HZ);
        wsConnection++;
        // IPAddress ip = webSocket.remoteIP(num);
        // USE_SERIAL.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);

        // send message to client
        // webSocket.sendTXT(num, "Connected");

        //        File f = SPIFFS.open("/data.bin", "r");
        //
        //        if (f) {
        //          // f.seek( -(sizeof(logData) / sizeof(uint8_t)) , SeekEnd);
        //
        //          size_t pkgSize = sizeof(logData) * 100;
        //          uint16_t cycle = f.size() / pkgSize;
        //          byte b[pkgSize];
        //
        //          for (uint16_t j = 0 ; j < cycle; j++) {
        //            for (uint16_t i = 0 ; i < pkgSize; i++) {
        //              b[i] = f.read();
        //            }
        //            webSocket.sendBIN(num, b, pkgSize);
        //            yield();
        //          }
        //          f.close();
        //        }

      }
      break;
    case WStype_TEXT:
      USE_SERIAL.printf("[%u] get Text: %s\n", num, payload);

      // send message to client
      // webSocket.sendTXT(num, "message here");

      // send data to all connected clients
      // webSocket.broadcastTXT("message here");

      break;


    case WStype_BIN:
      USE_SERIAL.printf("[%u] get binary lenght: %u\n", num, lenght);
      hexdump(payload, lenght);

      // send message to client
      // webSocket.sendBIN(num, payload, lenght);
      break;
  }

}

void write_csv() {

  if (!SPIFFS.exists("/data.csv")) {
    File file = SPIFFS.open("/data.csv", "w+");
    file.print("Date,NTC,MAX6675,DHT12,Humidity" );
    file.print("\r");
    file.close();
  }

  File file = SPIFFS.open("/data.csv", "a+");
  if (file) {

    DateTime now = rtc.now();
    yield();
    file.print(now.year(), DEC);
    file.print('/');
    file.print(now.month(), DEC);
    file.print('/');
    file.print(now.day(), DEC);
    // file.print(" (");
    // file.print(daysOfTheWeek[now.dayOfTheWeek()]);
    // file.print(") ");
    file.print(" ");
    file.print(now.hour(), DEC);
    file.print(':');
    file.print(now.minute(), DEC);
    file.print(':');
    file.print(now.second(), DEC);
    file.print(",");

    //      file.print(" since midnight 1/1/1970 = ");
    //      file.print(now.unixtime());
    //      file.print("s = ");
    //      file.print(now.unixtime() / 86400L);
    //      file.print(",");

    file.print(",");
    //file.print(logData.MAX6675);
    file.print(",");
    //  file.print(logData.DHT12_temp );
    file.print(",");
    //  file.print(logData.DHT12_hum );
    file.print("\r");
    file.close();
  }

}
