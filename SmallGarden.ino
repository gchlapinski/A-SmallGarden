// development ideas:
// - mix vent cyclical actions
// - cfg file with all const
// - another thingspeak channel to read orders
//   + turn off
//   + reset
//   + watering
// - wifi connect/reconnect/soft reset
// - save error code in eeprom???

#include <avr/pgmspace.h>
//#include <avr/power.h>
#include <avr/wdt.h>

#define gchSer 0
#define gchSDC 0

///////////////////////////////////////////////////////////////// lcd 20x4 with button on (for 10 sec.)
#include <LiquidCrystal_I2C.h> 
LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); 
const byte lcdButton = 7;
boolean backLight = false;
int lightCount = 0;
const int lightCountLimit = 500;
unsigned long lightOn = 0;
const unsigned long lightFreq = 200; 
// one sign = 8 bytes (7x8=56bytes)
const byte charDegree[] PROGMEM = {B01000, B10100, B01000, B00011, B00100, B00100, B00011, B00000};
const byte charAboveLamp[] PROGMEM = {B00100, B01110, B10101, B00100, B00000, B01110, B11111, B00000};
const byte charGround[] PROGMEM = {B00100, B00100, B00100, B10101, B01110, B00100, B00000, B11111};
const byte charUnderLamp[] PROGMEM = {B01110, B11111, B00000, B00100, B01110, B10101, B00100, B00100};
const byte charOut[] PROGMEM = {B10000, B10000, B10010, B10100, B11111, B10100, B10010, B10000};
const byte charHumidity[] PROGMEM = {B10000, B11100, B10100, B10100, B00000, B11110, B10101, B10101};
const byte charHumidityGr[] PROGMEM = {B10000, B11100, B10100, B00000, B11110, B10101, B00000, B11111};
const byte photoLevel[] PROGMEM = {B01110, B10001, B10101, B10001, B01110, B01110, B00100, B00000}; 

///////////////////////////////////////////////////////////////// shift register (changes vs webpage)
const byte latchPin = 12; //Pin connected to ST_CP of 74HC595
const byte clockPin = 11; //Pin connected to SH_CP of 74HC595
const byte dataPin = 13; // Pin connected to DS of 74HC595
byte relayState = 0;

///////////////////////////////////////////////////////////////// relay devices
// 0-3 220V; 4-7 12V
const byte relLight = 0;
const byte relVentOut = 1;
const byte relHeatCable = 2;
//const byte rel??? = 3; // 12V
const byte relVentIn = 7;
const byte relVentMix = 6;
//const byte relPumpR = 5;
//const byte relPumpL = 4;

///////////////////////////////////////////////////////////////// monitor/management variables
const byte startDay = 15; // 15:00
const byte startNight = 6;
const int errorHot = 330; // 32.0st
const int errorCold = 170;
const int errorUp = 500;
const int upHot = 450;
const int tooHot = 295; 
const int almostHot = 260;
const int notSoHot = 230;

unsigned long lastLightOff = 0;
unsigned long lastHeatCableOff = 0;
unsigned long lastVentOut = 0;
unsigned long lastVentIn = 0;
unsigned long lastVentMix = 0;
const unsigned long durVent = 100000; // 100sec
const unsigned long breakVent = 3600000; // 60min
const unsigned long lastOffTime = 60000; // turn on for at least 1min

///////////////////////////////////////////////////////////////// humidity sensors
#include <dht.h>
dht hIn, hOut;
const byte humInPin = 9; // pin for dht22
const byte humOutPin = 8; // pin for dht22
int humIn = 0;
int humOut = 0;

///////////////////////////////////////////////////////////////// ADC values
const byte humLplusPin = 5;
const byte humRplusPin = 6;
const byte humLpin = 17;
const byte humRpin = 16;
const byte photoLpin = 14;
const byte photoRpin = 15;
int humL = 0;
int humR = 0;
int photoL = 0;
int photoR = 0;
boolean leftRight = true;
const unsigned long measureInterval = 3600000;
const unsigned long measureTime = 400000;
unsigned long lastMeasure = 100000;
boolean measuring = false;

///////////////////////////////////////////////////////////////// termometers
#include <OneWire.h> 
#include <DallasTemperature.h>
const byte termPin = 10; // onewire pin for DallasTemp
OneWire termWire(termPin);
DallasTemperature term(&termWire);
//DeviceAddress termG = {0x28, 0xCB, 0xFF, 0x3D, 0x8, 0x0, 0x0, 0x40}; // 28 EE 7B BE 19 16 01 61
//DeviceAddress termU = {0x28, 0xF2, 0xD5, 0x4B, 0x8, 0x0, 0x0, 0xDF}; // 28 EE 7A BF 19 16 01 23
//DeviceAddress termA = {0x28, 0x84, 0xA1, 0x3D, 0x8, 0x0, 0x0, 0x26}; // 28 EE 73 BE 19 16 01 5F
//DeviceAddress termO = {0x28, 0x9F, 0xB6, 0x9F, 0x8, 0x0, 0x0, 0x6}; //  28 EE 67 C0 19 16 01 81

DeviceAddress termG = {0x28, 0xEE, 0x7B, 0xBE, 0x19, 0x16, 0x01, 0x61}; // 28 EE 7B BE 19 16 01 61
DeviceAddress termU = {0x28, 0xEE, 0x7A, 0xBF, 0x19, 0x16, 0x01, 0x23}; // 28 EE 7A BF 19 16 01 23
DeviceAddress termA = {0x28, 0xEE, 0x73, 0xBE, 0x19, 0x16, 0x01, 0x5F}; // 28 EE 73 BE 19 16 01 5F
DeviceAddress termO = {0x28, 0xEE, 0x67, 0xC0, 0x19, 0x16, 0x01, 0x81}; //  28 EE 67 C0 19 16 01 81
int tempG = 0;
int tempU = 0;
int tempA = 0;
int tempO = 0;
byte tempError = 0; // error counter
const byte tempErrorMax = 250; // max error number

///////////////////////////////////////////////////////////////// clock i2c
//#include <Wire.h> // is it needed??? 
#include <RTClib.h>
RTC_DS3231 clock;
byte hClock = 0; 
byte mClock = 0;

///////////////////////////////////////////////////////////////// time management
unsigned long oneMillis = 0;
unsigned long sensorUpd = 0;
const unsigned long sensorUpdFreq = 2000;
boolean getValues = true;

///////////////////////////////////////////////////////////////// WiFi
// orginal arduino uno + esp01 require 3.3v on sd module
// clone arduino + esp03 was tested on 5v on sd module
#if gchSDC
  #include <SPI.h>
  #include <SD.h>

  byte fileNumber = 0;
  byte fileNumberTmp = 0;
  unsigned long newFile = 0; 
  File myFile;  
  char fileNameDEBUG[] = "wifi000.txt";
//#else 
  // turn off spi interface
  //power_spi_disable();
#endif

#include <SoftwareSerial.h> // esp-01
SoftwareSerial esp(2, 3); // pins 2-TX, 3-RX
const byte espRSTpin = 4;
const unsigned long espRSTinterval = 3600000; //3600000; 
unsigned long espRSTlast = 0;
boolean espRSTstatus = true;
boolean espRSTforced = false;
boolean espReseting = false;

unsigned long lastUpdate = 0; // to send data
const unsigned long updateFreq = 120000; // frequency of thinkspeak update

char msg[255]; // read message from esp (is it possible to decrease size)
byte chN = 0; // read char by char to create msg
boolean msgSend = false;
boolean startConn = false;
boolean linked = false;
boolean atError = false;

char prgmBuf[20];
const char channelId PROGMEM = "212263";
const char writeKey[] PROGMEM = "5Q86U4CLZMLU3IUN";
const char readKey[] PROGMEM = "0XAQ8WS7Q9SZG6SH";
// non-functional assumption that esp is already connected (saved pass)
// AT+CWQAP & AT+CWJAP
//const char wfNm[] PROGMEM = "Fafikowo";
//const char wfPs[] PROGMEM = "Bambaryla69";

const byte sensorsNumber = 8; // number of fields
int fieldValue[] = {0, 0, 0, 0, 0, 0, 0, 0}; 

/////////////////////////////////////////////////////////////////////////////////
void setup() {
  // watchdog
  wdt_disable();
  
  // shift register pins
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  // set everything off on relay
  shiftRegister();  

  // termometers
  term.begin();
  term.setResolution(termG, 10);
  term.setResolution(termU, 10);
  term.setResolution(termA, 10);
  term.setResolution(termO, 10);
  term.requestTemperatures();
  
  // clock setup
  clock.begin();
  if (clock.lostPower()) {
    clock.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // power pins for ADC humidity sensors
  digitalWrite(humLplusPin, LOW);
  digitalWrite(humRplusPin, LOW);
  pinMode(humLplusPin, OUTPUT);
  pinMode(humRplusPin, OUTPUT);

  // lcd and button on initialization
  pinMode(lcdButton, INPUT_PULLUP);
  lcdInit();
  lcd.setBacklight(false);

  // esp-01 initialization
  pinMode(espRSTpin, OUTPUT);
  digitalWrite(espRSTpin, HIGH);
  esp.begin(9600);

  // debug part of code
  #if gchSer 
    Serial.begin(9600);
    Serial.print(F("StartUp... FreeRAM: "));
    Serial.println(freeRam());
  //#else
    // turn off usart
    //power_usart0_disable();
  #endif
  
  #if gchSDC 
    pinMode(10, OUTPUT);
    if (!SD.begin(10)) {
      return;
    }
    if (SD.exists(fileNameDEBUG)) {
      SD.remove(fileNameDEBUG);
      //delay(300);
    }
    myFile = SD.open(fileNameDEBUG, FILE_WRITE);
    myFile.print(F("StartUp... FreeRAM: "));
    myFile.println(freeRam());
    myFile.close();
  #endif

  wdt_enable(WDTO_4S);
  delay(1500);
}

/////////////////////////////////////////////////////////////////////////////////
void loop() {
  oneMillis = millis();

  //if (tempError <= tempErrorMax) {
    wdt_reset();
  //}

  // sesnor/devices management
  if (oneMillis - sensorUpd > sensorUpdFreq) {
    readClock();
    readTermometers();
    readAirHumidity();
    readADC();
    
    sensorUpd += sensorUpdFreq;    
    boxMonitor();
  }

  lcdButtonPress();

  // wifi thingspeak update
  if (espRSTstatus == true) {
    if (atError == false) {
      updateResults();
    } else {
      // resetting/reconnecting procedure
      espFlush();
      startConn = false;
      linked = false;
      atError = false;
      lastUpdate += 30000;
    }
  }
  // wifi reset (once per 1h)
  if (oneMillis - espRSTlast > espRSTinterval && startConn == false || espRSTforced == true) {
    espRSTlast = oneMillis;
    espRSTforced = false;
    espRSTstatus = false;
    espReseting = true;
    digitalWrite(espRSTpin, LOW);
    delay(1500);  
    digitalWrite(espRSTpin, HIGH);
    
    #if gchSer
      Serial.println(F("RST"));
    #endif
  }

  #if gchSer
    if (espReseting == true) {
      while (esp.available() > 0) {
        char gc69 = esp.read();
        Serial.print(gc69);
      }
    }
  #endif
  
  if (oneMillis - espRSTlast > 30000 && espRSTstatus == false) {
    espReseting = false;
    espRSTstatus = true;
    espFlush();
    startConn = false;
    linked = false;
    atError = false;
    lastUpdate = oneMillis;
  }

  #if gchSDC  
    if (oneMillis - newFile > 3600000 && linked == false) {
      fileNumber++;
      fileNumberTmp = fileNumber;

      if (fileNumberTmp >= 100) {
        fileNameDEBUG[4] = fileNumberTmp/100 + 48;
        fileNumberTmp = fileNumberTmp - (fileNumberTmp/100)*100;
      } else {
        fileNameDEBUG[4] = '0';
      }
      if (fileNumberTmp >= 10) {
        fileNameDEBUG[5] = fileNumberTmp/10 + 48;
        fileNumberTmp = fileNumberTmp - (fileNumberTmp/10)*10;
      } else {
        fileNameDEBUG[5] = '0';
      }
      fileNameDEBUG[6] = fileNumberTmp + 48;
         
      if (SD.exists(fileNameDEBUG)) {
        SD.remove(fileNameDEBUG);
        //delay(300); // is it really necessary???
      }
      myFile = SD.open(fileNameDEBUG, FILE_WRITE);
      myFile.close();
      newFile = oneMillis;
    }
  #endif
}

//////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////// wifi
//////////////////////////////////////////////////////////////////////////
void updateResults() {
  float value = -10.0;

  if (oneMillis - lastUpdate > updateFreq) {
    if (getValues == true) {
      fieldValue[0] = tempG;
      fieldValue[1] = tempU;
      fieldValue[2] = tempA;
      fieldValue[3] = tempO;
      fieldValue[4] = humIn;
      fieldValue[5] = humOut;
      if (leftRight == true) {
        fieldValue[6] = photoL;
        fieldValue[7] = photoR;
        leftRight = false;
      } else {
        fieldValue[6] = -humL;
        fieldValue[7] = -humR;
        leftRight = true;
      }

      getValues = false;
    }
    
    atCommand(fieldValue, sensorsNumber, true);
    readData();
    value = decodeData();
    // thingspeak approved
    if (value > 0.0) {
      espFlush();
      lastUpdate += updateFreq;
      startConn = false;
      linked = false;
      getValues = true;
    }
    // thingspeak denied 
    if (value == 0.0) {
      espFlush();
      startConn = false;
      linked = false;
      lastUpdate += 15000; // is it enough?? plausible error
    }   
  }
  // some troubles??
  // AT+RST is needed!!
  if (oneMillis - lastUpdate > 3*updateFreq) {
    espRSTstatus = false;
    espRSTforced = true;
  }
}   

// retaining functionality of getting value from thing speak
// updateField=false => fieldNo must be atomic
void atCommand(int value[], byte fieldNo, boolean updateField) {
  if (startConn == false) {
    esp.println(F("AT+CIPSTART=\"TCP\",\"184.106.153.149\",80")); // to progmem candidate IP address
    startConn = true;
  }  
  
  if (startConn == true && linked == true) {
    char atCmd[150];    
    atCmd[0] = '\0';
    
    if (updateField == true) {
      char lengthAT[] = "AT+CIPSEND=   ";
      char val[6], field[] = "&fieldX=";
      byte lenAT = 0;
      // initialize AT command
      strcat(atCmd, strcpy_P(prgmBuf, PSTR("GET /update?api_key=")));
      strcat(atCmd, strcpy_P(prgmBuf, writeKey));
      // adding fields
      for (byte i=0; i<fieldNo; i++) {
        field[6] = i + 49; // byte2char + 1

        if (value[i] < 0) {
          if (value[i] <= -100) {
            lenAT = 5 ;
          } else {
            lenAT = 4;
          }
        } else {
          if (value[i] < 100) {
            lenAT = 3;
          } else {
            lenAT = 4;
          }      
        }

        dtostrf(value[i]*0.1, lenAT, 1, val);     
        
        strcat(atCmd, field);
        strcat(atCmd, val);
      }
      // length of the command strlen(atCmd)
      lenAT = strlen(atCmd) + 2;
      if (lenAT < 100) {
        dtostrf(lenAT, 2, 0, val);

        lengthAT[11] = val[0];
        lengthAT[12] = val[1];
        lengthAT[13] = '\0';
      } else {
        dtostrf(lenAT, 3, 0, val);
        lengthAT[11] = val[0];
        lengthAT[12] = val[1];
        lengthAT[13] = val[2];
      }      
      esp.println(lengthAT);
      linked = false;
    } else {
      esp.println(F("AT+CIPSEND=61"));
      linked = false;

      char field[2];

      field[0] = fieldNo + 48;
      field[1] = '\0';
      strcat(atCmd, strcpy_P(prgmBuf, PSTR("GET /channels/")));
      strcat(atCmd, strcpy_P(prgmBuf, channelId));
      strcat(atCmd, strcpy_P(prgmBuf, PSTR("/fields/")));
      strcat(atCmd, field);
      strcat(atCmd, strcpy_P(prgmBuf, PSTR("/last?api_key=")));
      strcat(atCmd, strcpy_P(prgmBuf, readKey));
    }
    #if gchSer
      Serial.println(atCmd);
    #endif

    #if gchSDC
      myFile = SD.open(fileNameDEBUG, FILE_WRITE);
      myFile.println(atCmd);
      myFile.close();
    #endif
    
    esp.println(atCmd);
  } 
}

float decodeData() {  
  float valRec = -10.0;
  
  if (msgSend == true) {
    char buf[8];
    byte i, nLen, n = 0;
    boolean ipdFound = false;

    for (i=0; i<(chN - 6); i++) {
      if (msg[i] == '+' && msg[i+1] == 'I' && msg[i+2] == 'P' && msg[i+3] =='D') {
        ipdFound = true;
        break;  
      } // "+IPD,2:13.65"

      if (startConn == true && linked == false &&
          (msg[i] == 'A' && msg[i+1] == 'L' && msg[i+2] == 'R' 
           && msg[i+3] =='E' && msg[i+4] == 'A' && msg[i+5] == 'Y'
           || msg[i] == 'C' && msg[i+1] == 'O' && msg[i+2] == 'N' 
           && msg[i+3] =='N' && msg[i+4] == 'E' && msg[i+5] == 'C'
           || msg[i] == 'L' && msg[i+1] == 'i' && msg[i+2] == 'n' 
           && msg[i+3] =='k' && msg[i+4] == 'e' && msg[i+5] == 'd')) {
        linked = true;
        break;  
      } // "ALREAY" | "CONNEC(T)" | "Linked"
      if (msg[i] == 'E' && msg[i+1] == 'R' && msg[i+2] == 'R' 
           && msg[i+3] =='O' && msg[i+4] == 'R') {
        atError = true;
        break;       
      } // "ERROR"
    }

    if (ipdFound == true) {
      i = i + 5;
      while (msg[i] != ':') {
        buf[n++] = msg[i];
        i++;
      }
      buf[n] = '\0';
      nLen = (byte)atoi(buf);

      i++;
      for (n=0; n<nLen; n++) {
        buf[n] = msg[i++];
      }
      buf[n] = '\0';
      valRec = atof(buf);
    } 
    
    chN = 0;
    msgSend = false;
  } 

  return valRec;
}

void readData() {
  const int timeout = 1000;  
  static boolean msgRec = false; 
  static unsigned long lastRec = 0;
  char ch;
  
  while (esp.available() > 0 && msgSend == false) {
    ch = esp.read();
    
    #if gchSer
      Serial.write(ch);
    #endif

    if (msgRec == true) {
      if (millis() - lastRec < timeout) {
        lastRec = millis();
        msg[chN++] = ch;
        if (chN == 0) { // byte type
          chN = 255;
        }        
      } else {
        msg[chN] = '\0';
        msgRec = false;
        msgSend = true;
        
        #if gchSDC
           myFile = SD.open(fileNameDEBUG, FILE_WRITE);
           myFile.println(msg);
           myFile.close();
        #endif      
      }
    } else {
      msgRec = true;
      msg[chN++] = ch;
      lastRec = millis();
    }
  }

  if (esp.available() == 0 && msgSend == false && msgRec == true 
    && millis() - lastRec > timeout) {
    msg[chN] = '\0';
    msgRec = false;
    msgSend = true;  

    #if gchSDC
      myFile = SD.open(fileNameDEBUG, FILE_WRITE);
      myFile.println(msg);
      myFile.close();
    #endif      
  }
}

void espFlush() {
  while (esp.available() > 0) {
    esp.read();
  }
}

//////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////// monitor
//////////////////////////////////////////////////////////////////////////
void boxMonitor() {
  byte relayStateTmp = 0;
  relayStateTmp = relayState;
  
  //////////////////////////////////////////////////////////// light
  // if turn off, it waits at least one minute to turn on
  if (startDay < startNight) {
    if (((relayState & _BV(relLight)) == 0)
        && hClock >= startDay && hClock < startNight   // day time 
        && tempU < errorHot                            // heat protection (under lamp)
        && tempA < errorUp                             // heat protection (above lamp)
        && oneMillis - lastLightOff > lastOffTime) {   // quick turn on/off protection
      relayState |= _BV(relLight);
    } else if (((relayState & _BV(relLight)) == _BV(relLight)) 
               && (hClock < startDay || hClock >= startNight
                  || tempU >= errorHot + 10
                  || tempA >= errorUp + 15)) {
      relayState &= ~_BV(relLight);
      lastLightOff = oneMillis;
    }
  } else {
    if (((relayState & _BV(relLight)) == 0)
        && (hClock >= startDay || hClock < startNight) // day time (day at night)
        && tempU < errorHot                            // heat protection 
        && oneMillis - lastLightOff > lastOffTime) {   // quick turn on/off protection
      relayState |= _BV(relLight);
    } else if (((relayState & _BV(relLight)) == _BV(relLight)) 
               && (hClock < startDay && hClock >= startNight 
                  || tempU >= errorHot + 10
                  || tempA >= errorUp + 15)) {
      relayState &= ~_BV(relLight);
      lastLightOff = oneMillis;
    }
  }

  //////////////////////////////////////////////////////////// heat cable
  // if turn off, it waits at least one minute to turn on
  if (((relayState & _BV(relHeatCable)) == 0)
      && tempG < errorCold                             // cold protection
      && oneMillis - lastHeatCableOff > lastOffTime) { // quick turn on/off protection
    relayState |= _BV(relHeatCable);
  } else if (((relayState & _BV(relHeatCable)) == _BV(relHeatCable))   // ????????????????????????????  
             && (tempG + tempU) >= 2*(errorCold + 15)) {             // histereza = 1.5st  
    relayState &= ~_BV(relHeatCable);
    lastHeatCableOff = oneMillis;
  }

  //////////////////////////////////////////////////////////// vent out
  // It turns off after 3min when meets conditions
  if (((relayState & _BV(relVentOut)) == 0)
      && (tempU > tooHot                               // heat warning (under lamp) 
          || tempA > upHot                             // heat warning (over lamp)
          || oneMillis - lastVentOut > breakVent)) {   // cyclical turn on/off
    relayState |= _BV(relVentOut);
    lastVentOut = oneMillis;                           // moving time
  } else if (((relayState & _BV(relVentOut)) == _BV(relVentOut))
             && oneMillis - lastVentOut > durVent
             && tempU <= tooHot - 15
             && tempA <= upHot - 20) {   // quick turn on/off protection
    relayState &= ~_BV(relVentOut);
  }

  //////////////////////////////////////////////////////////// vent in
  // It turns off after 3min when meets conditions
  if (((relayState & _BV(relVentIn)) == 0)
      && tempU > tempO                                 // check temp outside
      && (tempU > tooHot
          || (tempU > almostHot && tempG > notSoHot)   // cooling
          || humIn < humOut - 50                       // humidity control - diff 5%
          && tempU > notSoHot                          // if it is not cold
          && oneMillis - lastVentIn > breakVent)) {    // quick turn on/off protection
    relayState |= _BV(relVentIn);
    lastVentIn = oneMillis;
  } else if (((relayState & _BV(relVentIn)) == _BV(relVentIn))
             && oneMillis - lastVentIn > durVent
             && (tempU <= tempO
                 || tempU <= almostHot - 10 
                 || (tempU <= tooHot - 20
                     && tempG <= notSoHot - 10))) {    // quick trun on off protection
    relayState &= ~_BV(relVentIn);
  }

  //////////////////////////////////////////////////////////// vent mix
  // It turns off after 1.5min when meets conditions
  if (((relayState & _BV(relVentMix)) == 0)
      && tempA - 50 > tempU                            // use heat from above the lamp 
      && tempU < notSoHot) {                           // if it is cold  
    relayState |= _BV(relVentMix);
    lastVentMix = oneMillis;
  } else if (((relayState & _BV(relVentMix)) == _BV(relVentMix))
             && oneMillis - lastVentMix > durVent
             && tempU >= notSoHot + 15) { // quick turn on/off protection
    relayState &= ~_BV(relVentMix);
  }
  
  /////////////////////////////////////////////////////////// change
  if (relayStateTmp != relayState) {
    shiftRegister();
  }
}

void shiftRegister() {
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, MSBFIRST, relayState);    
  digitalWrite(latchPin, HIGH);
}

/////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////// read sensors
/////////////////////////////////////////////////////////////////////////////
void readADC() {
  if (oneMillis - lastMeasure > measureInterval && measuring == false) {
    digitalWrite(humLplusPin, HIGH);
    digitalWrite(humRplusPin, HIGH);
    lastMeasure = oneMillis;
    measuring = true;
  }  
  if (oneMillis - lastMeasure < measureTime && measuring == true) {
    humL = (humL + analogRead(humLpin)*0.97751)/2; 
    humR = (humR + analogRead(humRpin)*0.97751)/2; 
  }
  if (oneMillis - lastMeasure >= measureTime && measuring == true) {
    digitalWrite(humLplusPin, LOW);
    digitalWrite(humRplusPin, LOW);
    measuring = false;
  }

  photoL = analogRead(photoLpin)*0.97751; 
  photoR = analogRead(photoRpin)*0.97751;
}

void readTermometers() {
  int tempTmp = 0;

  tempTmp = floor(10*term.getTempC(termG));
  if (tempTmp > -1200 && tempTmp < 800) {
    tempG = tempTmp;
  } else {
    tempError++; 
  }
  tempTmp = floor(10*term.getTempC(termU));  
  if (tempTmp > -1200 && tempTmp < 800) {
    tempU = tempTmp;
  } else {
    tempError++; 
  } 
  tempTmp = floor(10*term.getTempC(termA));  
  if (tempTmp > -1200 && tempTmp < 800 && tempA != tempTmp) {
    tempA = tempTmp;
    tempError = 0;
  } else {
    tempError++; 
  } 
  tempTmp = floor(10*term.getTempC(termO));  
  if (tempTmp > -1200 && tempTmp < 800) {
    tempO = tempTmp;
  } else {
    tempError++; 
  } 

  term.requestTemperatures();
}

void readAirHumidity() {
  int chk=0;
  
  chk = hIn.read(humInPin);
  if (chk == DHTLIB_OK) {
    humIn = floor(10*hIn.humidity);    
  }
  
  chk = hOut.read(humOutPin);
  if (chk == DHTLIB_OK) {
    humOut = floor(10*hOut.humidity);    
  }  
}

void readClock() {
  DateTime dt = clock.now();
  
  hClock = dt.hour();
  mClock = dt.minute();
}

////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////// lcd fun
////////////////////////////////////////////////////////////////////////
void showLcd() {
  lcdValue(1, 0, tempA);
  lcdValue(1, 1, tempU);
  lcdValue(1, 2, tempG);
  lcdValue(1, 3, tempO);

  lcdValue(7, 0, humIn);
  lcdValue(7, 1, humOut);
  lcdValue(7, 2, humL);
  lcdValue(7, 3, humR);

  lcdValue(13, 0, photoL);
  lcdValue(13, 1, photoR);

  showClock(13, 2);
  showRelay(12, 3); 
}

void lcdConst() {
  lcdChar(5, 0, 0);
  lcdChar(5, 1, 0);
  lcdChar(5, 2, 0);
  lcdChar(5, 3, 0);
  lcdChar(0, 0, 1);
  lcdChar(0, 1, 3);
  lcdChar(0, 2, 2);
  lcdChar(0, 3, 4);
  lcdChar(6, 0, 5);
  lcdChar(6, 1, 5);
  lcdChar(6, 2, 6);
  lcdChar(6, 3, 6);
  lcdChar(11, 0, 73);
  lcdChar(11, 1, 79);
  lcdChar(11, 2, 76);
  lcdChar(11, 3, 82);
  lcdChar(12, 0, 7);
  lcdChar(12, 1, 7);
  lcdChar(17, 0, 76);
  lcdChar(17, 1, 82);
  lcdChar(19, 0, 103);
  lcdChar(19, 1, 243);
}

void lcdChar(byte x, byte y, byte c) {
  lcd.setCursor(x, y);
  lcd.write(c);
}

char dispVal[5]; // variable for lcd purpose 
void displayValue(int v) {
  if (v < 1000) {
    dtostrf(v*0.1, 4, 1, dispVal);
  } else {
    dispVal[0] = '9';
    dispVal[1] = '9';
    dispVal[2] = '.';
    dispVal[3] = '9';
    dispVal[4] = '\0';
  }
}

void lcdValue(byte x, byte y, int v) {
  lcd.setCursor(x, y);
  displayValue(v);
  lcd.print(dispVal);
}

void showClock(byte x, byte y) {
  lcd.setCursor(x, y);
  if (hClock < 10) {
    lcd.print(F(" "));
  }
  lcd.print(hClock);
  lcd.print(F("h"));
  if (mClock < 10) {
    lcd.print(F("0"));
  }
  lcd.print(mClock);
  lcd.print(F("m")); 
}

void showRelay(byte x, byte y) {
  byte relayTmp = relayState;
  lcd.setCursor(x, y);
  //lcd.write((uint8_t)252); // pacman ghost
  //lcd.write((uint8_t)165); // central dot
  //lcd.write((uint8_t)92); // alien
  //lcd.write((uint8_t)219); // big square
  //lcd.write((uint8_t)243); // infinity 
      
  for (int i=7; i>=0; i--) {
    byte st = 1 << i;
    if (relayTmp >= st) {
      lcd.write((uint8_t)219);
      relayTmp -= st;  
    } else {
      lcd.write((uint8_t)165);
    }
  }
}

void lcdButtonPress() {
  if (backLight == false && digitalRead(lcdButton) == LOW) {
    lcdInit();
    backLight = true;
    lightOn = oneMillis;
    lightCount = 0;
    lcdConst();
  }
  if (backLight == true && oneMillis - lightOn > lightFreq) {
    showLcd();
    lightOn += lightFreq;
    lightCount++;
  }
  if (backLight == true && lightCount > lightCountLimit) {
    lcdClear();
    backLight = false;
  }
}

void lcdClear() {
  for (byte line=0; line<4; line++) {
    lcd.setCursor(0, line);
    lcd.print(F("                    ")); 
  }
  lcd.setBacklight(false);
}

void lcdInit() {
  lcd.begin(20, 4); 
  initCustomChar();
}

void initCustomChar() {
  byte charTmp[8];

  for (byte i=0; i<8; i++) {
    charTmp[i] = pgm_read_byte(&charDegree[i]);
  }
  lcd.createChar(0, charTmp);

  for (byte i=0; i<8; i++) {
    charTmp[i] = pgm_read_byte(&charAboveLamp[i]);
  }
  lcd.createChar(1, charTmp);

  for (byte i=0; i<8; i++) {
    charTmp[i] = pgm_read_byte(&charGround[i]);
  }
  lcd.createChar(2, charTmp);

  for (byte i=0; i<8; i++) {
    charTmp[i] = pgm_read_byte(&charUnderLamp[i]);
  }
  lcd.createChar(3, charTmp);

  for (byte i=0; i<8; i++) {
    charTmp[i] = pgm_read_byte(&charOut[i]);
  }
  lcd.createChar(4, charTmp);

  for (byte i=0; i<8; i++) {
    charTmp[i] = pgm_read_byte(&charHumidity[i]);
  }
  lcd.createChar(5, charTmp);

  for (byte i=0; i<8; i++) {
    charTmp[i] = pgm_read_byte(&charHumidityGr[i]);
  }
  lcd.createChar(6, charTmp);

  for (byte i=0; i<8; i++) {
    charTmp[i] = pgm_read_byte(&photoLevel[i]);
  }
  lcd.createChar(7, charTmp);
}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////// utilities
//////////////////////////////////////////////////////////////////////////
// how much I can use...
int freeRam() {
  extern int __heap_start, *__brkval; 
  int v; 
  
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

void shiftRegisterOneDevice(byte p, boolean on) {
  digitalWrite(latchPin, LOW);
  
  if (on == true) {
    relayState |= _BV(p);
  } else {
    relayState &= ~_BV(p);
  }

  shiftOut(dataPin, clockPin, MSBFIRST, relayState);    
  digitalWrite(latchPin, HIGH);
}
