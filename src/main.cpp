#include <Arduino.h>
#include <elapsedMillis.h>
#include <LoRa.h>
#include <TinyGPS++.h>
#include <time.h>
#include <TimeLib.h>
#include <EEPROM.h>
#include <SPIMemory.h>
#include <Wire.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <MPR121.h>
#include <SFE_BMP180.h>

///// LIBRARY DECLARATIONS /////
TinyGPSPlus gps;
SPIFlash flash(1);
elapsedMillis mTime;
SoftwareSerial gps_serial(19,20);
SFE_BMP180 bmp;

///// PIN DEFINITIONS /////
#define GPS_PIN A0
#define RTC_PIN 21
const int PROGMEM RINT = 2;
const int PROGMEM AINT = 11;
const int PROGMEM LCS = 4;
const int PROGMEM LRST = 3;
const int PROGMEM LDIO = 0;

///// DEVICE DEFINITIONS /////

const uint16_t tag = 11111;
const uint8_t devType = 107;

///// VARIABLES /////

// Flash Adressess //
uint32_t wAdd = 0;                // Write Address Parameter
uint32_t rAdd = 0;                // Read Address Parameter

// MPR Variables //
const uint8_t MPR121_ADDR = 0x5A;
const uint8_t MPR121_INT = 2;
uint8_t touch_trsh = 40;          // Touch Threshold *** USER CONFIG ***
uint8_t release_trsh = 20;        // Release Threshold *** USER CONFIG ***

// Device Variables //
uint16_t cnt;

// Other Variables //
bool wipe = true;

// GPS Control Variables //
int gpsFrequency = 60;            // GPS Frequency in minutes *** USER CONFIG ***
int gpsTimeout = 60;              // GPS Timesout after 'x' seconds *** USER CONFIG ***
int gpsHdop = 5;

// GPS Storage Variables //
float lat;                        // Storing last known Latitude
float lng;                        // Storign last known Longitude

// Radio Variables //
int radioFrequency = 1;           //Frequency of Pings in minutes *** USER CONFIG ***
int rcv_duration = 5;             // Receive Window Duration in seconds *** USER CONFIG ***

// Other Variables //

bool isrt = false;


//...................................//
//           FUNCTIONS               //
//...................................//

void selfTest(){  

}

void isr(){
  isrt = true;
  noInterrupts();

}

void getMeta(){
  char status;
  double T;
  double P;
  status = bmp.getTemperature(T);
    if (status != 0)
    {
      // Print out the measurement:
      Serial.print(F("temperature: "));
      Serial.print(T,2);
      Serial.println(F(" deg C, "));
      status = bmp.startPressure(1);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);
        status = bmp.getPressure(P,T);
        if (status != 0)
        {
          // Print out the measurement:
          Serial.print(F("absolute pressure: "));
          Serial.print(P,2);
          Serial.println(F(" mb"));
        }
      }
    }
  

}

void activationPing(){

  struct ping{
    uint16_t tag;
    byte request;
  }px1;

  struct resp{
    uint16_t tag;
    byte resp;
  }rs1;

  int x;

  px1.tag = tag;
  px1.request = (byte)73;


  LoRa.idle();
  LoRa.beginPacket();
  LoRa.write((uint8_t*)&px1, sizeof(px1));
  LoRa.endPacket();

  mTime = 0;
  while (mTime < 30000)
  {
    x = LoRa.parsePacket();
    if (x)
    {
      Serial.println(F("Incoming1"));
      Serial.println(x);
    }
    
    if (x == 3)
    {
      while (LoRa.available())
      {
        Serial.println(F("Incoming"));
        LoRa.readBytes((uint8_t*)&rs1, sizeof(rs1));
      }
      break;      
    }   

    
  } 
  LoRa.sleep();

  if (rs1.tag == tag && rs1.resp == (byte)70)
  {
    Serial.print(F("System Initialising"));
    EEPROM.write(1, true);
    Serial.println(EEPROM.read(1));
    Serial.print(F("System Initialising"));
     /// Begin GPS and Acquire Lock ////
    digitalWrite(GPS_PIN, HIGH);
      do{ 
        while (gps_serial.available() > 0)
        {
          if (gps.encode(gps_serial.read()))
          {
            if (!gps.location.isValid())
            {
              Serial.println(F("Not Valid"));
            }else{
              Serial.println(gps.location.isUpdated());
              Serial.print("Location Age:");
              Serial.println(gps.location.age());
              Serial.print("Time Age:");
              Serial.println(gps.time.age());
              Serial.print("Date Age:");
              Serial.println(gps.date.age());
              Serial.print("Satellites:");
              Serial.println(gps.satellites.value());
              Serial.print("HDOP:");
              Serial.println(gps.hdop.hdop());
            }
          }
        }
      }while(!gps.location.isValid());
    if (gps.location.age() < 60000)
    {
      //pack data into struct
      lat = gps.location.lat();
      lng = gps.location.lng();
    }
    if (gps.time.isValid())
    {
      setTime(gps.time.hour(),gps.time.minute(),gps.time.second(),gps.date.day(),gps.date.month(),gps.date.year());
      time_t n = now();
    }
    
    digitalWrite(GPS_PIN, LOW);
    
    wipe = true;

    px1.request = (byte)106;
    px1.tag = tag;
    LoRa.idle();
    LoRa.beginPacket();
    LoRa.write((uint8_t*)&px1, sizeof(px1));
    LoRa.endPacket();
    LoRa.sleep();
  }

  if (rs1.tag == tag && rs1.resp == (byte)71)
  {
    Serial.print(F("System Initialising"));
    EEPROM.write(1, true);
    Serial.println(EEPROM.read(1));
     /// Begin GPS and Acquire Lock ////
    digitalWrite(GPS_PIN, HIGH);
      do{ 
        while (gps_serial.available())
        {
          if (gps.encode(gps_serial.read()))
          {
            if (!gps.location.isValid())
            {
              Serial.println(F("Not Valid"));
            }else{
              Serial.println(gps.location.isUpdated());
              Serial.print("Location Age:");
              Serial.println(gps.location.age());
              Serial.print("Time Age:");
              Serial.println(gps.time.age());
              Serial.print("Date Age:");
              Serial.println(gps.date.age());
              Serial.print("Satellites:");
              Serial.println(gps.satellites.value());
              Serial.print("HDOP:");
              Serial.println(gps.hdop.hdop());
            }
          }
        }
      }while(!gps.location.isValid());
    if (gps.location.age() < 60000)
    {
      //pack data into struct
      lat = gps.location.lat();
      lng = gps.location.lng();
    }
    if (gps.time.isValid())
    {
      setTime(gps.time.hour(),gps.time.minute(),gps.time.second(),gps.date.day(),gps.date.month(),gps.date.year());
      time_t n = now();
    }
    digitalWrite(GPS_PIN, LOW);

    wipe = false;

    px1.request = (byte)105;
    px1.tag = tag;
    LoRa.idle();
    LoRa.beginPacket();
    LoRa.write((uint8_t*)&px1, sizeof(px1));
    LoRa.endPacket();
    LoRa.sleep();

  }

  if (rs1.tag == tag && rs1.resp == (byte)115){
        Serial.print(F("Indefinite Sleep"));
        EEPROM.put(1, false);
        Serial.println(EEPROM.read(1));
        delay(50);
        disablePower(POWER_ADC);
        disablePower(POWER_SERIAL0);
        disablePower(POWER_SERIAL1);
        disablePower(POWER_SPI);
        disablePower(POWER_WIRE);
        sleepMode(SLEEP_POWER_DOWN);
        sleep();
  }

  Serial.println(EEPROM.read(1));

  if (EEPROM.read(1) == false){
    Serial.println(F("SLEEP1"));
    delay(50);
    disablePower(POWER_ADC);
    disablePower(POWER_SERIAL0);
    disablePower(POWER_SERIAL1);
    disablePower(POWER_SPI);
    disablePower(POWER_WIRE);
    sleepMode(SLEEP_POWER_DOWN);
    sleep();
  }else{
    Serial.println(F("Reset"));
  }
    
}

void recGPS(){
  mTime = 0;
  digitalWrite(GPS_PIN, HIGH);
  Serial.println(gpsTimeout*1000);
  while (mTime <= (unsigned)gpsTimeout*1000)
  {
    while (gps_serial.available())
    {
      if (gps.encode(gps_serial.read()))
      {
        if (!gps.location.isValid())
        {
          Serial.println("Acquiring");
        }else{
          Serial.println(gps.location.isUpdated());
          Serial.print("Location Age:");
          Serial.println(gps.location.age());
          Serial.print("Time Age:");
          Serial.println(gps.time.age());
          Serial.print("Date Age:");
          Serial.println(gps.date.age());
          Serial.print("Satellites:");
          Serial.println(gps.satellites.value());
          Serial.print("HDOP:");
          Serial.println(gps.hdop.hdop());
        }       
      }      
    }
    if (gps.hdop.hdop() < (double)gpsHdop && gps.location.age() < 1000)
    {
      break;
    }  
  }   

  digitalWrite(GPS_PIN, LOW);
  struct data{
    uint32_t datetime;
    uint16_t locktime;
    float lat;
    float lng;
    byte hdop;
    bool act;
    }dat;

  if (gps.location.age() < 60000)
  {
    //pack data into struct
    lat = gps.location.lat();
    lng = gps.location.lng();
    dat.lat = gps.location.lat();
    dat.lng = gps.location.lng();
  }else{
    // pack data into struct with lat long = 0
    dat.lat = 0;
    dat.lng = 0;
  }
    setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), gps.date.day(), gps.date.month(), gps.date.year());
    dat.datetime = (uint32_t)now();
    dat.locktime = mTime/1000;
    dat.hdop = gps.hdop.hdop();
    // if(act_mode == true){
    //   dat.act = true;
    // }else{
    //   dat.act = false;
    // }
    Serial.println(dat.datetime);
    Serial.println(dat.lat);
    Serial.println(dat.lng);
    Serial.println(dat.locktime);
    Serial.println(dat.hdop);
    Serial.println(dat.act);


  if (flash.powerUp())
  {
    Serial.println(F("Powered Up"));
    delay(500);
    wAdd = flash.getAddress(sizeof(dat));
    // Serial.println(sizeof(dat));
    Serial.println(wAdd);
    if (flash.writeAnything(wAdd, dat))
    {
      Serial.println(F("Write Successful"));
      cnt = cnt + 1;
    }else
    {
      Serial.println(F("Write Failed"));
      Serial.println(flash.error(VERBOSE));
    }     
  }else
  {
    Serial.println(F("Power Up Failed"));
  }   
  flash.powerDown();

}

void read_send(){ 
  struct data{
    uint32_t datetime;
    uint16_t locktime;
    float lat;
    float lng;
    byte hdop;
    bool act;
    }dat;

  if (flash.powerUp())
  {
    if (flash.readAnything(rAdd, dat))
    {
      // dat.id = tag;
      Serial.println(F("Read Successful"));
      Serial.println(dat.datetime);
      Serial.println(dat.hdop);
      Serial.println(dat.lat);
      Serial.println(dat.lng);
      Serial.println(dat.locktime);
      Serial.println(dat.act);
    }else
    {
      Serial.println(F("Read Failed"));
    }    
  }
      LoRa.idle();
      LoRa.beginPacket();
      LoRa.write((uint8_t*)&dat, sizeof(dat));
      LoRa.endPacket();
      LoRa.sleep();
}

void Ping(float x, float y, uint16_t a, uint16_t c, byte d){

  struct ping{
    uint16_t ta;    
    uint16_t cnt;
    float la;
    float ln;
    uint8_t devtyp;
  }p;
  p.devtyp = d;
  p.ta = a;
  p.la = x;
  p.ln = y;
  p.cnt = c;
  Serial.print("Size"); Serial.println((int)sizeof(p));
  LoRa.idle();
  LoRa.beginPacket();
  LoRa.write((uint8_t*)&p, sizeof(p));
  LoRa.endPacket();
  LoRa.sleep();
  
}

void receive(unsigned int time){
  Serial.println(F("Receiving"));
  LoRa.idle();
  mTime = 0;
  int x;
  do
  {  
    x = LoRa.parsePacket();
    if (x)
    {
      Serial.println(x);
    }
    
    
    if (x == 3)
    { 
      Serial.print(F("int : ")); Serial.println(x);
      struct request{
      uint16_t tag;
      byte request;
      }r;
      while (LoRa.available())
      {
        Serial.println(F("Reading in"));
        LoRa.readBytes((uint8_t*)&r, sizeof(r));
      }
      Serial.println(r.tag);
      Serial.println(r.request);
      if (r.tag == tag && r.request == (byte)82)
      {
        
        do
        {
          Serial.println("Init Stream");
          read_send();
          rAdd = rAdd + 16;
          Serial.println(rAdd);
        
        } while (rAdd <= wAdd);

        delay(1000);
        struct resp{
        uint16_t tag;
        byte res;
        }r;
        r.res = (byte) 68;
        r.tag = tag;

        LoRa.idle();
        LoRa.beginPacket();
        LoRa.write((uint8_t*)&r, sizeof(r));
        LoRa.endPacket();
        LoRa.sleep();
      }            
    }

    if (x == 21)
    {
      struct setttings{
        uint32_t pingTime;
        uint16_t act_trsh;
        uint16_t act_gps_frq;
        uint16_t act_duration;
        uint16_t gpsFrq;
        uint16_t gpsTout;
        uint8_t hdop;
        uint8_t radioFrq;
        uint8_t rcv_dur;
        uint8_t sch_dur;
        uint8_t sch_rpt_dur;
        bool act_enabled;
        bool sch_enabled;
      } __attribute__((__packed__)) set;

      while (LoRa.available())
      {
        Serial.println(F("Incoming Settings"));
        LoRa.readBytes((uint8_t*)&set, sizeof(set));
      }

      // PingAlarmTime = set.pingTime;
      // act_treshold = set.act_trsh;
      // act_gpsFrequency = set.act_gps_frq;
      // act_duration = set.act_duration;
      gpsFrequency = set.gpsFrq;
      gpsTimeout = set.gpsTout;
      gpsHdop = set.hdop;
      radioFrequency = set.radioFrq;
      rcv_duration = set.rcv_dur;
      // sch_duration = set.sch_dur;
      // sch_rpt_duration = set.sch_rpt_dur;
      // activity_enabled = set.act_enabled;
      // scheduled = set.sch_enabled;

      Serial.println(set.pingTime);
      Serial.println(set.act_trsh);
      Serial.println(set.act_gps_frq);
      Serial.println(set.act_duration);
      Serial.println(set.gpsFrq);
      Serial.println(set.gpsTout);
      Serial.println(set.hdop);
      Serial.println(set.radioFrq);
      Serial.println(set.rcv_dur);
      Serial.println(set.sch_dur);
      Serial.println(set.sch_rpt_dur);
      Serial.println(set.act_enabled);
      Serial.println(set.sch_enabled);
      delay(100);

      struct resp{
        uint16_t tag;
        byte res;
        }r;
        r.res = (byte)83;
        r.tag = tag;

        LoRa.idle();
        LoRa.beginPacket();
        LoRa.write((uint8_t*)&r, sizeof(r));
        LoRa.endPacket();
        LoRa.sleep();
    }
  }while(mTime <= time);
  LoRa.sleep();
  delay(50);
}

void setup(){

  enablePower(POWER_ADC);
  enablePower(POWER_SERIAL0);
  enablePower(POWER_SERIAL1);
  enablePower(POWER_SPI);
  enablePower(POWER_WIRE);

  pinMode(RTC_PIN, OUTPUT);
  pinMode(AINT, INPUT);
  pinMode(RINT, INPUT);
  digitalWrite(RINT, LOW);

  Serial.begin(9600);
  gps_serial.begin(9600);
  delay(1000);

  Serial.println(F("SYSTEM INIT..."));

  if (!MPR121.begin(MPR121_ADDR)) {
    Serial.println("error setting up MPR121");
    switch (MPR121.getError()) {
      case NO_ERROR:
        Serial.println("no error");
        break;
      case ADDRESS_UNKNOWN:
        Serial.println("incorrect address");
        break;
      case READBACK_FAIL:
        Serial.println("readback failure");
        break;
      case OVERCURRENT_FLAG:
        Serial.println("overcurrent on REXT pin");
        break;
      case OUT_OF_RANGE:
        Serial.println("electrode out of range");
        break;
      case NOT_INITED:
        Serial.println("not initialised");
        break;
      default:
        Serial.println("unknown error");
        break;
    }
    while (1);
  }
  MPR121.setInterruptPin(MPR121_INT);
  MPR121.setSamplePeriod(SAMPLE_INTERVAL_32MS);

  MPR121.setTouchThreshold(touch_trsh);
  MPR121.setReleaseThreshold(release_trsh);
  MPR121.autoSetElectrodes();

  bmp.begin();

  // Begin LoRa Radio//
  LoRa.setPins(LCS, LRST, LDIO);
  if(!LoRa.begin(867E6)){
    Serial.println(F("LoRa Failed Init"));
  }
  LoRa.setTxPower(20, PA_OUTPUT_PA_BOOST_PIN);
  LoRa.setSpreadingFactor(12);
  // LoRa.setSignalBandwidth(62.5E3);
  LoRa.sleep();

  // Activation Ping //
  // activationPing();

  // Enable & Start Flash //
  
  // if(flash.powerUp()){
  //   Serial.println(F("Powered Up1"));
  // }
  // if(!flash.begin()){
  //   Serial.println(F("Flash again"));
  //   Serial.println(flash.error(VERBOSE));
  // } 
  // Serial.println(flash.getManID());
  // if(flash.powerUp()){
  //   Serial.println(F("Powered Up"));
  // }else
  // {
  //   Serial.println(F("PWR UP Failed!"));
  // }
  // if (wipe == true)
  // {
  //   Serial.println(F("WIPING FLASH"));
  //   if(flash.eraseChip()){
  //   Serial.println(F("Memory Wiped"));  
  //   }else
  //   {
  //     Serial.println(flash.error(VERBOSE));
  //   }
  // }else{
  //   rAdd = flash.getAddress(16);
  // }    
  // if(flash.powerDown()){
  //   Serial.println("Powered Down");
  //   digitalWrite(1, HIGH);
  // }else{
  //   Serial.println(flash.error(VERBOSE));
  // }

  // // Attach Interupt //
  attachInterrupt(digitalPinToInterrupt(RINT), isr, CHANGE);

}

void loop(){
  
}

