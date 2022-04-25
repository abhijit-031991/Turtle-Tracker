#include <Arduino.h>
#include <SPIMemory.h>

SPIFlash flash(1);

uint32_t wAdd = 0;                // Write Address Parameter
uint32_t rAdd = 0;   
uint32_t rAddNow;

void read_send(){ 
  char data[] = "";

  if (flash.powerUp())
  {
    if (flash.readAnything(rAdd, data))
    {
      // dat.id = tag;
      Serial.println(F("Read Successful"));
      Serial.println(data);
    }else
    {
      Serial.println(F("Read Failed"));
    }    
  }
}

void setup(){
    Serial.begin(9600);
    Serial.println(F("SYSTEM INIT..."));
  
    // Enable & Start Flash //
  
    if(flash.powerUp()){
        Serial.println(F("Powered Up1"));
    }
    if(!flash.begin()){
        Serial.println(F("Flash again"));
        Serial.println(flash.error(VERBOSE));
    } 
    Serial.println(flash.getManID());
    if(flash.powerUp()){
        Serial.println(F("Powered Up"));
    }else
    {
        Serial.println(F("PWR UP Failed!"));
    }
    rAddNow = flash.getAddress(16);
    Serial.println(rAddNow);
    Serial.println(F("SYSTEM READY"));
    delay(10000);

}

void loop(){
    do
    {
        read_send();
        rAdd = rAdd + 10;
    } while (rAdd <= rAddNow);
    
    
}