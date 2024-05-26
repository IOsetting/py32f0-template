This change is not meant to be merged in at this point. See comments on top of `User/main.c`


Pasting testing master I2C code for ESP32


``` C
// ESP32 I2C Scanner
// Based on code of Nick Gammon  http://www.gammon.com.au/forum/?id=10896
// ESP32 DevKit - Arduino IDE 1.8.5
// Device tested PCF8574 - Use pullup resistors 3K3 ohms !
// PCF8574 Default Freq 100 KHz 

#include <Wire.h>
#include <Arduino.h>
#include <Adafruit_PCF8574.h>

Adafruit_PCF8574 pcf8574;

void Scanner ()
{
  Serial.println ();
  Serial.println ("I2C scanner. Scanning ...");
  uint8_t count = 0;

  Wire.begin();
  for (uint8_t i = 8; i < 120; i++)
  {
    //save start time
    unsigned long startTime = micros();
    Wire.beginTransmission (i);          // Begin I2C transmission Address (i)
    if (Wire.endTransmission () == 0)  // Receive 0 = success (ACK response) 
    {
      Serial.print ("Found address: ");
      Serial.print ("0x");
      Serial.print (i, HEX);     // PCF8574 7 bit address
      Serial.print (" time: ");
      Serial.println (micros() - startTime);
      count++;
    }
  }
  Serial.print ("Found ");      
  Serial.print (count, DEC);        // numbers of devices
  Serial.println (" device(s).");
}

void setup()
{
  Serial.begin (115200);  
  Wire.begin (26, 27);   // sda= GPIO_26 /scl= GPIO_27
  pcf8574.begin(0x50, &Wire);
  Scanner ();
  pinMode(15, INPUT);
}

uint8_t loopCount = 0;
void loop()
{
  unsigned long startTime = micros();
  Serial.print("Loop: ");
  Serial.print(loopCount);

  if (digitalRead(15) == HIGH)
  {
    pcf8574.digitalWriteByte(loopCount);
    Serial.print(" Write: ");
    Serial.print(loopCount);
  }
  else
  {
    uint8_t ret = pcf8574.digitalReadByte();
    Serial.print(" Read: ");
    Serial.print(ret);
  }
  
  Serial.print(" Time: ");
  Serial.println (micros() - startTime);
  delay (1000);
  loopCount++;
}



```