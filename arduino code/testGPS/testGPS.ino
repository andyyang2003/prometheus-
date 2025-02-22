#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
TinyGPSPlus gps;
SoftwareSerial ss(4, 3);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  ss.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  while(ss.available() > 0)
  {
    if(gps.encode(ss.read()) > 0)
    {
      
      Serial.print("LAT="); Serial.print(gps.location.lat(), 6);
      Serial.print(F(","));
      Serial.print("LNG="); Serial.println(gps.location.lng(), 6);
      
    }
    /*Serial.print(char(ss.read()));
    Serial.print("LAT="); Serial.print(gps.location.lat(), 6);
    Serial.print("LNG="); Serial.println(gps.location.lng(), 6);
    Serial.println(gps.encode(ss.read()));
    
    if (gps.encode(ss.read()));
    {
    Serial.print("LAT="); Serial.print(gps.location.lat(), 6);
    Serial.print("LNG="); Serial.println(gps.location.lng(), 6);
    }*/
  }
}
