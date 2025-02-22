//https://www.techcoil.com/blog/how-to-show-the-temperature-and-humidity-of-your-room-on-a-heltec-wifi-kit-32-attached-to-a-dht11-dht22-sensor/
#include <heltec.h>
#include "DHT.h"
#include "HT_SSD1306Wire.h"
#include <string.h>

float currentTemp;
float currentHumidity;
int readingID = 0;
#define DHTPIN 19
DHT dht(DHTPIN, DHT22);
SSD1306Wire  display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED); // addr , freq , i2c group , resolution , rst

/*
void display()
{
  String temperatureDisplay = "Temperature: " + (String)currentTemp + " C";
  String humidityDisplay = "Humidity: " + (String)currentHumidity + " %";
  Heltec.display()->clear();
  Heltec.display()->drawString(0, 0, temperatureDisplay);
  Heltec.display()->drawString(0, 12, humidityDisplay);
  Heltec.display()->display();
}
*/
void VextON(void) // powers on the oled
{
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext, LOW);
}

void VextOFF(void) //Vext default OFF
{
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext, HIGH);
}
void displayHumidity(void)
{
  VextON();
  display.init();
  display.clear();
  display.display();
  display.setFont(ArialMT_Plain_16);
  display.drawString(display.getWidth()/2, display.getHeight()/2-16/2, "ROTATE_0");
  display.display();
  delay(2000);
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println(F("DHT22 test!"));
  VextON();
  display.init();
  display.clear();
  display.display();
  display.setFont(ArialMT_Plain_16);
  display.drawString(128, display.getHeight()/2-16/2, "INITIALIZING DISPLAY");
  display.display();
  delay(2000);
  dht.begin();

  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  float temperature = dht.readTemperature();  
  float humidity = dht.readHumidity();
  
  char humidityString[20];
  char temperatureString[20];
  char readingIDString[20];
  dtostrf(humidity, 6, 2, humidityString);
  dtostrf(temperature, 6, 2, temperatureString);
  if (isnan(temperature) || isnan(humidity)) {
    //Serial.println(F("Failed to read from DHT sensor!"));
    display.clear();
    display.display();
    display.drawString(display.getWidth()/2-4, display.getHeight()/2-16/2, "FAILED");
    display.display();
    return;
  }
  
  Serial.print(F("Humidity: "));
  Serial.print(humidity);
  Serial.print("%\n");
  
  display.clear();
  display.display();
  //display.drawString(0, 48, humidityString); //display.getHeight()/2-16/2
  //display.drawString(0, 0, temperatureString);
  char message[100] = "Reading ID: ";
  strcat(message, readingIDString);
  display.drawString(0, 0, message);
  //display.drawString(82, 0, "135");
  display.display();
  delay(5000);
}
