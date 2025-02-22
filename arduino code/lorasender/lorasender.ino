#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include "HT_SSD1306Wire.h"
#include <string.h>

#define SCK 5
#define MISO 11
#define MOSI 10
#define SS 8
#define RST 12
#define DIO0 14
#define BAND 915E6
int readingID = 0;
int counter = 0;
String LoRaMessage = "";
float temperature = 0;
float humidity = 0;


SSD1306Wire  display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED); // addr , freq , i2c group , resolution , rst

#define DHTPIN 19
DHT dht(DHTPIN, DHT22);
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
void startLoRA(){
  //SPI LoRa pins
  SPI.begin(SCK, MISO, MOSI, SS);
  //setup LoRa transceiver module
  LoRa.setPins(SS, RST, DIO0);

  while (!LoRa.begin(BAND) && counter < 10) {
    Serial.print(".");
    counter++;
    delay(500);
  }
  if (counter == 10) {
    // Increment readingID on every new reading
    readingID++;
    Serial.println("Starting LoRa failed!"); 
  }
  Serial.println("LoRa Initialization OK!");
  display.clear();
  display.drawString(0, display.getHeight()/2-16/2, "LORA INIT OK");
  display.display();
  delay(2000);
}
void startOLED()
{
  VextON();
  display.init();
  display.clear();
  display.display();
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, display.getHeight()/2-16/2, "INITIALIZING DISPLAY");
  display.display();
  delay(2000);
}
void getReadings()
{
  temperature = dht.readTemperature();  
  humidity = dht.readHumidity();
  char humidityString[20];
  char temperatureString[20];
  dtostrf(humidity, 6, 2, humidityString);
  dtostrf(temperature, 6, 2, temperatureString);
}
void sendReadings()
{
  char humidityString[20];
  char temperatureString[20];
  char readingIDString[20] = readingID;
  dtostrf(humidity, 6, 2, humidityString);
  dtostrf(temperature, 6, 2, temperatureString);
  dtostrf(readingID, 6, 2, readingIDString);
  LoRaMessage = readingIDString + "/" + temperatureString + "&" + humidityString; //create data payload
  LoRa.beginPacket();
  LoRa.print(LoRaMessage);
  LoRa.endPacket();

  display.clear();
  char message[50] = "Reading ID: ";
  strcat(message, readingIDString);
  display.drawString(0, 0, message);
  char humid_message[50] = "Humidity: ");
  strcat(humid_message, humidityString);
  char humid_message[50] = "Humidity: ");
  strcat(humid_message, humidityString);
  display.drawString(82, 0, humidity);
  readingID++;
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin
  startOLED();
  dht.begin();
}

void loop() {
  // put your main code here, to run repeatedly:

}
