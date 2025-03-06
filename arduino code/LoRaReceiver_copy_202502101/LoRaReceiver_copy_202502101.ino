#include "LoRaWan_APP.h"
#include "Arduino.h"

#include "HT_SSD1306Wire.h"
#include <heltec.h>

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

#include <sstream>
/* display */
SSD1306Wire  m_display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED); // addr , freq , i2c group , resolution , rst
/* display end */

/* lora/radio */
/* radio */
#define RF_FREQUENCY                                915000000 // Hz
#define TX_OUTPUT_POWER                             20        // dBm 2.23 was 5 turned to 20
#define LORA_BANDWIDTH                              0         // [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]
#define LORA_SPREADING_FACTOR                       9         // [SF7..SF12] was 7 turned to 9
#define LORA_CODINGRATE                             4         // [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false
#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 60 // Define the payload size here
#define SLEEP_TIME_MINUTES 5
#define SLEEP_TIME_SECONDS SLEEP_TIME_MINUTES * 60
#define SLEEP_TIME_MICROSECONDS SLEEP_TIME_SECONDS * 1000000LL
#define TXINITNUMBER 15
#define HISTORY_SIZE 3
#define PACKET_LENGTH 8
#define TEMP_THRESHOLD 500
#define HUM_THRESHOLD 200
#define DUST_THRESHOLD 400
char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];
int historyIndex;
int packetNumbers[HISTORY_SIZE] = {0};
int tempBuffer[HISTORY_SIZE] = {0};
int humBuffer[HISTORY_SIZE] = {0};
int dustBuffer[HISTORY_SIZE] = {0};
int gasDetectedBuffer[HISTORY_SIZE] = {0};
int fire_probability = 0; // Fire detection based on if temp_diff > temp_thresh, hum_diff < hum_thresh, dust_diff > dust_thresh

static RadioEvents_t RadioEvents;
int16_t txNumber;
int16_t rssi,rxSize;
bool lora_idle = true;
/* lora/radio end */

/*wifi / mqtt */
const char* ssid = "weenus";
const char* password = "andyyang";


const char* mqtt_broker = "8a6d3324d5d542f682b67e26bbc1baa4.s1.eu.hivemq.cloud";
const int mqtt_port = 8883; //tls
const char* mqtt_username = "ayang11";
const char* mqtt_password = "gVXI1250KD$";

const char* topic_publish = "esp32/dht22_sensor";

WiFiClientSecure wifiClient;
PubSubClient mqttClient(wifiClient);

// Variables for timing
long previous_time = 0;

volatile bool newLoraData = false;
char receivedLoRaData[BUFFER_SIZE];
void calcDifferences(char* packet)
{
  // Parse incoming packet into an integer array
  int values[PACKET_LENGTH] = {0};
  int index = 0;
  char packetCopy[60];  // Make a copy of packet (since strtok modifies the string)
  strcpy(packetCopy, packet);

  char* token = strtok(packetCopy, " ");
  while (token != NULL && index < PACKET_LENGTH) {
      values[index] = atoi(token);
      token = strtok(NULL, " ");
      index++;
  }

  // Store the new packet in history
  int prevIndex = historyIndex;  // Previous packet index
  historyIndex = (historyIndex + 1) % HISTORY_SIZE; // Circular buffer update
  
  packetNumbers[historyIndex] = values[0];  // Packet #
  tempBuffer[historyIndex] = values[1];     // Temperature
  humBuffer[historyIndex] = values[2];      // Humidity
  dustBuffer[historyIndex] = values[3];     // Dust level
  gasDetectedBuffer[historyIndex] = values[4];
  // If at least two packets exist, compute and print differences
  if (packetNumbers[0] != 0 && packetNumbers[HISTORY_SIZE - 1] != 0) 
  {
    int tempDiff = tempBuffer[HISTORY_SIZE - 1] - tempBuffer[0];
    int humDiff = humBuffer[HISTORY_SIZE - 1] - humBuffer[0];
    int dustDiff = dustBuffer[HISTORY_SIZE - 1] - dustBuffer[0];

    Serial.print("Differences (Packet 10 - Packet 0): ");
    Serial.printf("Temp: %d - %d: ", tempBuffer[HISTORY_SIZE - 1],tempBuffer[0]); Serial.print(tempDiff);
    Serial.print(" Hum: "); Serial.print(humDiff);
    Serial.print(" Dust: "); Serial.print(dustDiff);
    Serial.println();
    bool tempHigh_1 = abs(tempDiff) > 500;
    bool tempHigh_2 = abs(tempDiff) > 1000;
    bool tempHigh_3 = abs(tempDiff) > 2000;

    bool humLow = abs(humDiff) < HUM_THRESHOLD;
    bool dustHigh = abs(dustDiff) > DUST_THRESHOLD;
    // Check if differences exceed thresholds
    if (tempHigh_1) {
      fire_probability += 5;
    }
    else if (tempHigh_2)
    {
      fire_probability += 30;
    }
    else if (tempHigh_3)
    {
      fire_probability += 50;
    }
    if (dustHigh) {
      fire_probability += 2;
    }
    if (humLow) {
      fire_probability += 2;
    }
    if (tempHigh_1 && humLow) {
      fire_probability += 10;  // Extra boost for temp & humidity combined
    }
    if (dustHigh && humLow) {
      fire_probability += 4;  // Another boost
    }
    if (tempHigh_1 && dustHigh && humLow) {
      fire_probability += 20;  // Max alert
    }
    if (fire_probability >= 100)
    {
      sendHighAlert(true);
      fire_probability = 100;
    }
    if (!tempHigh_1 || !tempHigh_2 || !tempHigh_3 || !humLow || !dustHigh)
    {
      fire_probability = max(fire_probability - 20, 0);
    }
  }
}
void setupMQTT() {
  mqttClient.setServer(mqtt_broker, mqtt_port);
}
void sendHighAlert(bool alertState)
{
  if (alertState) { 
    Radio.Send((uint8_t *)"HIGH_ALERT_TRUE", strlen("HIGH_ALERT_TRUE"));
  } else {
    Radio.Send((uint8_t *)"HIGH_ALERT_FALSE", strlen("HIGH_ALERT_FALSE"));
  }
}
void reconnect() {
  Serial.println("Connecting to MQTT Broker...");
  while (!mqttClient.connected()) {
    Serial.println("Reconnecting to MQTT Broker...");
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    
    if (mqttClient.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("Connected to MQTT Broker.");
    } else {
      Serial.print("Failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}
void VextON(void) // powers on the oled
{
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext, LOW);
}
void setup() {
    Serial.begin(115200);
    Mcu.begin();
    VextON();
    m_display.init();
    m_display.clear();
    m_display.display();
    txNumber=0;
    rssi=0;
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("");
    Serial.println("Connected to Wi-Fi");

    // Initialize secure WiFiClient
    wifiClient.setInsecure(); // Use this only for testing, it allows connecting without a root certificate
  
    setupMQTT();
    RadioEvents.RxDone = OnRxDone;
    Radio.Init( &RadioEvents );
    Radio.SetChannel( RF_FREQUENCY );
    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                               LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                               LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                               0, true, 0, 0, LORA_IQ_INVERSION_ON, true );
    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );  
    Radio.Rx(0);                          
}

void loop()
{
  if (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.println("wifi not connected");
      m_display.drawString(0, 0, "no wifi");
  }
  if (!mqttClient.connected()) 
  {
    Serial.println("mqtt not connected");
    m_display.drawString(0, 0, "no mqtt");
    reconnect();
  }
  mqttClient.loop();
  // if not receiving/doing anything, go into doing stuff
  if(lora_idle)
  {
    sendHighAlert(true);
    lora_idle = false;
    Serial.println("into RX mode");
    mqttClient.publish(topic_publish, receivedLoRaData); //publish
  }
  Radio.Rx(0);
  Radio.IrqProcess( );
}
  

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
  rssi=rssi;
  rxSize=size;
  memcpy(rxpacket, payload, size );
  rxpacket[size]='\0';

  Radio.Sleep( );
  Serial.printf("\r\nreceived packet \"%s\" with rssi %d , length %d\r\n", rxpacket, rssi, rxSize);
  m_display.clear();
  m_display.display();
  m_display.drawString(0, 0, rxpacket);
  m_display.display();
  strcpy(receivedLoRaData, rxpacket);

  char fire_probability_string[3];
  itoa(fire_probability, fire_probability_string, 10);
  strncat(receivedLoRaData, fire_probability_string, sizeof(fire_probability_string) - 1);
  calcDifferences(rxpacket);

  lora_idle = true;
  Radio.Rx(0); // open to reading
}