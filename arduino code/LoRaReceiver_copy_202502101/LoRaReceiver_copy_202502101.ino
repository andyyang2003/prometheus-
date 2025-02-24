#include "LoRaWan_APP.h"
#include "Arduino.h"

#include "HT_SSD1306Wire.h"
#include <heltec.h>

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
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
#define SLEEP_TIME_MINUTES 0.5
#define SLEEP_TIME_SECONDS SLEEP_TIME_MINUTES * 60
#define SLEEP_TIME_MICROSECONDS SLEEP_TIME_SECONDS * 1000000LL
#define TXINITNUMBER 15
char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];
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

void setupMQTT() {
  mqttClient.setServer(mqtt_broker, mqtt_port);
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
    lora_idle = false;
    Serial.println("into RX mode");
    mqttClient.publish(topic_publish, receivedLoRaData); //publish
    Radio.Rx(0); // open to reading
  }
  Radio.IrqProcess( );
}
  

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    rssi=rssi;
    rxSize=size;
    memcpy(rxpacket, payload, size );
    rxpacket[size]='\0';
    Radio.Sleep( );
    Serial.printf("\r\nreceived packet \"%s\" with rssi %d , length %d\r\n",rxpacket,rssi,rxSize);
    m_display.clear();
    m_display.display();
    m_display.drawString(0, 0, rxpacket);
    m_display.display();
    strcpy(receivedLoRaData, rxpacket);
    lora_idle = true;
}