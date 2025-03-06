#include "LoRaWan_APP.h"
#include "Arduino.h"

#include "HT_SSD1306Wire.h"
#include <heltec.h>

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

/* display */
SSD1306Wire m_display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);
/* display end */

/* lora/radio */
#define RF_FREQUENCY 915000000
#define TX_OUTPUT_POWER 20
#define LORA_BANDWIDTH 0
#define LORA_SPREADING_FACTOR 9
#define LORA_CODINGRATE 4
#define LORA_PREAMBLE_LENGTH 8
#define LORA_SYMBOL_TIMEOUT 0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define RX_TIMEOUT_VALUE 1000
#define BUFFER_SIZE 60

char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];
static RadioEvents_t RadioEvents;
int16_t rssi, rxSize;
bool lora_idle = true;
/* lora/radio end */

/*wifi / mqtt */
const char *ssid = "weenus";
const char *password = "andyyang";

const char *mqtt_broker = "8a6d3324d5d542f682b67e26bbc1baa4.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;
const char *mqtt_username = "ayang11";
const char *mqtt_password = "gVXI1250KD$";

const char *topic_publish = "esp32/dht22_sensor";

WiFiClientSecure wifiClient;
PubSubClient mqttClient(wifiClient);

volatile bool newLoraData = false;
char receivedLoRaData[BUFFER_SIZE];

void setupMQTT() {
  mqttClient.setServer(mqtt_broker, mqtt_port);
}

void sendCommand(const char *command) {
  Radio.Send((uint8_t *)command, strlen(command));
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

void VextON(void) {
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
}

void setup() {
  Serial.begin(115200);
  Mcu.begin();
  VextON();
  m_display.init();
  m_display.clear();
  m_display.display();

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("Connected to Wi-Fi");

  wifiClient.setInsecure();

  setupMQTT();
  RadioEvents.RxDone = OnRxDone;
  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                    0, true, 0, 0, LORA_IQ_INVERSION_ON, true);
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                    true, 0, 0, LORA_IQ_INVERSION_ON, 3000);
  Radio.Rx(0);
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("wifi not connected");
    m_display.drawString(0, 0, "no wifi");
  }
  if (!mqttClient.connected()) {
    Serial.println("mqtt not connected");
    m_display.drawString(0, 0, "no mqtt");
    reconnect();
  }
  mqttClient.loop();

  if (lora_idle) {
    mqttClient.publish(topic_publish, receivedLoRaData);
    lora_idle = false;
  }
  Radio.IrqProcess();

  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "STOP") {
      sendCommand("STOP");
      Serial.println("Sending STOP command");
    } else if (command == "START") {
      sendCommand("START");
      Serial.println("Sending START command");
    } else if (command == "HIGH_ALERT_TRUE") {
        sendCommand("HIGH_ALERT_TRUE");
        Serial.println("sending HIGH_ALERT_TRUE");
    } else if (command == "HIGH_ALERT_FALSE"){
        sendCommand("HIGH_ALERT_FALSE");
        Serial.println("sending HIGH_ALERT_FALSE");
    }
  }
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  rssi = rssi;
  rxSize = size;
  memcpy(rxpacket, payload, size);
  rxpacket[size] = '\0';
  Radio.Sleep();
  Serial.printf("\r\nreceived packet \"%s\" with rssi %d , length %d\r\n", rxpacket, rssi, rxSize);
  m_display.clear();
  m_display.display();
  m_display.drawString(0, 0, rxpacket);
  m_display.display();
  strcpy(receivedLoRaData, rxpacket);
  lora_idle = true;
  Radio.Rx(0);
}