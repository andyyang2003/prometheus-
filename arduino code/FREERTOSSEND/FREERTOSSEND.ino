#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "DHT.h"
#include <FreeRTOS.h>

// Define FreeRTOS tasks
void TaskReadSensors(void *pvParameters);
void TaskSendLoRa(void *pvParameters);

float currentTemp;
float currentHumidity;

// DHT sensor settings
#define DHTPIN 26
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

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


QueueHandle_t sensorQueue;
QueueHandle_t alertQueue;

bool HIGH_ALERT = false;
uint32_t sendInterval = pdMS_TO_TICKS(10000);
static RadioEvents_t RadioEvents;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  dht.begin();
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxDone = OnRxDone;

  Radio.Init( &RadioEvents );
  Radio.SetChannel( RF_FREQUENCY );
  Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                  LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                  LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                  true, 0, 0, LORA_IQ_INVERSION_ON, 3000 ); 
  Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                               LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                               LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                               0, true, 0, 0, LORA_IQ_INVERSION_ON, true );
  sensorQueue = xQueueCreate(5, sizeof(float) * 2);
  alertQueue = xQueueCreate(1, sizeof(bool));  // To hold the alert flag
  xTaskCreatePinnedToCore(TaskReadSensors, "ReadSensors", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskSendLoRa, "SendLoRa", 2048, NULL, 1, NULL, 1);

  Radio.Rx(0);
}

void TaskReadSensors(void *pvParameters) {
    float sensorData[2];
    while (1) {
        sensorData[0] = dht.readTemperature();
        sensorData[1] = dht.readHumidity();
        /*
        PM25_AQI_Data data;
        if (aqi.read(&data)) {
            sensorData[2] = data.pm25_standard;
            sensorData[3] = data.pm10_standard;
        } else {
            sensorData[2] = -1; // Error reading PM2.5
            sensorData[3] = -1; // Error reading PM10
        }*/
        
        xQueueSend(sensorQueue, &sensorData, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void TaskSendLoRa(void *pvParameters) {
    float sensorData[2];
    while (1) {
        if (xQueueReceive(sensorQueue, &sensorData, portMAX_DELAY)) {
            char payload[128];
            snprintf(payload, sizeof(payload), "Temp: %.2f Hum: %.2f", 
                     sensorData[0], sensorData[1]);
            Serial.printf("%s", payload);
            
            // Check the current alert status
        if (xQueueReceive(alertQueue, &HIGH_ALERT, 0)) {
          // If HIGH_ALERT is active, set the interval to 30 seconds
          if (HIGH_ALERT) {
            sendInterval = pdMS_TO_TICKS(30000);  // 30 seconds
          } else {
            sendInterval = pdMS_TO_TICKS(60000);  // 1 minute
          }
        }

      // Send the LoRa data
      Radio.Send((uint8_t *)payload, strlen(payload));
      }
      // Wait for the appropriate interval before sending again
      vTaskDelay(sendInterval);
    }
}
void OnTxDone(void) {
  // LoRa transmission completed
  Serial.println("TX done...");
}

void OnTxTimeout(void) {
  // LoRa transmission timeout
  Serial.println("TX Timeout...");
}
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  char receivedData[BUFFER_SIZE];
  memcpy(receivedData, payload, size);
  receivedData[size] = '\0'; // Null-terminate the string

  if (strcmp(receivedData, "HIGH_ALERT_TRUE") == 0) {
    HIGH_ALERT = true;
    Serial.println("HIGH ALERT");
  } else if (strcmp(receivedData, "HIGH_ALERT_FALSE") == 0) {
    HIGH_ALERT = false;
    Serial.println("NORMAL MODE");
  }
}
void loop() {
  // put your main code here, to run repeatedly:  
  vTaskDelay(portMAX_DELAY);
}
