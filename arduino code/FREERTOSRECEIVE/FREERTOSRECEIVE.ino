#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <FreeRTOS.h>

#define RF_FREQUENCY                                915000000 // Hz
#define LORA_BANDWIDTH                              0         // [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]
#define LORA_SPREADING_FACTOR                       9         // [SF7..SF12]
#define LORA_CODINGRATE                             4         // [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false
#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 128 // Define the payload size here

QueueHandle_t alertQueue;
QueueHandle_t sensorDataQueue;
bool HIGH_ALERT = false;

static RadioEvents_t RadioEvents;

void setup() {
  Serial.begin(115200);

  // Initialize LoRa
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.RxTimeout = OnRxTimeout;
  RadioEvents.TxDone = OnTxDone;

  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                    0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

  // Create queues
  alertQueue = xQueueCreate(1, sizeof(bool));  // Queue to hold alert status
  sensorDataQueue = xQueueCreate(5, sizeof(float) * 2);  // Queue to hold sensor data

  // Create FreeRTOS tasks
  xTaskCreatePinnedToCore(TaskHandleAlert, "AlertHandler", 2048, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(TaskReadSensors, "ReadSensors", 2048, NULL, 1, NULL, 0);

  Radio.Rx(0);  // Start receiving data
}

void loop() {
  // Main loop is empty as all logic is handled by tasks
  vTaskDelay(portMAX_DELAY);
}

void TaskReadSensors(void *pvParameters) {
  float sensorData[2];  // Temperature and Humidity

  while (1) {
    // Simulate reading from sensor (Replace with actual DHT sensor reading code)
    sensorData[0] = random(20, 30);  // Example temperature
    sensorData[1] = random(30, 60);  // Example humidity

    // Send data to sensor data queue
    xQueueSend(sensorDataQueue, &sensorData, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(2000));  // Read every 2 seconds
  }
}

void TaskHandleAlert(void *pvParameters) {
  while (1) {
    // Handle alert status based on message received from radio
    if (xQueueReceive(alertQueue, &HIGH_ALERT, portMAX_DELAY)) {
      // If HIGH_ALERT is active, change behavior
      if (HIGH_ALERT) {
        Serial.println("HIGH ALERT Mode Activated");
        // Here you can trigger other actions like adjusting frequency, etc.
      } else {
        Serial.println("Normal Mode Activated");
      }
    }
  }
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  char receivedData[BUFFER_SIZE];
  memcpy(receivedData, payload, size);
  receivedData[size] = '\0';  // Null-terminate the string

  // Process received data
  Serial.print("Received Data: ");
  Serial.println(receivedData);

  // If alert message is received, update alert status
  if (strcmp(receivedData, "HIGH_ALERT_TRUE") == 0) {
    HIGH_ALERT = true;
    xQueueSend(alertQueue, &HIGH_ALERT, 0);  // Send to alert queue
    sendAlertStatus("HIGH_ALERT_TRUE");  // Send alert status via LoRa
  } else if (strcmp(receivedData, "HIGH_ALERT_FALSE") == 0) {
    HIGH_ALERT = false;
    xQueueSend(alertQueue, &HIGH_ALERT, 0);  // Send to alert queue
    sendAlertStatus("HIGH_ALERT_FALSE");  // Send alert status via LoRa
  } else {
    // Process sensor data (e.g., temp and humidity)
    float temperature, humidity;
    if (sscanf(receivedData, "Temp: %f Hum: %f", &temperature, &humidity) == 2) {
      Serial.print("Temperature: ");
      Serial.println(temperature);
      Serial.print("Humidity: ");
      Serial.println(humidity);
    } else {
      Serial.println("Failed to parse sensor data.");
    }
  }
}

void sendAlertStatus(const char *alertStatus) {
  // Send alert status message via LoRa
  Radio.Send((uint8_t *)alertStatus, strlen(alertStatus));
}

void OnRxTimeout(void) {
  // LoRa reception timeout
  Serial.println("RX Timeout...");
}

void OnTxDone(void) {
  // LoRa transmission completed
  Serial.println("TX done...");
}
