#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "HT_SSD1306Wire.h"
#include <heltec.h>
#include <string.h>
#include "DHT.h"
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

#define DISPLAY
#define DEBUG

/* SENSOR PIN DEFINITIONS */
#define DHTPIN 26
#define iled 33
#define dustAnalogOut 7
#define GPS_RX 4
#define GPS_TX -1
#define mqAnalog 6
/* SENSOR PIN DEFINITION END */

/* SENSOR CONSTANTS ---------------------------------------------------- */
float currentTemp;
float currentHumidity;
char humidityString[6];
char temperatureString[6];
DHT dht(DHTPIN, DHT22);

#define COV_RATIO 0.2
#define NO_DUST_VOLTAGE 400
#define SYS_VOLTAGE 3300
#define GASTHRESHOLD 1500
float dustdensity, dustvoltage;
int adcvalue;
unsigned long previousDustMillis = 0;
const long dustInterval = 280;
float dustDensity;

int gasDetected = 0;
/* SENSOR CONSTANTS END ---------------------------------------------------- */

/* radio */
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
#define SLEEP_TIME_MINUTES 0.5
#define SLEEP_TIME_SECONDS SLEEP_TIME_MINUTES * 60
#define SLEEP_TIME_MICROSECONDS SLEEP_TIME_SECONDS * 1000000LL
#define TXINITNUMBER 15
static RadioEvents_t RadioEvents;
void OnTxDone(void);
void OnTxTimeout(void);
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);

char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];
RTC_DATA_ATTR unsigned int txNumber = 0;
/*radio end */

/* display */
SSD1306Wire m_display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);
void VextON(void);
/* display end */

/* gps */
HardwareSerial gpsSerial(2);
TinyGPSPlus gps;
double lat, lon;
/* gps end */
const int BUFFER_SIZE_COLLECTION = 30;
float tempBuffer[BUFFER_SIZE_COLLECTION];
float humBuffer[BUFFER_SIZE_COLLECTION];
float dustBuffer[BUFFER_SIZE_COLLECTION];
int gasBuffer[BUFFER_SIZE_COLLECTION];
int collectionIndex = 0;

volatile bool HIGH_ALERT = false;
volatile bool stopSending = false; // Flag to stop sending

// Function Prototypes
void setup();
void loop();
void readDHT();
void readDustSensor();
void readGPS();
void gasSensor();
int filterData(int m);
void sendData();
void displayData(const char *message);
void VextOn();
void VextOff();
void vextOff();
void averageData();

void setup() {
  Serial.begin(115200);
  Mcu.begin();

#ifdef DISPLAY
  VextON();
  m_display.init();
  m_display.clear();
  m_display.display();
#endif

  pinMode(iled, OUTPUT);
  digitalWrite(iled, LOW);

  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxDone = OnRxDone;

  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                    true, 0, 0, LORA_IQ_INVERSION_ON, 3000);
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                    0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

  dht.begin();
  gpsSerial.begin(9600, SERIAL_8N1, 4, -1);
  pinMode(mqAnalog, INPUT);
  Radio.Rx(0);
}

void loop() {
  readDHT();
  readDustSensor();
  readGPS();
  gasSensor();
  Radio.IrqProcess();

  if (!stopSending) { // Check if sending should be stopped
    switch (HIGH_ALERT) {
      case false:
        tempBuffer[collectionIndex] = currentTemp;
        humBuffer[collectionIndex] = currentHumidity;
        dustBuffer[collectionIndex] = dustdensity;
        gasBuffer[collectionIndex] = gasDetected;
        collectionIndex++;
        Serial.print("collection index: ");
        Serial.println(collectionIndex);
        if (collectionIndex >= BUFFER_SIZE_COLLECTION) {
          averageData();
          collectionIndex = 0;
          sendData();
          //esp_sleep_enable_timer_wakeup(SLEEP_TIME_MICROSECONDS);
          Serial.println("Going to sleep now");
          Serial.flush();
          //esp_deep_sleep_start();
        }
        break;
      case true:
        sendData();
        break;
      default:
        break;
    }
  }
  if (stopSending)
  {
    Serial.println("stop sending");
  }
}

void averageData() {
  float avgTemp = 0;
  float avgHum = 0;
  float avgDust = 0;
  int avgGas = 0;
  for (int i = 0; i < BUFFER_SIZE_COLLECTION; i++) {
    avgTemp += tempBuffer[i];
    avgHum += humBuffer[i];
    avgDust += dustBuffer[i];
    avgGas += gasBuffer[i];
  }
  avgTemp /= BUFFER_SIZE_COLLECTION;
  avgHum /= BUFFER_SIZE_COLLECTION;
  avgDust /= BUFFER_SIZE_COLLECTION;
  avgGas /= BUFFER_SIZE_COLLECTION;

  currentTemp = avgTemp;
  currentHumidity = avgHum;
  dustDensity = avgDust;
  gasDetected = avgGas > (BUFFER_SIZE_COLLECTION / 2);
}

void sendData() {
  txNumber += 1;
  if (txNumber > 99) {
    txNumber = 0;
  }
  int32_t latInt = (int32_t)(lat * 100);
  int32_t lonInt = (int32_t)(lon * 100);
  uint16_t tempInt = (uint16_t)(currentTemp * 100);
  uint16_t humInt = (uint16_t)(currentHumidity * 100);
  uint16_t dustInt = (uint16_t)(dustdensity * 100);
  uint16_t txInt = (uint16_t)txNumber;
  uint8_t gasInt = (uint8_t)gasDetected;
  sprintf(txpacket, "%u %u %u %u %d %d %u %u", txNumber, tempInt, humInt, dustInt, latInt, lonInt, gasInt, HIGH_ALERT);

#ifdef DISPLAY
  Serial.printf("\r\nsending packet \"%s\" , length %d\r\n", txpacket, strlen(txpacket));
  m_display.clear();
  m_display.display();
  m_display.drawString(0, 0, txpacket);
  m_display.display();
#endif
  Radio.Send((uint8_t *)txpacket, strlen(txpacket));
}

int filterData(int m) {
  static int flag = 0;
  static int _buff[10], sum;
  const int _buff_max = 10;
  int i;
  if (flag == 0) {
    flag = 1;
    for (i = 0, sum = 0; i < _buff_max; i++) {
      _buff[i] = m;
      sum += _buff[i];
    }
    return m;
  } else {
    sum -= _buff[0];
    for (i = 0; i < _buff_max - 1; i++) {
      _buff[i] = _buff[i + 1];
    }
    _buff[9] = m;
    sum += _buff[9];
    i = sum / 10.0;
    return i;
  }
}

void gasSensor() {
  int gassensorAnalog = analogRead(mqAnalog);
  if (gassensorAnalog > GASTHRESHOLD) {
    gasDetected = 1;
  } else {
    gasDetected = 0;
  }
}

void readDHT() {
  currentTemp = dht.readTemperature();
  currentHumidity = dht.readHumidity();
  if (isnan(currentTemp) || isnan(currentHumidity)) {
#ifdef DEBUG
    Serial.println(F("Failed to read from DHT"));
#endif
#ifdef DISPLAY
    m_display.clear();
    m_display.display();
    m_display.drawString(m_display.getWidth() / 2 - 4, m_display.getHeight() / 2 - 16 / 2, "FAILED");
    m_display.display();
#endif
    return;
  }
}

void readDustSensor() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousDustMillis >= dustInterval) {
    previousDustMillis = currentMillis;
    digitalWrite(iled, HIGH);
    delayMicroseconds(40);
    adcvalue = analogRead(dustAnalogOut);
    digitalWrite(iled, LOW);
    delayMicroseconds(43);

    adcvalue = filterData(adcvalue);
    dustvoltage = (SYS_VOLTAGE / 4096.0) * adcvalue * 11;
    if (dustvoltage >= NO_DUST_VOLTAGE) {
      dustvoltage -= NO_DUST_VOLTAGE;
      dustdensity = dustvoltage * COV_RATIO;
    } else {
      dustdensity = 0;
    }
  }
}

void VextON(void) {
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
}

void readGPS() {
  static unsigned long prevGPS = 0;
  const unsigned long gpsTimeOut = 50;
  const int maxBytesToRead = 32;
  int bytesRead = 0;
  unsigned long currentTime = millis();
  if (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read()) > 0) {
      lat = gps.location.lat();
      lon = gps.location.lng();
    }
    bytesRead++;
    prevGPS = millis();
  }
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  char receivedData[BUFFER_SIZE];
  memcpy(receivedData, payload, size);
  receivedData[size] = '\0';

  if (strcmp(receivedData, "STOP") == 0) {
    stopSending = true; // Stop sending when "STOP" is received
  } else if (strcmp(receivedData, "START") == 0) {
    stopSending = false; // Start sending when "START" is received.
  } else if (strcmp(receivedData, "HIGH_ALERT_TRUE") == 0) {
      HIGH_ALERT = true;
  } else if (strcmp(receivedData, "HIGH_ALERT_FALSE") == 0) {
      HIGH_ALERT = false;
  }
  Radio.Rx(0);
  Serial.println(stopSending);
}

void OnTxDone(void) {
  Serial.println("TX done......");
}

void OnTxTimeout(void) {
  Radio.Sleep();
  Serial.println("TX Timeout......");
}