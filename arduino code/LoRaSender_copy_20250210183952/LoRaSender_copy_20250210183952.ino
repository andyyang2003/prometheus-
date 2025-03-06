#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "HT_SSD1306Wire.h"
#include <heltec.h>
#include <string.h>
#include "DHT.h"
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include "FreeRTOS.h"
//#include "esp_rtc_mem.h"

#define DISPLAY
#define DEBUG

/* SENSOR PIN DEFINITIONS */
#define DHTPIN 26
#define iled 33
#define dustAnalogOut 7
#define GPS_RX 4 // NOT USED 
#define GPS_TX -1 //not used to send data out
#define mqAnalog 6
/* SENSOR PIN DEFINITION END */

/* SENSOR CONSTANTS ---------------------------------------------------- */

/* dht */
float currentTemp;
float currentHumidity;
char humidityString[6]; // 100.00%
char temperatureString[6]; // 100.00
DHT dht(DHTPIN, DHT22);
/* dht end*/

/* dust sensors */
#define COV_RATIO 0.2
#define NO_DUST_VOLTAGE 400 //baseline threshold
#define SYS_VOLTAGE 5000 // 3.3V for esp32
#define GASTHRESHOLD 1500
float dustdensity, dustvoltage;
int adcvalue;
unsigned long previousDustMillis = 0;
const long dustInterval = 280; // Time between dust readings (ms)
float dustDensity;
/*dust sensor end */

/*gas sensor*/
int gasDetected = 0; // 0 if no gas, 1 if gas.
/*gas sensor end*/

/* SENSOR CONSTANTS END ---------------------------------------------------- */

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
#define SLEEP_TIME_MINUTES 0.1
#define SLEEP_TIME_SECONDS SLEEP_TIME_MINUTES * 60
#define SLEEP_TIME_MICROSECONDS SLEEP_TIME_SECONDS * 1000000LL
#define TXINITNUMBER 15
static RadioEvents_t RadioEvents;
void OnTxDone( void );
void OnTxTimeout( void );
bool lora_idle=true;
char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];
RTC_DATA_ATTR unsigned int txNumber = 0;
/*radio end */

/* display */
SSD1306Wire  m_display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED); // addr , freq , i2c group , resolution , rst
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
// Function Prototypes

void setup();
void loop();
void readDHT();
void readDustSensor();
void readGPS();
void gasSensor();
int filterData(int m);
void sendLoRaPacket();
void displayData(const char *message);
void VextOn();
void VextOff();
void vextOff();
void onTxDone();
void onRxDone();
void onTxTimeout();
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
  
  //txNumber=0;
  pinMode(iled, OUTPUT);
  digitalWrite(iled, LOW); // internal led for dust sensing off initially

  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  
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
                            

  dht.begin();
  gpsSerial.begin(9600, SERIAL_8N1, 4, -1);
  pinMode(mqAnalog, INPUT);
  Radio.Rx(0);
}

void loop()
{
  readDHT();
  readDustSensor();
  readGPS();
  gasSensor();
  Radio.Rx(0);
  Radio.IrqProcess();
  switch(HIGH_ALERT)
  {
    case false:
      Serial.printf("ALERT STATE: %s ", HIGH_ALERT ? "true" : "false");
      tempBuffer[collectionIndex] = currentTemp;
      humBuffer[collectionIndex] = currentHumidity;
      dustBuffer[collectionIndex] = dustdensity;  
      gasBuffer[collectionIndex] = gasDetected;
      collectionIndex++;
      //Serial.print("collection index: "); Serial.println(collectionIndex);
      if (collectionIndex >= BUFFER_SIZE_COLLECTION)
      {
        averageData();
        collectionIndex = 0;
        sendData();
        delay(1000);
        while (!lora_idle)
        {
          Radio.IrqProcess();
          delay(10);
        }
        esp_sleep_enable_timer_wakeup(SLEEP_TIME_MICROSECONDS);
        Serial.println("Going to sleep now");
        Serial.flush(); // Ensure serial data is sent before sleep
        esp_deep_sleep_start(); 
      }
      break;
    case true:
      Serial.printf("ALERT STATE: %s ", HIGH_ALERT ? "true" : "false");
      sendData();
      delay(1000);
      while (!lora_idle)
      {
        Radio.IrqProcess();
        delay(10);
      }
      break;
  }

}
void averageData()
{
  float avgTemp = 0; 
  float avgHum = 0; 
  float avgDust = 0;
  int avgGas = 0;
  for (int i = 0; i < BUFFER_SIZE_COLLECTION; i++)
  {
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
void sendData()
{
  txNumber += 1;
  if (txNumber > 99)
  {
    txNumber = 0;
  }
  //payload: tx#, temp, hum, dust content, lat, long, gas detect
  /* INTEGER TYPES */
  int32_t latInt = (int32_t)(lat * 100); // Increased precision
  int32_t lonInt = (int32_t)(lon * 100);
  uint16_t tempInt = (uint16_t)(currentTemp * 100);
  uint16_t humInt = (uint16_t)(currentHumidity * 100);
  uint16_t dustInt = (uint16_t)(dustdensity * 100);
  uint16_t txInt = (uint16_t)txNumber;
  uint8_t gasInt = (uint8_t)gasDetected;
  sprintf(txpacket,"%u %u %u %u %d %d %u " ,txNumber, tempInt, humInt, dustInt, gasInt, latInt, lonInt);  //start a package

  #ifdef DISPLAY
  Serial.printf("\r\nsending packet \"%s\" , length %d\r\n", txpacket, strlen(txpacket));
  m_display.clear();
  m_display.display();
  m_display.drawString(0, 0, txpacket);
  m_display.display();
  #endif
  Radio.Send( (uint8_t *)txpacket, strlen(txpacket) );
  lora_idle = false;
}
int filterData(int m) //filters data and takes average
{
  static int flag = 0;
  static int _buff[10], sum;
  const int _buff_max = 10;
  int i;
  if (flag == 0) // on first call when flag 0
  {
    flag = 1;
    for (i = 0, sum = 0; i < _buff_max; i++)
    {
      _buff[i] = m; // fill buffer with initial values
      sum += _buff[i]; // sum all values for average 
    }
    return m; // return collected value
  }
  else
  {
    sum -= _buff[0]; // delete last recorded
    for (i = 0; i < _buff_max - 1; i++)
    {
      _buff[i] = _buff[i + 1]; // shift all values down
    }
    _buff[9] = m; // collect new value
    sum += _buff[9]; // add new value to sum
    i = sum / 10.0; // average all values for smoother values
    return i;
  }
}
void gasSensor() {
  int gassensorAnalog = analogRead(mqAnalog);
  if (gassensorAnalog > GASTHRESHOLD)
  {
    gasDetected = 1;
  }
  else
  {
    gasDetected = 0;
  }
}
void readDHT()
{
  currentTemp = dht.readTemperature();
  currentHumidity = dht.readHumidity(); 
  if (isnan(currentTemp) || isnan(currentHumidity))
  {
    #ifdef DEBUG
    Serial.println(F("Failed to read from DHT"));
    #endif
    #ifdef DISPLAY
    m_display.clear();
    m_display.display();
    m_display.drawString(m_display.getWidth()/2-4, m_display.getHeight()/2-16/2, "FAILED");
    m_display.display();
    #endif
    return;
  }
}
void readDustSensor() { //reads dust sensor
  unsigned long currentMillis = millis(); // get time at function call
  // if time between this call and last call is greater than collection period
  if (currentMillis - previousDustMillis >= dustInterval) {
    previousDustMillis = currentMillis; // set to noq
  // collect dust
  //according to https://files.waveshare.com/upload/0/0a/Dust-Sensor-User-Manual-EN.pdf
  // 1. sensor enables iled
  // 2. controller takes 0.28 ms to start reading from analog (amount of time to reach steady state)
  // 3. 0.04ms sampling period
  // ~ 0.32ms -> 40 microseconds to be safe
    digitalWrite(iled, HIGH);
    delayMicroseconds(40); // keep tiny delay for non blocking but allows sensor
    adcvalue = analogRead(dustAnalogOut);
    digitalWrite(iled, LOW);
    delayMicroseconds(43); 

    adcvalue = filterData(adcvalue); // filter just averages values for smooth collection
    // SYS VOLTAGE DEPENDS ON OUTPUT VOLTAGE - 3.3-5 for ESP32
    // ESP32 ADCVALUE is 12 bit make sure divide sys voltage by 4096
    dustvoltage = (SYS_VOLTAGE / 4096.0) * adcvalue * 11;
    if (dustvoltage >= NO_DUST_VOLTAGE) {
      dustvoltage -= NO_DUST_VOLTAGE;
      dustdensity = dustvoltage * COV_RATIO;
    } else {
      dustdensity = 0;
    }

  }
}

void VextON(void) // powers on the oled for display
{
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext, LOW);
}
void readGPS()
{
  static unsigned long prevGPS = 0;
  const unsigned long gpsTimeOut = 50; //50ms
  const int maxBytesToRead = 32;
  int bytesRead = 0;
  unsigned long currentTime = millis();
  if (gpsSerial.available() > 0)
  {
    if (gps.encode(gpsSerial.read()) > 0)
    {
      lat = gps.location.lat();
      lon = gps.location.lng();
      //Serial.printf("%0.2f %0.2f\n", lat, lon);
    }
    bytesRead++;
    prevGPS = millis();
  }
}
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  char receivedData[BUFFER_SIZE];
  memcpy(receivedData, payload, size);
  receivedData[size] = '\0'; // Null-terminate the string

  if (strcmp(receivedData, "HIGH_ALERT_TRUE") == 0) {
    HIGH_ALERT = true;
  } else if (strcmp(receivedData, "HIGH_ALERT_FALSE") == 0) {
    HIGH_ALERT = false;
  }
  Radio.Rx(0); // Restart listening
  Serial.println(HIGH_ALERT);
}
void OnTxDone( void )
{
	Serial.println("TX done......");
	lora_idle = true;
}

void OnTxTimeout( void )
{
    Radio.Sleep( );
    Serial.println("TX Timeout......");
    lora_idle = true;
}