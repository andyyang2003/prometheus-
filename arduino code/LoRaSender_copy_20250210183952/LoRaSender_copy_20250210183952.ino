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

/* PIN DEFINITIONS */
#define DHTPIN 26
#define iled 33
#define dustAnalogOut 7

#define GPS_RX 48 // NOT USED 
#define GPS_TX 4

#define mqAnalog 6

/* radio */
#define RF_FREQUENCY                                915000000 // Hz
#define TX_OUTPUT_POWER                             5        // dBm
#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false
#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 60 // Define the payload size here
static RadioEvents_t RadioEvents;
void OnTxDone( void );
void OnTxTimeout( void );
bool lora_idle=true;
char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];
double txNumber;
/*radio end */

/* display */
SSD1306Wire  m_display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED); // addr , freq , i2c group , resolution , rst
void VextON(void);
/* display end */

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
#define SYS_VOLTAGE 3300 // 3.3V for esp32
float dustdensity, dustvoltage;
int adcvalue;
unsigned long previousDustMillis = 0;
const long dustInterval = 280; // Time between dust readings (ms)
float dustDensity;
/*dust sensor end */

/* gps */
SoftwareSerial gpsSerial(GPS_RX, GPS_TX);
TinyGPSPlus gps;
double lat, lon;
/* gps end */

/*gas sensor*/
int gasDetected = 0; // 0 if no gas, 1 if gas.
/*gas sensor end*/

int Filter(int m) //filters data and takes average
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
  if (gassensorAnalog > 1000)
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
  //dtostrf(currentHumidity, 6, 2, humidityString);
  //dtostrf(currentTemperature, 6, 2, temperatureString);
  
  if (isnan(currentTemp) || isnan(currentHumidity))
  {
    #ifdef DEBUG
    Serial.println(F("Failed to read from DHT"));
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

    adcvalue = Filter(adcvalue); // filter just averages values for smooth collection
    // SYS VOLTAGE DEPENDS ON OUTPUT VOLTAGE - 3.3 for ESP32
    // ESP32 ADCVALUE is 12 bit!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! make sure divide sys voltage by 4096
    dustvoltage = (SYS_VOLTAGE / 4096.0) * adcvalue * 11;
    if (dustvoltage >= NO_DUST_VOLTAGE) {
      dustvoltage -= NO_DUST_VOLTAGE;
      dustdensity = dustvoltage * COV_RATIO;
    } else {
      dustdensity = 0;
    }

    Serial.print("Dust Density: ");
    Serial.println(dustdensity);
  }
}

void VextON(void) // powers on the oled for display
{
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext, LOW);
}

void setup() {
  Serial.begin(115200);
  Mcu.begin();
  // display
  VextON();
  m_display.init();
  m_display.clear();
  m_display.display();
  txNumber=0;
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

  dht.begin();
  gpsSerial.begin(115200);

  pinMode(mqAnalog, INPUT);
}

void loop()
{
  readDHT();
  readDustSensor();
  readGPS();
// packet - temperature, humidity, dust density, gps location
	if(lora_idle == true)
	{
    delay(2000);
		txNumber += 0.01;
		//sprintf(txpacket, "txnum: %0.2f, temp: %0.2f, hum: %0.2f", txNumber, currentTemp, currentHumidity);  //start a package
    sprintf(txpacket,"tx:%0.2f t:%0.2f h:%0.2f d:%0.2f lt:%0.2f ln:%0.2f gas:%d" ,txNumber, currentTemp, currentHumidity, dustdensity, lat, lon, gasDetected);  //start a package

    #ifdef DISPLAY
		Serial.printf("\r\nsending packet \"%s\" , length %d\r\n", txpacket, strlen(txpacket));
    m_display.clear();
    m_display.display();
    m_display.drawString(0, 0, txpacket);
    m_display.display();
    #endif
		Radio.Send( (uint8_t *)txpacket, strlen(txpacket) ); //send the package out	
    lora_idle = false;
	}
  Radio.IrqProcess( ); // interrupt driven
}
void readGPS()
{
  static unsigned long prevGPS = 0;
  const unsigned long gpsTimeOut = 50; //50ms
  const int maxBytesToRead = 32;
  int bytesRead = 0;
  while (gpsSerial.available() > 0 && bytesRead < maxBytesToRead && (millis() - prevGPS) < gpsTimeOut)
  {
    if (gps.encode(gpsSerial.read()))
    {
      if (gps.location.isValid())
      {
        lat = gps.location.lat();
        lon = gps.location.lng();
      }
    }
    bytesRead++;
    prevGPS = millis();
  }
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