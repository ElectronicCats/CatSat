/************************************************************
SPANISH
CatSat.ino
CatSat - Satelite en Lata Educativo
Andres Sabas @ Electronic Cats
Eduardo Contreras @ Electronic Cats
Original Creation Date: Jul 10, 2016
https://github.com/ElectronicsCats/CatSat/

Este ejemplos demuestra el funcionamiento basico de los sensores y funcionalidad
basica del CatSat Kit Educativo
http://electroniccats.com

Especificaciones del entorno de Desarrollo:
  IDE: Arduino 1.8.4
  Plataforma de Hardware:
  Kit CanSat
  - Arduino Mini Pro
  - DHT22
  - GY-87
  - GPS L80


Este código es beerware si tu me ves ( o cualquier otro miembro de Electronic Cats) 
a nivel local, y tu has encontrado nuestro código útil ,
por favor comprar una ronda de cervezas!

Distribuido como; no se da ninguna garantía.
************************************************************/

/************************************************************
ENGLISH
CatSat.ino
CatSat - Satelite en Lata Educativo
Andres Sabas @ Electronic Cats
Original Creation Date: Jal 10, 2016
https://github.com/ElectronicsCats/CatSat/

This example demonstrates how to use CatSat

Development environment specifics:
  IDE: Arduino 1.8.4
  Hardware Platform:
  Kit CanSat
  - Arduino Mini Pro
  - DHT22
  - GY-87
  - GPS L80

This code is beerware; if you see me (or any other Electronic Cats 
member) at the local, and you've found our code helpful, 
please buy us a round!

Distributed as-is; no warranty is given.

Library I2CDev and MPU6050
https://github.com/jrowberg/i2cdevlib

Library Arduino LoRa
https://github.com/sandeepmistry/arduino-LoRa
*/

#include <LoRa.h>

#include <Wire.h>
#include <SoftwareSerial.h>
#include <I2Cdev.h>

#include <NMEAGPS.h>

#include <MPU6050.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_BMP085_U.h>
#include <DHT_U.h>

#define DHTPIN 6 // Pin digital para DHT22
#define DHTTYPE DHT22   // DHT 22  (AM2302)

#define RFM95_CS 10 
#define RFM95_RST 9
#define RFM95_INT 2

//Command activation Balloon mode
#define PMTK_SET_NMEA_886_PMTK_FR_MODE  "$PMTK001,886,3*36"

long selectBand(int);

/************************************************************
*    IMPORTANTE CAMBIAR id_node DEPENDIENDO TU CANSAT      *
************************************************************/

String id_node= "A1"; 

/*******************************************************  
 *Selecciona un canal entre 0 y 12 este debe coincidir *
 *con el canal de tu satelite                          *
 *******************************************************/
int channel = 12;   

// Inicializar DHT sensor.
DHT_Unified dht(DHTPIN, DHTTYPE);

MPU6050 accelgyro;
//Variables de Aceleración y Giroscopio
//Accel and Gyro Vars
int16_t ax, ay, az;
int16_t gx, gy, gz;

String Todo; //String a mandar

NMEAGPS gps;
static const int RXPin = 5, TXPin = 6;
static const uint32_t GPSBaud = 9600;
int gps_flag = 0;

SoftwareSerial ss(RXPin, TXPin);
//#define ss Serial

uint32_t delayMS;

/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(1);

Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(2);

unsigned long previousMillis = 0;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 5;

void enviarInfo(String outgoing) {
  LoRa.beginPacket();                   // start packet
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
  Serial.println("Dato enviado");
}


void setup() {
  Serial.begin(115200);
  ss.begin(GPSBaud);
  dht.begin();
  /*
   * Activation Balloon mode: 
   * For high-altitude balloon purpose that the vertical movement will 
   * have more effect on the position calculation
  */
  ss.println(PMTK_SET_NMEA_886_PMTK_FR_MODE);
  
    /*****LoRa init****/
   //Re-write pins CS, reset, y IRQ 
  LoRa.setPins(RFM95_CS, RFM95_RST, RFM95_INT); // CS, reset, int pin

  if (!LoRa.begin(selectBand(channel))) {           // initialize ratio at 915 MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }

  LoRa.setTxPower(17); //Set the max transmition power
  LoRa.setSpreadingFactor(10); //Change the SF to get longer distances

  /******************/
 
  /* Initialise the sensor */
  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP085 ... check your connections */
    Serial.print(F("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!"));
    while(1);
  }
  
  accelgyro.initialize();  /// Initialize MPU

  accelgyro.setI2CMasterModeEnabled(false);
  accelgyro.setI2CBypassEnabled(true) ;
  accelgyro.setSleepEnabled(false);
    
  /* Initialise the sensor */
  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println(F("Ooops, no HMC5883 detected ... Check your wiring!"));
    while(1);
  }
  
  Serial.println(F("CatSat Ready!"));
  
}

void loop() {
  if(millis() > previousMillis + TX_INTERVAL * 1000){
    readSensors();
    if(gps_flag == 1)
    {
      Serial.println(Todo);
      enviarInfo(Todo);
      previousMillis = millis();
      Todo = "";
      gps_flag = 0;   
    }
  }
}

bool readSensors(void){
  Todo += id_node;  //Add id to String 
  Todo += ",";
  sensors_event_t event;

  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
    Todo += 0;
    Todo += ","; 
  }
  else {
    /*Uncomment for debbuger*/
    /*
    Serial.print(F("TemperatureDHT: "));
    Serial.print(event.temperature);
    Serial.println(F(" *C"));
    */
    Todo += event.temperature;
    Todo += ","; 
  }
  
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    //Serial.println(F("Error reading humidity!"));
    Todo += 0;
    Todo += ","; 
  }
  else {
    /*Uncomment for debbuger*/
    /*
    Serial.print(F("HumidityDHT: "));
    Serial.print(event.relative_humidity);
    Serial.println(F("%"));
    */
    Todo += event.relative_humidity;
    Todo += ",";
  }

  bmp.getEvent(&event);
 
  if (event.pressure)
  {
    /* Display atmospheric pressue in hPa */
    /*Uncomment for debbuger*/
    /*
    Serial.print(F("Pressure:    "));
    Serial.print(event.pressure);
    Serial.println(F(" hPa"));
    */
    Todo += event.pressure;
    Todo += ",";    
    
    float temperature;
    bmp.getTemperature(&temperature);
    Todo += temperature;
    Todo += ","; 
    /*Uncomment for debbuger*/
    /*
    Serial.print(F("Temperature: "));
    Serial.print(temperature);
    Serial.println(F(" C"));
    */
  }
  else
  {
    Serial.println(F("Sensor error"));
  }
  
   
  mag.getEvent(&event);
 
  // Display the results (magnetic vector values are in micro-Tesla (uT))
  /*Uncomment for debbuger*/
  /*
  Serial.print(F("Magnetometro:  ")); 
  Serial.print(F("X: ")); Serial.print(event.magnetic.x); Serial.print(F("  "));
  Serial.print(F("Y: ")); Serial.print(event.magnetic.y); Serial.print(F("  "));
  Serial.print(F("Z: ")); Serial.print(event.magnetic.z); Serial.print(F("  "));Serial.println(F("uT"));
  */
  Todo += event.magnetic.x;
  Todo += ",";
  Todo += event.magnetic.y;
  Todo += ",";
  Todo += event.magnetic.z;
  Todo += ",";
  /*Uncomment for debbuger*/
  /*
  Serial.print(F("Acelerometro ")); 
  Serial.print(F("X:")); Serial.print(ax); Serial.print("\t");
  Serial.print(F("Y:")); Serial.print(ay); Serial.print("\t");
  Serial.print(F("Z:")); Serial.print(az); Serial.print("\n");
  Serial.print(F("Giroscopio ")); 
  Serial.print(F("X:")); Serial.print(gx); Serial.print("\t");
  Serial.print(F("X:")); Serial.print(gy); Serial.print("\t");
  Serial.print(F("X:")); Serial.println(gz);Serial.print("\n");
  */
  Todo += ax;
  Todo += ",";
  Todo += ay;
  Todo += ",";
  Todo += az;
  Todo += ",";
  Todo += gx;
  Todo += ",";
  Todo += gy;
  Todo += ",";
  Todo += gz;
  Todo += ","; 
  gpsread();
  }

void gpsread(void){
  
  while ((ss.available() > 0) && (gps_flag == 0))
    {
    gps_fix fix = gps.read();
     Serial.print(F("Location: ")); 
      if (fix.valid.location)
      { 
        Todo += String(fix.latitude(), 6);
        Todo += ",";
        Todo += String(fix.longitude(), 6);
        Todo += "\n";
        Serial.print(fix.latitude(), 6);
        Serial.print(F(","));
        Serial.print(fix.longitude(), 6);
        gps_flag = 1;
      }
      else
      { 
        Todo += "0";
        Todo += ",";
        Todo += "0";
        Todo += "\n";
        Serial.print(F("INVALID"));
        gps_flag = 1;
      }

      Serial.print(F("  Date/Time: "));
      if (fix.valid.date)
      {
        Serial.print(fix.dateTime.month);
        Serial.print(F("/"));
        Serial.print(fix.dateTime.day);
        Serial.print(F("/"));
        Serial.print(fix.dateTime.year);
      }
      else
      {
        Serial.print(F("INVALID"));
      }

      Serial.print(F(""));
      if (fix.valid.date)
      {
        if (fix.dateTime.hours < 10) Serial.print(F("0"));
        Serial.print(fix.dateTime.hours);
        Serial.print(F(":"));
      if (fix.dateTime.minutes < 10) Serial.print(F("0"));
        Serial.print(fix.dateTime.minutes);
        Serial.print(F(":"));
      if (fix.dateTime.seconds < 10) Serial.print(F("0"));
        Serial.print(fix.dateTime.seconds);
        Serial.print(F("."));
      }
      else
      {
        Serial.print(F("INVALID"));
      }

      Serial.println(); 
     }
  
}

long selectBand(int a){    
  switch(a){ 
    case 0:
    return 903080000; //903.08Mhz
  break;
    case 1:
    return 905240000; //905.24
  break;
    case 2:
    return 907400000; //907.40
  break;
    case 3:
    return 909560000; //909.56
  break;
    case 4:
    return 911720000; //911.72
  break;
    case 5:
    return 913880000; //913.88
  break;
    case 6:
    return 916040000; //916.04
  break;
    case 7:
    return 918200000; // 918.20
  break;
    case 8:
    return 920360000; //920.36
  break;
    case 9:
    return 922520000; //922.52
  break;
    case 10:
    return 924680000; //924.68
  break;
    case 11:
    return 926840000; //926.84
  break;
    case 12:
    return 915000000; //915
  break;
  }
}
