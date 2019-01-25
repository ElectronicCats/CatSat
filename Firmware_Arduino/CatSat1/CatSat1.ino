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
  IDE: Arduino 1.6.8
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
  IDE: Arduino 1.6.8
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

Library TinyGPS++
https://github.com/mikalhart/TinyGPSPlus

Library LoRa Radio
http://www.airspayce.com/mikem/arduino/RadioHead/index.html

************************************************************
*    IMPORTANTE CAMBIAR id_node DEPENDIENDO TU CANSAT      *
************************************************************/
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
#define RF95_FREQ 915.0 //usados creando el objeto

//Command activation Balloon mode
#define PMTK_SET_NMEA_886_PMTK_FR_MODE  "$PMTK001,886,3*36"

float selectBand(int);

/************************************************************
*    IMPORTANTE CAMBIAR id_node DEPENDIENDO TU CANSAT      *
************************************************************/

String id_node= "A1"; 


/*******************************************************  
 *Selecciona un canal entre 0 y 12 este debe coincidir *
 *con el canal de tu satelite                          *
 *******************************************************/
int channel = 12;   

byte msgCount = 0;            // count of outgoing messages


float chann;

DHT_Unified dht(DHTPIN, DHTTYPE);

MPU6050 accelgyro;
// #### Variables de Aceleración y Giroscopio
// #### Accel and Gyro Vars
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
  /* Get a new sensor event */ 
  Todo += id_node;  //Add id to String 
  Todo += ",";
  sensors_event_t event;
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Todo += 0;
    Todo += ","; 
  }
  else {
    Todo += event.relative_humidity;
    Todo += ",";
  }

  bmp.getEvent(&event);
 
  if (event.pressure)
  {
    Todo += event.pressure;
    Todo += ",";    
     
    float temperature;
    bmp.getTemperature(&temperature);
    Todo += temperature;
    Todo += ","; 
  }
  else
  {
    Serial.println(F("Sensor error"));
  }
  
  gpsread();
 
  if(gps_flag == 1)
  {
    Serial.println(Todo);
    enviarInfo(Todo);   
  }
  Todo = "";
  delay(2000);  
  gps_flag = 0;
  
}

void enviarInfo(String outgoing) {
  LoRa.beginPacket();                   // start packet
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
  msgCount++;                           // increment message ID
  Serial.println("Dato enviado");
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
        Todo += ",";
        Todo += String(fix.altitude_cm(), 6);
        Todo += ",";
        Todo += String(fix.speed_kph(), 6);
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
        Todo += ",";
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


float selectBand(int a){    
  switch(a){ 
    case 0:
    return 903.08;
  break;
    case 1:
    return 905.24;
  break;
    case 2:
    return 907.40;
  break;
    case 3:
    return 909.56;
  break;
    case 4:
    return 911.72;
  break;
    case 5:
    return 913.88;
  break;
    case 6:
    return 916.04;
  break;
    case 7:
    return 918.20;
  break;
    case 8:
    return 920.36;
  break;
    case 9:
    return 922.52;
  break;
    case 10:
    return 924.68;
  break;
    case 11:
    return 926.84;
  break;
    case 12:
    return 915;
  break;
  }
  
 }
