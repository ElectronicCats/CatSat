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
#include <SPI.h>
#include <RH_RF95.h>

#include <Wire.h>
#include <SoftwareSerial.h>
#include <I2Cdev.h>

#include <TinyGPS++.h>

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
#define RF95_FREQ 915.0 //Usados creando el objeto

//Command activation Balloon mode
#define PMTK_SET_NMEA_886_PMTK_FR_MODE  "$PMTK001,886,3*36"

String id_node= "A1"; //CAMBIAR ID DE NODO
int channel = 12;      //Cambiar canal de tu satelite
float chann;

//Creamos objeto LoRa
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Inicializar DHT sensor.
DHT_Unified dht(DHTPIN, DHTTYPE);

//MPU6050 accelgyro;

String Todo; //String a mandar

TinyGPSPlus gps;
static const int RXPin = 5, TXPin = 6;
static const uint32_t GPSBaud = 9600;
int gps_flag = 0;

SoftwareSerial ss(RXPin, TXPin);

uint32_t delayMS;

/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(1);

Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(2);

float selectBand(int);

void gpsread(void){
  while ((ss.available() > 0) && (gps_flag == 0))
    if (gps.encode(ss.read()))
    {
     Serial.print(F("Location: ")); 
      if (gps.location.isValid())
      { 
        Todo += String(gps.location.lat(), 6);
        Todo += ",";
        Todo += String(gps.location.lng(), 6);
        Todo += ",";
        Todo += String(gps.altitude.meters(), 6);
        Todo += ",";
        Todo += String(gps.speed.mps(), 6);
        Todo += "\n";
        Serial.print(gps.location.lat(), 6);
        Serial.print(F(","));
        Serial.print(gps.location.lng(), 6);
        Serial.print(F(","));
        Serial.print(gps.altitude.meters(), 6);
        Serial.print(F(","));
        Serial.print(gps.speed.mps(), 6);
        
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
      if (gps.date.isValid())
      {
        Serial.print(gps.date.month());
        Serial.print(F("/"));
        Serial.print(gps.date.day());
        Serial.print(F("/"));
        Serial.print(gps.date.year());
      }
      else
      {
        Serial.print(F("INVALID"));
      }

      Serial.print(F(""));
      if (gps.time.isValid())
      {
        if (gps.time.hour() < 10) Serial.print(F("0"));
        Serial.print(gps.time.hour());
        Serial.print(F(":"));
      if (gps.time.minute() < 10) Serial.print(F("0"));
        Serial.print(gps.time.minute());
        Serial.print(F(":"));
      if (gps.time.second() < 10) Serial.print(F("0"));
        Serial.print(gps.time.second());
        Serial.print(F("."));
      if (gps.time.centisecond() < 10) Serial.print(F("0"));
        Serial.print(gps.time.centisecond());
      }
      else
      {
        Serial.print(F("INVALID"));
      }

      Serial.println(); 
     }
      

      if (millis() > 5000 && gps.charsProcessed() < 10)
      {
        Serial.println(F("No GPS detected: check wiring."));
        while(true);
      }
  
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
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  delay(100);
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  
  while (!rf95.init()) {
    Serial.println(F("LoRa radio init failed"));
    while (1);
  }
 // setFrequency(Bw125Cr48Sf4096);   //Bw125 CR=4/8 SF12

  chann = selectBand(channel);
  if (!rf95.setFrequency(chann)) {
    while (1);
  }


  rf95.setTxPower(23, false); //Set the max transmition power
  /******************/
  Serial.println("LoRa radio init OK!");
 
  
  if(!bmp.begin())
  {
    Serial.print(F("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!"));
    while(1);
  }

  
  Serial.println(F("CatSat!"));

    
  /* Initialise the sensor 
  if(!mag.begin())
  {
    Serial.println(F("Ooops, no HMC5883 detected ... Check your wiring!"));
    while(1);
  }
*/
  /* Display some basic information on this sensor */
  /*Uncomment for debbuger*/
  /*
  Serial.println(F("Display some basic information on this sensors"));
  displayHMCDetails();
  displayBMPDetails();
  displayDHTDetails();
  */
  
  
}

void loop() {
  /* Get a new sensor event */ 
  Todo += id_node;  //Add id to String 
  Todo += ",";
  sensors_event_t event;
  //Todo += 0;
  //Todo += ","; 
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
 
  /* Display the results (barometric pressure is measure in hPa) */
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
     
    /* First we get the current temperature from the BMP085 */
    float temperature;
    bmp.getTemperature(&temperature);
    Todo += temperature;
    Todo += ","; 
  }
  else
  {
    //Serial.println(F("Sensor error"));
  }
  
  gpsread();
 
  if(gps_flag == 1)
  {
    char todoch[Todo.length()+1];
    Todo.toCharArray(todoch,Todo.length());
    Serial.println(todoch);
    rf95.send((uint8_t *)todoch,Todo.length()); 
  }
  Todo = "";
  delay(2000);  
  gps_flag = 0;
  
}


float selectBand(int a)
{    
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
