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

*/
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

//Creamos objeto LoRa
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Inicializar DHT sensor.
DHT_Unified dht(DHTPIN, DHTTYPE);

MPU6050 accelgyro;
// #### Variables de Aceleración y Giroscopio
// #### Accel and Gyro Vars
int16_t ax, ay, az;
int16_t gx, gy, gz;

String Todo; //String a mandar

TinyGPSPlus gps;
static const int RXPin = 5, TXPin = 6;
static const uint32_t GPSBaud = 9600;
int gps_flag = 0;

SoftwareSerial ss(RXPin, TXPin);
//#define ss Serial

uint32_t delayMS;

/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(1);

Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(2);


/*Uncomment for debbuger*/
/*
void displayHMCDetails(void)
{
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" uT"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" uT"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" uT"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
  delay(500);
}

void displayBMPDetails(void)
{
  sensor_t sensor;
  bmp.getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" hPa"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" hPa"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" hPa"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
  delay(500);
}

void displayDHTDetails(void)
{
  // Print temperature sensor details.
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.println(F("Temperature"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" *C"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" *C"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" *C"));  
  Serial.println(F("------------------------------------"));
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.println(F("Humidity"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F("%"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F("%"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F("%"));  
  Serial.println(F("------------------------------------"));
   // verify connection
  Serial.println(F("------------------------------------")); 
  Serial.println(F("Position"));
  Serial.println(F("Testing device connections..."));
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  Serial.println("------------------------------------"); 
  // Set delay between sensor readings based on sensor details.
  delayMS = sensor.min_delay / 1000;
}
*/

void gpsread(void){
  
  // 
  while ((ss.available() > 0) && (gps_flag == 0))
    if (gps.encode(ss.read()))
    {
     Serial.print(F("Location: ")); 
      if (gps.location.isValid())
      { 
        Todo += String(gps.location.lat(), 6);
        Todo += ",";
        Todo += String(gps.location.lng(), 6);
        Todo += "\n";
        Serial.print(gps.location.lat(), 6);
        Serial.print(F(","));
        Serial.print(gps.location.lng(), 6);
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
  Serial.println("LoRa radio init OK!");
 
  // Defaults after init are 915.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  float chann = selectBand(channel);
  if (!rf95.setFrequency(chann)) {
    Serial.println(F("setFrequency failed"));
    while (1);
    }
    
  rf95.setTxPower(23, false); //Set the max transmition power
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

  /* Display some basic information on this sensor */
  /*Uncomment for debbuger*/
  /*
  Serial.println(F("Display some basic information on this sensors"));
  displayHMCDetails();
  displayBMPDetails();
  displayDHTDetails();
  */
  
  //Serial.println(F("CatSat!"));
  
}

void loop() {
  /* Get a new sensor event */ 
  Todo += id_node;  //Add id to String 
  Todo += ",";
  sensors_event_t event;

  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    //Serial.println(F("Error reading temperature!"));
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
  // Get humidity event and print its value.
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
    /* Calculating altitude with reasonable accuracy requires pressure    *
     * sea level pressure for your position at the moment the data is     *
     * converted, as well as the ambient temperature in degress           *
     * celcius.  If you don't have these values, a 'generic' value of     *
     * 1013.25 hPa can be used (defined as SENSORS_PRESSURE_SEALEVELHPA   *
     * in sensors.h), but this isn't ideal and will give variable         *
     * results from one day to the next.                                  *
     *                                                                    *
     * You can usually find the current SLP value by looking at weather   *
     * websites or from environmental information centers near any major  *
     * airport.                                                           *
     *                                                                    *
     * For example, for Paris, France you can check the current mean      *
     * pressure and sea level at: http://bit.ly/16Au8ol                   */
     
    /* First we get the current temperature from the BMP085 */
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
    
    /* Then convert the atmospheric pressure, and SLP to altitude         */
    /* Update this next line with the current SLP for better results      */
    //float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
    /*Uncomment for debbuger*/
    /*
    Serial.print(F("Altitude:    ")); 
    Serial.print(bmp.pressureToAltitude(seaLevelPressure,
                                        event.pressure)); 
    Serial.println(F(" m"));
    Serial.println(F(""));
    */
  }
  else
  {
    //Serial.println(F("Sensor error"));
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
  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  //float heading = atan2(event.magnetic.y, event.magnetic.x);
  
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  //float declinationAngle = 0.22;
  //heading += declinationAngle;
  
  // Correct for when signs are reversed.
 /* if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI; 
  

  //Uncomment for debbuger
  Serial.print(F("Heading (degrees): ")); Serial.println(headingDegrees);
*/
  
  // ##########  read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        // display tab-separated accel/gyro x/y/z values
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
     // Todo += "\n";  

        gpsread();
 
  if(gps_flag == 1)
  {
    char todoch[Todo.length()+1];
    Todo.toCharArray(todoch,Todo.length());
    Serial.println(todoch);
    rf95.send((uint8_t *)todoch,Todo.length());   
  }
  Todo = "";
  delay(1000);  
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
