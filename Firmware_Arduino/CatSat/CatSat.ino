/************************************************************
SPANISH
CatSat.ino
CatSat - Satelite en Lata Educativo
Andres Sabas @ Electronic Cats
Original Creation Date: Jal 10, 2016
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
************************************************************/


#include <Wire.h>
#include <SoftwareSerial.h>
#include <I2Cdev.h>

#include <TinyGPS.h>

#include "MPU6050.h"

#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_BMP085_U.h>
// #include <DHT.h>
#include <DHT_U.h>

#define DHTPIN 2 // Pin digital para DHT22

#define DHTTYPE DHT22   // DHT 22  (AM2302)

// Inicializar DHT sensor.
DHT_Unified dht(DHTPIN, DHTTYPE);

MPU6050 accelgyro;
// #### Variables de Aceleración y Giroscopio
// #### Accel and Gyro Vars
int16_t ax, ay, az;
int16_t gx, gy, gz;
// ####

// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_ACCELGYRO


TinyGPS gps;
SoftwareSerial ss(4, 3); //4(rx) and 3(tx)

uint32_t delayMS;

/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(1);

Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(2);

void displayHMCDetails(void)
{
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void displayBMPDetails(void)
{
  sensor_t sensor;
  bmp.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" hPa");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" hPa");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" hPa");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void displayDHTDetails(void)
{
  // Print temperature sensor details.
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.println("Temperature");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" *C");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" *C");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" *C");  
  Serial.println("------------------------------------");
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.println("Humidity");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println("%");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println("%");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println("%");  
  Serial.println("------------------------------------");
   // verify connection
  Serial.println("------------------------------------"); 
  Serial.println("Position");
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  Serial.println("------------------------------------"); 
  // Set delay between sensor readings based on sensor details.
  delayMS = sensor.min_delay / 1000;
}

void gpsread(void){
  
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (ss.available())
    {
      char c = ss.read();
      // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

  if (newData)
  {
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    Serial.print(F("LAT="));
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print(F(" LON="));
    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    Serial.print(F(" SAT="));
    Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    Serial.print(F(" PREC="));
    Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
  }
  
  gps.stats(&chars, &sentences, &failed);
  Serial.print(F(" CHARS="));
  Serial.print(chars);
  Serial.print(F(" SENTENCES="));
  Serial.print(sentences);
  Serial.print(F(" CSUM ERR="));
  Serial.println(failed);
  if (chars == 0)
    Serial.println(F("** No characters received from GPS: check wiring **"));
}

void setup() {
  Serial.begin(9600);
  ss.begin(115200);
  dht.begin();

  /* Initialise the sensor */
  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP085 ... check your connections */
    Serial.print(F("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!"));
    while(1);
  }

   #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
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
  Serial.println(F("Display some basic information on this sensors"));
  displayHMCDetails();
  displayBMPDetails();
  displayDHTDetails();
  
  Serial.println(F("CatSat!"));
  
}

void loop() {
  /* Get a new sensor event */ 
  sensors_event_t event;

  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println("Error reading temperature!");
  }
  else {
    Serial.print("TemperatureDHT: ");
    Serial.print(event.temperature);
    Serial.println(" *C");
  }
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println("Error reading humidity!");
  }
  else {
    Serial.print("HumidityDHT: ");
    Serial.print(event.relative_humidity);
    Serial.println("%");
  }

  bmp.getEvent(&event);
 
  /* Display the results (barometric pressure is measure in hPa) */
  if (event.pressure)
  {
    /* Display atmospheric pressue in hPa */
    Serial.print("Pressure:    ");
    Serial.print(event.pressure);
    Serial.println(" hPa");
    
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
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" C");

    /* Then convert the atmospheric pressure, and SLP to altitude         */
    /* Update this next line with the current SLP for better results      */
    float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
    Serial.print("Altitude:    "); 
    Serial.print(bmp.pressureToAltitude(seaLevelPressure,
                                        event.pressure)); 
    Serial.println(" m");
    Serial.println("");
  }
  else
  {
    Serial.println("Sensor error");
  }
  
   
  mag.getEvent(&event);
 
  // Display the results (magnetic vector values are in micro-Tesla (uT))
  Serial.print("Magnetometro:  "); 
  Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");

  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  float declinationAngle = 0.22;
  heading += declinationAngle;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI; 
  
  Serial.print("Heading (degrees): "); Serial.println(headingDegrees);

  // ##########  read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        // display tab-separated accel/gyro x/y/z values
        Serial.print("Acelerometro "); 
        Serial.print("X:"); Serial.print(ax); Serial.print("\t");
        Serial.print("Y:"); Serial.print(ay); Serial.print("\t");
        Serial.print("Z:"); Serial.print(az); Serial.print("\n");
        Serial.print("Giroscopio "); 
        Serial.print("X:"); Serial.print(gx); Serial.print("\t");
        Serial.print("X:"); Serial.print(gy); Serial.print("\t");
        Serial.print("X:"); Serial.println(gz);Serial.print("\n");
    
    /// Fin de Giroscopio y acelerometro.
  Serial.println ("------------fin de sensado--------------");
  
  gpsread();
  
  delay(1000);  

}
