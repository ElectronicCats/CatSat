/************************************************************
SPANISH
EarthStation.ino
EarthStation - Estacion Terrena para CatSat
Eduardo Contreras @ Electronic Cats
Original Creation Date: Jul 14, 2016
https://github.com/ElectronicsCats/CatSat/

Este ejemplo demuestra el funcionamiento basico de la estacion terrena
la cual filtra los Satelites

http://electroniccats.com

Especificaciones del entorno de Desarrollo:
  IDE: Arduino 1.6.8
  Plataforma de Hardware:
  Estacion terrena CanSat
  - Arduino Mini Pro
  - RFM95


Este código es beerware si tu me ves ( o cualquier otro miembro de Electronic Cats) 
a nivel local, y tu has encontrado nuestro código útil ,
por favor comprar una ronda de cervezas!

Distribuido como; no se da ninguna garantía.
************************************************************/

/************************************************************
ENGLISH
EarthStation.ino
EarthStation - Earth-Station for CatSat
Eduardo contreras @ Electronic Cats
Original Creation Date: Jal 14, 2016
https://github.com/ElectronicsCats/CatSat/

This example demonstrates how to use the earth-station for CatSat

Development environment specifics:
  IDE: Arduino 1.6.8
  Hardware Platform:
  Kit CanSat
  - Arduino Mini Pro
  - RFM95
  

This code is beerware; if you see me (or any other Electronic Cats 
member) at the local, and you've found our code helpful, 
please buy us a round!

Distributed as-is; no warranty is given.

Library LoRa Radio
http://www.airspayce.com/mikem/arduino/RadioHead/index.html
**********************************************************
*IMPORTANTE CAMBIA TU ID DEPENDIENDO DE TU CANSAT         *
**********************************************************/

#include <SPI.h>
#include <RH_RF95.h>

String ID = "A1";

String buff;
RH_RF95 rf95(10, 2);
 
 
void setup() 
{     
  pinMode(9, OUTPUT);
  digitalWrite(9, HIGH);
 
  while (!Serial);
  Serial.begin(9600);
  delay(100);
  
  // manual reset
  digitalWrite(9, LOW);
  delay(10);
  digitalWrite(9, HIGH);
  delay(10);
 
  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  if (!rf95.setFrequency(915.0)) {
    while (1);
  }
  rf95.setTxPower(23, false);
}
 
void loop()
{
  
  if (rf95.available())
  {
    //Recibio mensaje completo  
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    
    if (rf95.recv(buf, &len))
    {
      String buff = (char*)buf;
       if(buff.startsWith(ID))
        {
          Serial.println((char*)buf);
          }
      }
    else
      {
      Serial.println(F("Receive failed"));
       }
  }
}


