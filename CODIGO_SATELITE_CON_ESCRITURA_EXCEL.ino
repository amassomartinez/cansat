#include <Servo.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include "Adafruit_VEML6075.h"//Librería del BMP180
SoftwareSerial radio(0, 1); // RX, TX   Radio APC220 
SoftwareSerial ss(4, 3);    // RX, TX   Recepción GPS Neo6M
Adafruit_BMP085 bmp;        // BMP180
TinyGPSPlus gps;            // GPS Neo6M
Adafruit_VEML6075 uv = Adafruit_VEML6075();
String orden = "";
String paracaidas="";
int A0Mq;
int D0Mq;
int segundo = 0;
void setup() {
  Serial.begin(9600);
  radio.begin(9600);
  ss.begin(9600);
  Serial.println("CLEARSHEET");
  Serial.println("RESETTIMER");
  if (! uv.begin()) {
    Serial.println("Failed to communicate with VEML6075 sensor, check wiring?");
    while (1) { delay(100); }
  }
  if (!bmp.begin()) {
    Serial.println("No se encuentra el sensor BMP180, comprueba la conexión!");
    Serial.println("No se encuentra el sensor BMP180, comprueba la conexión!");
  }
}
 

void loop() { // run over and over
    A0Mq = analogRead(A0);
    D0Mq = digitalRead (8);
    /****** Comienza la transmisión por radio ******/
    while (Serial.available()>0)
    {
    char caracter= Serial.read();
    if (caracter != '\n'){
    orden = orden + caracter;
    delay (25);
    }
    if (orden=="abrir"){
    paracaidas = "PARACAIDAS ABIERTO"; 
    }
    }
    Serial.print("DATA,TIME,");
    Serial.print(bmp.readTemperature());
    Serial.print(",");
    Serial.print(bmp.readPressure()/100.0,2);    // Presión en hPa 
    Serial.print(",");
    Serial.print(bmp.readAltitude(102110));  // Altitud barométrica en m 
    Serial.print(",");
    Serial.print(gps.location.lat(),6);      // Latitud GPS
    Serial.print(",");
    Serial.print(gps.location.lng(),6);      // Longitud GPS
    Serial.print(",");
    Serial.print(gps.altitude.meters());   // Altitud GPS
    smartDelay(25);
    Serial.print(",");
    Serial.print(uv.readUVI());
    Serial.print(",");
    Serial.println(A0Mq);
    delay (1000);
    
    /****** Fin de transimisón por radio ******/
}


static void smartDelay(unsigned long ms){
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}
