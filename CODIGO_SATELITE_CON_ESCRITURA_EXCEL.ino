
#include <SoftwareSerial.h>   //Se incluye la librería para simular dos puertos serial
#include <TinyGPS++.h>        //Se incluye la librería del módulo GPS
#include <Wire.h>             //Se incluye la librería para usar los puertos SCL y SDA
#include <Adafruit_BMP085.h>  //Se incluye la librería del sensor de temperatura y presión atmosférica
#include "Adafruit_VEML6075.h"//Se incluye la librería del sensor de radiación ultravioleta
SoftwareSerial radio(0, 1); // RX, TX ; Se declara el puerto serial para el módulo APC220
SoftwareSerial ss(4, 3);    // RX, TX ; Se declara el puerto serial para el módulo GPS
Adafruit_BMP085 bmp;        // Se declara el sensor BMP085
TinyGPSPlus gps;            // Se declara el GPS
Adafruit_VEML6075 uv = Adafruit_VEML6075();  //Se declara el sensor de radiación ultravioleta
int A0Mq; //Se declara una variable tipo int para almacenar los datos de la conexión analógica del sensor de calidad de aire
int D0Mq; //Se declara una variable tipo int para almacenar los datos de la conexión digital del sensor de calidad de aire
void setup() {       //Configuración que se establece al iniciar el microcontrolador
  Serial.begin(9600); //Se inicia el puerto serial con una velocidad de transmision de 9600 baudios
  ss.begin(9600); //Se inicia el puerto serial del GPS con una velocidad de transmision de 9600 baudios
  Serial.println("CLEARSHEET"); //Se transmite este texto en mayúsculas para que al ser recibido por la macro de excel limpie las filas de la tabla
  Serial.println("RESETTIMER"); //Se transmite este texto en mayúsculas para que al ser recibido por la macro de excel reinicie el contador
  if (! uv.begin()) {
    Serial.println("Failed to communicate with VEML6075 sensor, check wiring?");        //Si no se encuentra recepción de datos del sensor VEML6075, se manda un error
    while (1) { delay(100); }
  }
  if (!bmp.begin()) {
    Serial.println("No se encuentra el sensor BMP180, comprueba la conexión!");         //Si no se encuentra recepción de datos del sensor BMP180, se manda un error
    Serial.println("No se encuentra el sensor BMP180, comprueba la conexión!");
  }
}
 

void loop() {    //Bucle de funciones que se repite
    A0Mq = analogRead(A0);      //Se establece la variable A0Mq como el valor de lectura cíclica del puerto analógico (A0)
    D0Mq = digitalRead (8);     //Se establece la variable D0Mq como el valor de lectura cíclica del puerto digital (8)
    Serial.print("DATA,TIME,"); //Se transmite este texto en mayúscula para que la macro de excel escriba en la primera columna el temporizador y en las siguientes columnas escriba los datos recibidos separados por comas
    Serial.print(bmp.readTemperature()); //Se transmite el valor de la temperatura
    Serial.print(",");       //Se envía un coma como texto para que al ser leída por la macro de excel cambie de columna para escribir los datos en la siguiente columna
    Serial.print(bmp.readPressure()/100.0,2);    //Se transmite el valor de la presión atmosférica
    Serial.print(",");     //Se envía un coma como texto para que al ser leída por la macro de excel cambie de columna para escribir los datos en la siguiente columna
    Serial.print(bmp.readAltitude(102110));  //Se transmite el valor de la altitud barométrica en metros
    Serial.print(",");     //Se envía un coma como texto para que al ser leída por la macro de excel cambie de columna para escribir los datos en la siguiente columna
    Serial.print(gps.location.lat(),6);     //Se transmite el valor de latitud
    Serial.print(",");    //Se envía un coma como texto para que al ser leída por la macro de excel cambie de columna para escribir los datos en la siguiente columna
    Serial.print(gps.location.lng(),6);    //Se transmite el valor de longitud
    Serial.print(",");    //Se envía un coma como texto para que al ser leída por la macro de excel cambie de columna para escribir los datos en la siguiente columna
    Serial.print(gps.altitude.meters());   //Se transmite el valor de altitud
    smartDelay(25);      //Se ejecuta un retraso inteligente declarado en la función smartDelay
    Serial.print(",");    //Se envía un coma como texto para que al ser leída por la macro de excel cambie de columna para escribir los datos en la siguiente columna
    Serial.print(uv.readUVI());     //Se transmite el valor de radiación ultravioleta
    Serial.print(",");   //Se envía un coma como texto para que al ser leída por la macro de excel cambie de columna para escribir los datos en la siguiente columna
    Serial.println(A0Mq);      //Se transmite el valor de partículas por millón presentes en el aire
    delay (1000);   //Se establece un intervalo de un segundo para que se vuelva a repetir el ciclo
}


static void smartDelay(unsigned long ms){
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}
