//Arduino Code
#include <TSYS01.h>
#include <Wire.h>
#include <floatToString.h>
#include <MS5837.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>




#ifdef USE_PULSE_OUT
  #include "ph_iso_surveyor.h"       
  Surveyor_pH_Isolated pH = Surveyor_pH_Isolated(A0);         
#else
  #include "ph_surveyor.h"             
  Surveyor_pH pH = Surveyor_pH(A0);   
#endif

Servo myServo;
#define servoPIN 9
MS5837 depthReader = MS5837();
TSYS01 tempReader = TSYS01();

#define TCADDR 0x70
#define NULL_SENSOR_VALUE -320000
#define FAULTY_SENSOR_VALUE -40404


void ReadAllSensors()
{
 // Wire.beginTransmission(0x76);
 tcaselect(1);
  Wire.beginTransmission(118);
    depthReader.read();
    tcaselect(7);
  Wire.beginTransmission(119);
    tempReader.read();
  String tempStringC;
  if (tempReader.temperature() < -1000)
  {
    // Serial.println("No Got it");
    tempStringC = FAULTY_SENSOR_VALUE;
  }
  else
  {
    // Serial.println("Got it");
    tempStringC = String(tempReader.temperature(), 5);
  }
  String pHString = String(pH.read_ph(), 5);
  String depthString = String(depthReader.depth(), 5);
  String altString = String(depthReader.altitude(), 5);
  String pressureString = String(depthReader.pressure(), 5);
  Serial.begin(9600);
  Wire.begin();
  Serial.println("Reading," + depthString + "," + pressureString + "," + altString +
                 "," + tempStringC + "," + pHString);
}
void tcaselect(uint8_t portOnTCA) {
  if (portOnTCA > 7) return;
 
  Wire.beginTransmission(TCADDR);
  Wire.write(1 << portOnTCA);
  Wire.endTransmission();  
}
void startupSensors()
{
  tcaselect(1);
  Wire.beginTransmission(118);

  depthReader.setModel(MS5837::MS5837_30BA);
  depthReader.setFluidDensity(997);
  depthReader.init();
  tcaselect(7);
  Wire.beginTransmission(119);
  tempReader.init();
  if (depthReader.init() && tempReader.init() && pH.begin())
  {
    Serial.println("All sensors are ready.");
  }
  else
  {
    Serial.println("failed from arduino side.");
    while(!depthReader.init()){
      Serial.println("Depth sensor.");
      delay(100);
    }
    while(!tempReader.init()){
      Serial.println("temp sensor.");
      delay(100);
    }
    while(!pH.begin()){
      Serial.println("PH sensor.");
      delay(100);
    }
    Serial.println("All sensors are ready.");
  }

  return 0;
}

void Dropper(){
  String command = Serial.readStringUntil('.');
  int state = command.toInt();
  switch(state){
    case 1:
      myServo.write(50);
      break;
    case 2:
      myServo.write(15);
      break;
    case 0:
      myServo.write(80);
      break;
  }
}
void setup()
{
  Serial.begin(9600);
  Wire.begin();
  startupSensors();
  Serial.begin(9600);
  Wire.begin();
  String givenString = Serial.readStringUntil('.');
  Serial.println(givenString);
  while (givenString != "start") {
    Serial.println("Sensors are waiting");
    givenString = Serial.readStringUntil('.');
    delay(10);
    // debugging: Serial.println(givenString);
  }
  myServo.attach(servoPIN);
  myServo.write(90);
  // loop();
}
void loop()
{
  ReadAllSensors();
  Dropper();
  
}
