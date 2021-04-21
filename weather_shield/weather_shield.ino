#include <SoftwareSerial.h>
#include <Wire.h> 
#include <MQUnifiedsensor.h>
#include "SparkFunMPL3115A2.h"
#include "SparkFun_Si7021_Breakout_Library.h" 

#define placa "Arduino UNO"
#define Voltage_Resolution 5
#define pin A0 
#define type "MQ-135" 
#define ADC_Bit_Resolution 10 
#define RatioMQ135CleanAir 3.6//RS / R0 = 3.6 ppm  

MPL3115A2 myPressure; 
Weather myHumidity;

//Static Variables (I/O Pins)
const byte STAT_BLUE = 7;
const byte STAT_GREEN = 8;
const byte REFERENCE_3V3 = A3;
const byte LIGHT = A1;
const byte BATT = A2;
const byte WSPEED = 3;
const byte RAIN = 2;
MQUnifiedsensor MQ135(placa, Voltage_Resolution, ADC_Bit_Resolution, pin, type);
long lastSecond; 
char data[26];
const int ledWarning  =  8; 
const int ledGood  =  9; 

//Weather Station Functions
void checkLevels(float CO)
{
  if (CO > 10){
    digitalWrite(ledGood,LOW);   
    digitalWrite(ledWarning,HIGH);   
  }else{
    digitalWrite(ledGood,HIGH); 
    digitalWrite(ledWarning,LOW);
  }
}

float get_CO()
{
  MQ135.setA(605.18); MQ135.setB(-3.937);
  return(MQ135.readSensor());
}

float get_Alcohol()
{
  MQ135.setA(77.255); MQ135.setB(-3.18);
  return(MQ135.readSensor());
}

float get_CO2()
{
  MQ135.setA(110.47); MQ135.setB(-2.862);
  return(MQ135.readSensor());
}

float get_light_level()
{
  float operatingVoltage = analogRead(REFERENCE_3V3);
  float lightSensor = analogRead(LIGHT);
  operatingVoltage = 3.3 / operatingVoltage; 
  lightSensor = operatingVoltage * lightSensor;
  return (lightSensor);
}

float get_battery_level()
{
  float operatingVoltage = analogRead(REFERENCE_3V3);
  float rawVoltage = analogRead(BATT);
  operatingVoltage = 3.30 / operatingVoltage; 
  rawVoltage = operatingVoltage * rawVoltage;
  rawVoltage *= 4.90;
  return (rawVoltage);
}

//Script start
void setup()
{
  float calcR0 = 0;
  Serial.begin(9600);
  myPressure.begin();
  myPressure.setModeBarometer();
  myPressure.setOversampleRate(7);
  myPressure.enableEventFlags();
  pinMode(ledGood,OUTPUT);   
  pinMode(ledWarning,OUTPUT);
  pinMode(WSPEED, INPUT_PULLUP);
  pinMode(RAIN, INPUT_PULLUP);
  
  MQ135.setRegressionMethod(1);
  MQ135.init(); 
  
  Serial.print("Calibrating please wait.");
  for(int i = 1; i<=10; i ++)
  {
    MQ135.update();
    calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
    Serial.print(".");
  }
  
  MQ135.setR0(calcR0/10);
  Serial.println("  done!.");
  
  if(isinf(calcR0)) {Serial.println("Warning: Conection issue founded, R0 is infite (Open circuit detected) please check your wiring and supply"); while(1);}
  if(calcR0 == 0){Serial.println("Warning: Conection issue founded, R0 is zero (Analog pin with short circuit to ground) please check your wiring and supply"); while(1);}
  
  digitalWrite(ledGood,HIGH);   
  digitalWrite(ledWarning, LOW);
  myHumidity.begin();
  lastSecond = millis();

//  attachInterrupt(0, rainIRQ, FALLING);
//  attachInterrupt(1, wspeedIRQ, FALLING);

//  interrupts();
  
  Serial.println("Weather Shield online!");
}

//Main Loop
void loop()
{
    MQ135.update();   
    float humidity = myHumidity.getRH(); 
    
    Serial.println();
    if (humidity == 998)
    {
      Serial.println("I2C communication to sensors is not working. Check solder connections.");
      myPressure.begin(); 
      myPressure.setModeBarometer();
      myPressure.setOversampleRate(7);
      myPressure.enableEventFlags();
      myHumidity.begin();
    }
    else
    {
      //Update all readings
      float CO = get_CO();
      float temp_h = (myHumidity.readTempF() - 32) * 5/9;
      float pressure = myPressure.readPressure();
      float light_lvl = get_light_level();
      float Alcohol = get_Alcohol();
      
      //format and print readings 
      String dataH = "Humidity: " + String(humidity) + "% ";
      dataH.toCharArray(data,26);
      Serial.write(data);
      
      String dataT = "Temp: " + String(temp_h) + "C ";
      dataT.toCharArray(data,26);
      Serial.write(data);
      
      String dataP = "Pressure: " + String(pressure) + "Pa ";
      dataP.toCharArray(data,26);
      Serial.write(data);
            
      String dataL = "Light: " + String(light_lvl) + "V ";
      dataL.toCharArray(data,26);
      Serial.print(data);
      
      String dataCO = "CO1: " + String(CO) + "PPM ";
      dataCO.toCharArray(data,26);
      Serial.write(data);
      checkLevels(CO);
      
      String dataAlc = "Alcohol: " + String(Alcohol) + "PPM ";
      dataAlc.toCharArray(data,26);
      Serial.write(data);

    }
    
    delay(10000);
}
