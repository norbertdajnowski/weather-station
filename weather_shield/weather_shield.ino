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
const byte WDIR = A0;
MQUnifiedsensor MQ135(placa, Voltage_Resolution, ADC_Bit_Resolution, pin, type);
long lastSecond; 
char data[26];
const int ledWarning  =  8; 
const int ledGood  =  9; 

//Global Variables
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
byte minutes; //Keeps track of where we are in various arrays of data
byte seconds;
long lastWindCheck = 0;
volatile long lastWindIRQ = 0;
volatile byte windClicks = 0;
volatile float rainHour[60]; //60 floating numbers to keep track of 60 minutes of rain

//These are all the weather values that wunderground expects:
int winddir = 0; // [0-360 instantaneous wind direction]
float windspeedmph = 0; // [mph instantaneous wind speed]
float rainin = 0; // [rain inches over the past hour)] -- the accumulated rainfall in the past 60 min
volatile float dailyrainin = 0; // [rain inches so far today in local time]

// volatiles are subject to modification by IRQs
volatile unsigned long raintime, rainlast, raininterval, rain;

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

void rainIRQ()
// Count rain gauge bucket tips as they occur
// Activated by the magnet and reed switch in the rain gauge, attached to input D2
{
  raintime = millis(); // grab current time
  raininterval = raintime - rainlast; // calculate interval between this and last event

    if (raininterval > 10) // ignore switch-bounce glitches less than 10mS after initial edge
  {
    dailyrainin += 0.011; //Each dump is 0.011" of water
    rainHour[minutes] += 0.011; //Increase this minute's amount of rain

    rainlast = raintime; // set up for next event
  }
}


void wspeedIRQ()
// Activated by the magnet in the anemometer (2 ticks per rotation), attached to input D3
{
  if (millis() - lastWindIRQ > 10) // Ignore switch-bounce glitches less than 10ms (142MPH max reading) after the reed switch closes
  {
    lastWindIRQ = millis(); //Grab the current time
    windClicks++; //There is 1.492MPH for each click per second.
  }
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
  
  // attach external interrupt pins to IRQ functions
  attachInterrupt(0, rainIRQ, FALLING);
  attachInterrupt(1, wspeedIRQ, FALLING);
  // turn on interrupts
  interrupts();
  
  Serial.println("Weather Shield online!");
}

//Main Loop
void loop()
{

    //Calc the wind speed and direction every second for 120 second to get 2 minute average
    float currentSpeed = get_wind_speed();
    windspeedmph = currentSpeed; //update global variable for windspeed when using the printWeather() function
    //float currentSpeed = random(5); //For testing
    int currentDirection = get_wind_direction();
    winddir = get_wind_direction(); 
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

      Serial.print("$,winddir=");
      Serial.print(winddir);
      Serial.print(",windspeedmph=");
      Serial.print(windspeedmph, 1);
      Serial.print(",rainin=");
      Serial.print(rainin, 2);
      Serial.print(",dailyrainin=");
      Serial.print(dailyrainin, 2);
    }
    
    delay(10000);
}

//Returns the instataneous wind speed
float get_wind_speed()
{
  float deltaTime = millis() - lastWindCheck; //750ms

  deltaTime /= 1000.0; //Covert to seconds

  float windSpeed = (float)windClicks / deltaTime; //3 / 0.750s = 4

  windClicks = 0; //Reset and start watching for new wind
  lastWindCheck = millis();

  windSpeed *= 1.492; //4 * 1.492 = 5.968MPH

  /* Serial.println();
   Serial.print("Windspeed:");
   Serial.println(windSpeed);*/

  return(windSpeed);
}

//Read the wind direction sensor, return heading in degrees
int get_wind_direction() 
{
  unsigned int adc;

  adc = analogRead(WDIR); // get the current reading from the sensor

  // The following table is ADC readings for the wind direction sensor output, sorted from low to high.
  // Each threshold is the midpoint between adjacent headings. The output is degrees for that ADC reading.
  // Note that these are not in compass degree order! See Weather Meters datasheet for more information.

  if (adc < 380) return (113);
  if (adc < 393) return (68);
  if (adc < 414) return (90);
  if (adc < 456) return (158);
  if (adc < 508) return (135);
  if (adc < 551) return (203);
  if (adc < 615) return (180);
  if (adc < 680) return (23);
  if (adc < 746) return (45);
  if (adc < 801) return (248);
  if (adc < 833) return (225);
  if (adc < 878) return (338);
  if (adc < 913) return (0);
  if (adc < 940) return (293);
  if (adc < 967) return (315);
  if (adc < 990) return (270);
  return (-1); // error, disconnected?
}
