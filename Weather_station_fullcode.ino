/*
  SD card read/write

  This example shows how to read and write data to and from an SD card file
  The circuit:
   SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 4 (for MKRZero SD: SDCARD_SS_PIN)

  created   Nov 2010
  by David A. Mellis
  modified 9 Apr 2012
  by Tom Igoe

  This example code is in the public domain.

*/
#include <ThreeWire.h>
#include <RtcDS1302.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>              
#include <LiquidCrystal_I2C.h> 
#include <Arduino.h>
#include <OneWire.h> 
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 8 
OneWire oneWire(ONE_WIRE_BUS); 
DallasTemperature sensors(&oneWire);
// #include <DHT.h>
// #include <DHT_U.h>
// #define DHTPIN 8
// #define DHTTYPE DHT22
// DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27, 20, 4); // 0x27 is I2C module address 

ThreeWire myWire(4, 5, 3); // IO, SCLK, CE
RtcDS1302<ThreeWire> Rtc(myWire);

//_________
int measurePin = A0; //Connect dust sensor to Arduino A0 pin
int ledPower = 7;   //Connect 3 led driver pins of dust sensor to Arduino D2
int samplingTime = 280; // time required to sample signal coming out  of the sensor
int deltaTime = 40; // 
int sleepTime = 9680;
float voMeasured = 0;
float calcVoltage = 0;
float dustDensity = 0;
//_________


//_________
const int hallSensorPin = 2; //( chân tín hiệu : ONLY pin 2) 
const float mmPerTilt = 0.00195; // Amount of rainfall (in mm) per sensor tilt
//_________


//_________
unsigned long startTime_temp = 0;
unsigned long startTime_dust = 0;
unsigned long startTime_wind = 0;
unsigned long startTime_rain = 0;
unsigned long startTime_uv = 0;
unsigned long startTime_lcd = 0;
unsigned long startTime_sd = 0;

volatile unsigned long tipCount = 0;
volatile bool raining = false;

float rainfallAmount;
float rainfallSpeed;

float Level;
float uvIntensity;
//_________

//_________
//pin definitions
// int UV_OUT = A1;    //Sensor Output
#define UV_OUT A1
#define REF_3V3 A2
// int REF_3V3 = A2;   //3.3V power on the Arduino board
//_________

//Takes an average of readings on a given pin
//Returns the average
int analogRead_average(int pinToRead)
{
  int NumberOfSamples = 8;
  int runningValue = 0; 

  for(int x = 0; x < NumberOfSamples; x++)
    runningValue += analogRead(pinToRead);
    runningValue /= NumberOfSamples;

  return(runningValue);
}

//The Arduino Map function but for floats
//From: http://forum.arduino.cc/index.php?topic=3922.0
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

File myFile;
const int chipSelect = 53;
String fileName = "datalog1.txt";

void setup() {
  lcd.init();            
  lcd.begin(20,4);      
  lcd.backlight();

  Serial.begin(9600);

  sensors.begin(); 

  pinMode(hallSensorPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(hallSensorPin), tipCount, RISING);
  
  // dht.begin();
  
  pinMode(ledPower,OUTPUT);
  
  rtcSetting();

  sdSetting();

  delay(1000);
  }

void loop() {
  RtcDateTime now = Rtc.GetDateTime();
  String dateTime = printDateTime(now);
  if (!now.IsValid())
  {
    // Common Causes:
    //    1) the battery on the device is low or even missing and the power line was disconnected
    Serial.println("RTC lost confidence in the DateTime!");
  }

//_________
  unsigned long currentTime_dust = millis();
  unsigned long deltaTime_dust = currentTime_dust - startTime_dust;

  if (deltaTime_dust >= 1000) {
    digitalWrite(ledPower,LOW); // power on the LED
    delayMicroseconds(samplingTime);
    voMeasured = analogRead(measurePin); // read the dust value
    delayMicroseconds(deltaTime);
    digitalWrite(ledPower,HIGH); // turn the LED off
    delayMicroseconds(sleepTime);
    calcVoltage = voMeasured * (5.0 / 1024.0);
    dustDensity = 170 * calcVoltage - 0.1;

    startTime_dust = currentTime_dust;
  }
//_________



//_________
  unsigned long currentTime_wind = millis();
  unsigned long deltaTime_wind = currentTime_wind - startTime_wind;

  if (deltaTime_wind >= 1000) {
    int sensorValue = analogRead(A3);
    float outvoltage = sensorValue * (5.0 / 1023.0);
    float Level = 6 * outvoltage; //The level of wind speed is proportional to the output voltage.

    startTime_wind = currentTime_wind;
  }
//_________



//_________
  unsigned long currentTime_rain = millis();
  unsigned long deltaTime_rain = currentTime_rain - startTime_rain;

  if (deltaTime_rain >= 10000) { // Check for 10 seconds
    detachInterrupt(digitalPinToInterrupt(hallSensorPin)); // Disable interrupt during calculations
    rainfallAmount = tipCount * mmPerTilt;
    rainfallSpeed = rainfallAmount / (deltaTime / 3600000.0); // Convert deltaTime to hours
    

    // Reset variables for the next 10 seconds
    startTime_rain = currentTime_rain;
    tipCount = 0;
    raining = false;
    attachInterrupt(digitalPinToInterrupt(hallSensorPin), tipCounter, RISING);
  }
//_________



//_________
  unsigned long currentTime_uv = millis();
  unsigned long deltaTime_uv = currentTime_uv - startTime_uv;

  if (deltaTime_uv) {
    int uv_Level = analogRead_average(UV_OUT);
    int ref_Level = analogRead_average(REF_3V3);

    //Use the 3.3V power pin as a reference to get a very accurate output value from sensor
    float output_Voltage = 3.3 / ref_Level * uv_Level;

    uvIntensity = mapfloat(output_Voltage, 0.99, 2.8, 0.0, 15.0); //Convert the voltage to a UV intensity level -numbers from datasheet-

    startTime_uv = currentTime_uv;
  }
//_________


//_________
  unsigned long currentTime_lcd = millis();
  unsigned long deltaTime_lcd = currentTime_lcd - startTime_lcd;

  if (deltaTime_lcd >= 4000) {
    sensors.requestTemperatures();

    lcd.setCursor(0,0);
    lcd.print("Temp:");
    lcd.print(sensors.getTempCByIndex(0));
    lcd.print(" do C");

    lcd.setCursor(0,1);
    lcd.print("Dust: ");
    lcd.print(dustDensity); 
    lcd.print("ug/m3");

    lcd.setCursor(0,2);
    lcd.print("Wind speed: ");
    lcd.print(Level);
    lcd.print("m/s");

    delay(2000);
    lcd.clear();  



    lcd.setCursor(0,0);
    lcd.print("Rain Speed:");
    lcd.print(rainfallSpeed);
    lcd.print("mm/h");

    lcd.setCursor(0,1);
    lcd.print("UV: ");
    lcd.print(uvIntensity);
    lcd.print("mW/cm2");

    delay(2000); // one seconds
    lcd.clear();

    startTime_lcd = currentTime_lcd;
  }
//_________


  float Temperature = sensors.getTempCByIndex(0);


//_________
  unsigned long currentTime_sd = millis();
  unsigned deltaTime_sd = currentTime_sd - startTime_sd;

  if (deltaTime_sd >= 1000) {
    String temp = printDHT(Temperature);
    String dust = printDUST(dustDensity);
    String wind = printWIND(Level);
    String rain = printRAIN(rainfallSpeed);
    String uv = printUV(uvIntensity);
    writeToSD(fileName,dateTime+" "+temp+" "+dust+" "+wind+" "+rain+" "+uv);

    startTime_sd = currentTime_sd;
  }
//_________
}

void rtcSetting() {
  Serial.print("compiled: ");
  Serial.print(__DATE__);
  Serial.println(__TIME__);

  Rtc.Begin();

  RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
  printDateTime(compiled);
  Serial.println();

  if (!Rtc.IsDateTimeValid())
  {
    // Common Causes:
    //    1) first time you ran and the device wasn't running yet
    //    2) the battery on the device is low or even missing

    Serial.println("RTC lost confidence in the DateTime!");
    Rtc.SetDateTime(compiled);
  }

  if (Rtc.GetIsWriteProtected())
  {
    Serial.println("RTC was write protected, enabling writing now");
    Rtc.SetIsWriteProtected(false);
  }

  if (!Rtc.GetIsRunning())
  {
    Serial.println("RTC was not actively running, starting now");
    Rtc.SetIsRunning(true);
  }

  RtcDateTime now = Rtc.GetDateTime();
  if (now < compiled)
  {
    Serial.println("RTC is older than compile time!  (Updating DateTime)");
    Rtc.SetDateTime(compiled);
  }
  else if (now > compiled)
  {
    Serial.println("RTC is newer than compile time. (this is expected)");
  }
  else if (now == compiled)
  {
    Serial.println("RTC is the same as compile time! (not expected but all is fine)");
  }
}

void sdSetting() {
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.print("Initializing SD card...");
  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");

  // open the file for reading:
  myFile = SD.open(fileName);
  if (myFile) {
    Serial.println(fileName);

    // read from the file until there's nothing else in it:
    while (myFile.available()) {
      Serial.write(myFile.read());
    }
    // close the file:
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening");
  }

//   // delete the file:
//  Serial.println("Removing...");
//  SD.remove(fileName);

//  writeToSD(fileName, "Start write data to file");

}


void writeToSD(String fileName, String str) {
  File dataFile = SD.open(fileName, FILE_WRITE);
  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(str);
    dataFile.close();
    // print to the serial port too:
    Serial.print("Saved: ");
    Serial.println(str);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening file");
  }
}

#define countof(a) (sizeof(a) / sizeof(a[0]))
String printDateTime(const RtcDateTime& dt)
{
  char datestring[20];

  snprintf_P(datestring,
             countof(datestring),
             PSTR("%02u/%02u/%04u %02u:%02u:%02u"),
             dt.Month(),
             dt.Day(),
             dt.Year(),
             dt.Hour(),
             dt.Minute(),
             dt.Second() );
  Serial.print(datestring);
  return datestring;
}

String printDHT(float Temperature) {
  // Serial.print("\tHumid: ");
  // Serial.print(Humidity);
  Serial.print("\tTemperature: ");
  Serial.print(Temperature);
  return String(Temperature);
}


String printDUST(float dustDensity) {
  Serial.print("\tDust: ");
  Serial.print(dustDensity);
  return String(dustDensity);
}


String printWIND(float Level) {
  Serial.print("\tWind: ");
  Serial.print(Level);
  return String(Level);
}


String printRAIN(float rainfallSpeed) {
  Serial.print("\tRain: ");
  Serial.print(rainfallSpeed);
  return String(rainfallSpeed);
}


String printUV(float uvIntensity) {
  Serial.print("\tUV: ");
  Serial.println(uvIntensity);
  return String(uvIntensity);
}


void tipCounter() {
  tipCount++;
  raining = true;
}