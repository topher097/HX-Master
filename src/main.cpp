
// Include libraries
#include <Arduino.h>
#include <Wire.h>                 // For I2C device communication
#include <LiquidCrystal_I2C.h>    // For LCD I2C communication
#include <Adafruit_MCP4725.h>     // For ADC I2C communication
// #include <SD.h>                   // For logging to SD card
// #include <SPI.h>                  // For SPI card access



// Declare devices
LiquidCrystal_I2C lcd(0x27,20,4);         // Address for LCD
Adafruit_MCP4725 piezo1;
Adafruit_MCP4725 piezo2;       
#define DAC_RESOLUTION    (8)
// SdFs sd;
// FsFile file;


// Define functions
float calcPressure(float);
float calcTempHeaterModuleThermistor(float);
float calcTempBoilSurfaceThermistor(float);
void updateLCD();
void getSensorData();
void sendData();



// Define global constants and variables
// Pressure transducer vars
float tempPressure;
float inletPressureUpstream;
float inletPressureDownstream;
float outletPressureVapor;
float outletPressureLiquid;
float pressureDriveVoltage = 12.0;          // Voltage driving pressure sensors
// Thermistor temperature vars
float tempTemperature;
float heaterTemperature1;
float heaterTemperature2;
float heaterTemperature3;
float heaterTemperature4;
float heaterTemperature5;
float boilSurfaceTemperature1;
float boilSurfaceTemperature2;
float boilSurfaceTemperature3;
float boilSurfaceTemperature4;
// Logging vars
unsigned int dataDelay = 10;                 // time between data read and transfer in milliseconds
unsigned int dataStartTime;                  // start time after data read and sent
unsigned long int testTime;                  // time since program started in milliseconds
// Controls vars
bool inPhase = true;
int frequency1;
int frequency2;
int wattage = 50;               // Desired effective wattage per cm^2
// Protection vars
bool safeHeater1 = true;
bool safeHeater2 = true;
bool safeHeater3 = true;
bool safeHeater4 = true;
bool safeHeater5 = true;
bool safeBoilSurface1 = true;
bool safeBoilSurface2 = true;
bool safeBoilSurface2 = true;
bool safeBoilSurface2 = true;



// Define pins
int P1 = A0;      // Inlet upstream pressure transducer
int P2 = A1;      // Inlet downstream pressure transducer
int P3 = A2;      // Outlet vapor pressure transducer
int P4 = A3;      // Outlet liquid pressure transducer
int HMT1 = A4;    // Heater module 1 thermistor
int HMT2 = A5;    // Heater module 2 thermistor
int HMT3 = A6;    // Heater module 3 thermistor
int HMT4 = A7;    // Heater module 4 thermistor
int HMT5 = A8;    // Heater module 5 thermistor
int BST1 = A14;   // Boil surface thermistor 1
int BST2 = A15;   // Boil surface thermistor 2
int BST3 = A16;   // Boil surface thermistor 3
int BST4 = A17;   // Boil surface thermistor 4
int P1 = 10;      // DAC for piezo 1
int P2 = 11;      // DAC for piezo 2



void setup() {
  // Initialize pinmodes
  pinMode(P1, INPUT);
  pinMode(P2, INPUT);
  pinMode(P3, INPUT);
  pinMode(P4, INPUT);
  pinMode(HMT1, INPUT);
  pinMode(HMT2, INPUT);
  pinMode(HMT3, INPUT);
  pinMode(HMT4, INPUT);
  pinMode(HMT5, INPUT);
  pinMode(BST1, INPUT);
  pinMode(BST2, INPUT);
  pinMode(BST3, INPUT);
  pinMode(BST4, INPUT);
  pinMode(P1, OUTPUT);
  pinMode(P2, OUTPUT);

  // Setup LCD
  lcd.backlight();
  updateLCD();

  // Setup DACs
  piezo1.begin(0x60);     // ADC for piezo 1 - GET CORRECT I2C ADDRESS
  piezo2.begin(0x62);     // ADC for piezo 2 - GET CORRECT I2C ADDRESS

  // Serial initialize
  Serial.begin(115200);

  // Initialize data timer
  dataStartTime = millis();
}


void loop() {
  // Check if time to read and send/log data
  if (millis()-dataStartTime >= dataDelay){
    // Get sensor data
    getSensorData();

    // Restart timer for data
    dataStartTime = millis();

    // Send data
    sendData();
  }

  // Control piezos
  

  // Control heater modules


}


// Get all sensor data anc calculate the appropriate variables
void getSensorData(){
  // Read and calculatre pressure sensor values
    float inletPressureUpstream = calcPressure(analogRead(P1), 1);     // pascal
    float inletPressureDownstream = calcPressure(analogRead(P2), 2);   // pascal
    float outletPressureVapor = calcPressure(analogRead(P3), 3);       // pascal
    float outletPressureLiquid = calcPressure(digitalRead(P4), 4);     // pascal

    // Read and calculate heater module temps
    float heaterTemperature1 = calcTempHeaterModuleThermistor(analogRead(HMT1));     // degree celcius
    float heaterTemperature2 = calcTempHeaterModuleThermistor(analogRead(HMT2));     // degree celcius
    float heaterTemperature3 = calcTempHeaterModuleThermistor(analogRead(HMT3));     // degree celcius
    float heaterTemperature4 = calcTempHeaterModuleThermistor(analogRead(HMT4));     // degree celcius
    float heaterTemperature5 = calcTempHeaterModuleThermistor(analogRead(HMT5));     // degree celcius

    // Read and calculate boil surface temps
    float boilSurfaceTemperature1 = calcTempBoilSurfaceThermistor(analogRead(BST1));    // degree celcius
    float boilSurfaceTemperature2 = calcTempBoilSurfaceThermistor(analogRead(BST2));    // degree celcius
    float boilSurfaceTemperature3 = calcTempBoilSurfaceThermistor(analogRead(BST3));    // degree celcius
    float boilSurfaceTemperature4 = calcTempBoilSurfaceThermistor(analogRead(BST4));    // degree celcius
}


// Send current data to serial port
void sendData(){
    Serial.print(testTime);
    Serial.print(',');
    Serial.print(inletPressureUpstream);
    Serial.print(',');
    Serial.print(inletPressureDownstream);
    Serial.print(',');
    Serial.print(outletPressureVapor);
    Serial.print(',');
    Serial.print(outletPressureLiquid);
    Serial.print(',');
    Serial.print(heaterTemperature1);
    Serial.print(',');
    Serial.print(heaterTemperature2);
    Serial.print(',');
    Serial.print(heaterTemperature3);
    Serial.print(',');
    Serial.print(heaterTemperature4);
    Serial.print(',');
    Serial.print(heaterTemperature5);
    Serial.print(',');
    Serial.print(boilSurfaceTemperature1);
    Serial.print(',');
    Serial.print(boilSurfaceTemperature2);
    Serial.print(',');
    Serial.print(boilSurfaceTemperature3);
    Serial.print(',');
    Serial.print(boilSurfaceTemperature4); 
}


// Calculate the pressure of the pressure transducer using interpolation function and voltage read from pin
// 30 psia sensor, 10V-15V drive, 100mV output
float calcPressure(float P, int S){
  // Get the sensor and determine which calibration curve to use
  if (S == 1){          // Upstream inlet sensor
    int atm_read = 500;       // Reading at atmospheric calibration
    float atm_psi = 14.5;     // Atmospheric pressure at calibration reading in PSI
    int cal_read = 750;       // Reading at pressurized calibration
    float cal_psi = 25.5;     // Pressure at calibration reading in PSI
    tempPressure = P * pressureDriveVoltage * 
  }
  else if (S == 2){     // Downstream inlet sensor

  }
  else if (S == 3){     // Vapor outlet sensor

  }
  else{                 // Liquid outlet sensor

  }
  
  return tempPressure;
}


// Calculate the temperature of the heater module thermistor using interpolation function and voltage read from pin
float calcTempHeaterModuleThermistor(float T){

  return tempTemperature;
}


// Calculate the temperature of the boil surface thermistor using interpolation function and voltage read from pin
float calcTempBoilSurfaceThermistor(float T){
  
  return tempTemperature;
}


// Update the LCD screen
void updateLCD(){


}