
// Include libraries
#include <Arduino.h>
#include <Wire.h>                 // For I2C device communication
#include <LiquidCrystal_I2C.h>    // For LCD I2C communication
#include <string>
#include <math.h> 
#include <string>
#include <vector>
#include <sstream>
#include <Adafruit_MAX31855.h>    // Thermocouple amplifier module
#include <SD.h>                   // For logging to SD card
#include <SPI.h>                  // For SPI card access

// Hardware serial (UART)
#define SLAVE_SERIAL Serial8

// Header files
#include <coeffDefinitions.h>       // Header file with interpolation function coefficients
#include <pinDefinitions.h>         // Header file with all master teensy pin definitions
#include <varDefinitions.h>         // Header file with all other variables used

// Namespaces
using namespace varDefinitions;
using namespace pinDefinitions;
using namespace coeffDefinitions;
using namespace std;

// I2C devices
LiquidCrystal_I2C lcd(0x27,20,4);         // Address for LCD

// SPI devices
//Adafruit_MAX31855 thermocouple(IFT_SCK, IFT_CS, IFT_MISO);      // Software SPI implementation
Adafruit_MAX31855 thermocouple(IFT_CS);                         // Hardware SPI implementation

// Timer objects
IntervalTimer blinkTimer;
IntervalTimer dataSlaveSend;

// Blink the status LED
void blinkLED() {
  if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }
  digitalWrite(BLINK, ledState);
  Serial.println("BLINK");
}

// Calculate the temperature of the heater module thermistor using interpolation function and voltage read from pin
float calcTempHeaterModuleThermistor(float V){
  // Note that the analog value is anwhere from 0 to 4095, so that value is converted to 0 to 3.3V in interpolation function automatically
  float tempTemperature = HMcoef0*pow(V, 0) + HMcoef1*pow(V, 1) + HMcoef2*pow(V, 2) + HMcoef3*pow(V, 3) + HMcoef4*pow(V, 4) + HMcoef5*pow(V, 5) + HMcoef6*pow(V, 6) + HMcoef7*pow(V, 7) + HMcoef8*pow(V, 8) + HMcoef9*pow(V, 9) + HMcoef10*pow(V, 10) + HMcoef11*pow(V, 11);
  return tempTemperature;
}

// Calculate the temperature of the boil surface thermistor using interpolation function and voltage read from pin
float calcTempBoilSurfaceThermistor(float V){
  // Note that the analog value is anwhere from 0 to 4095, so that value is converted to 0 to 3.3V in interpolation function automatically
  float tempTemperature = BScoef0*pow(V, 0) + BScoef1*pow(V, 1) + BScoef2*pow(V, 2) + BScoef3*pow(V, 3) + BScoef4*pow(V, 4) + BScoef5*pow(V, 5) + BScoef6*pow(V, 6) + BScoef7*pow(V, 7);
  return tempTemperature;
}

// Calculate the pressure of the pressure transducer using interpolation function and voltage read from pin
// 30 psia sensor, 10V-15V drive, 100mV output
float calcPressure(float V){
  // Get the analog value and convert to psi
  float tempPressure = Pcoef0*pow(V, 0) + Pcoef1*pow(V, 1);
  return tempPressure;
}

// Calculate the inlet flow rate using the two pressure measurements from inlet sensors
float calcInletFlowRate(){
  float valveOpenCv = 0.0818;                 // gal/min fully open on valve 2
  float galpermin_to_mlpermin = 3785.41;      // 1 gal/min is 3785.41 mL/min
  float sg = 1.54;                            // fluid specific gravity
  inletFlowRate = valveOpenCv * sqrt(abs(inletPressureUpstream - inletPressureDownstream)/sg);
  return inletFlowRate;
}

// Encode the piezo properties for serial as type String()
String encodeSlaveUART(){
  String encodedUART;
  encodedUART += (String)frequency1;
  encodedUART += (String)frequency2;
  encodedUART += (String)amplitude1;
  encodedUART += (String)amplitude2;
  encodedUART += (String)phase1;
  encodedUART += (String)phase2;
  encodedUART += (String)enable1;
  encodedUART += (String)enable2;
  return encodedUART;
}

// Decode the piezo properties from serial
void decodeSlaveUART(string incomingBytes){
  vector<float> v;
  stringstream ss(incomingBytes);
  
  // Split comma separated string into different elements and add to vector 
  while (ss.good()){
    string substr;
    float temp;
    getline(ss, substr, ',');
    istringstream ss2(substr);
    ss2 >> temp;
    v.push_back(temp);
  }
  
  // Take elements of vector and save to appropriate variables
  int i = 1;
  frequency1 = v[i]; i++;
  frequency2 = v[i]; i++;
  amplitude1 = v[i]; i++;
  amplitude2 = v[i]; i++;
  phase1     = v[i]; i++;
  phase2     = v[i]; i++;
  enable1    = (int)v[i]; i++;
  enable2    = (int)v[i];
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
    Serial.print(',');
    Serial.print(averageBoilSurfaceTemp);
    Serial.print(',');
    Serial.print(inletFlowRate); 
    Serial.print(',');
    Serial.print(inletFluidTemperature);
    Serial.write(13);   // Carriage return "CR"
    Serial.write(10);   // Linefeed "LF"
}

// Checks if all of the thermistors don't have a thermal runaway, if there is a thermal runaway detected, turn off experiment
void checkThermalRunaway(){
  // Heater modules
  if (heaterTemperature1 >= heaterMaxTemp || heaterTemperature2 >= heaterMaxTemp || heaterTemperature3 >= heaterMaxTemp || heaterTemperature4 >= heaterMaxTemp || heaterTemperature5 >= heaterMaxTemp){
    safeHeater = false;
  }

  // Boil surface
  if (boilSurfaceTemperature1 >= boilSurfaceMaxTemp || boilSurfaceTemperature2 >= boilSurfaceMaxTemp || boilSurfaceTemperature3 >= boilSurfaceMaxTemp || boilSurfaceTemperature4 >= boilSurfaceMaxTemp){
    safeBoilSurface = false;
  }
  
  // Rope heater/inlet temp
  if (inletFluidTemperature >= inletMaxTemp){
    safeInletTemp = false;
  }

  // If thermal runaway, run safety shut off and display error
  if (!safeBoilSurface || !safeHeater){
    while (true){
      // Turns off all heaters when thermal runaway occurs
      digitalWrite(HMD1, LOW);    // Turn off heater module 1
      digitalWrite(HMD2, LOW);    // Turn off heater module 2
      digitalWrite(HMD3, LOW);    // Turn off heater module 3
      digitalWrite(HMD4, LOW);    // Turn off heater module 4
      digitalWrite(HMD5, LOW);    // Turn off heater module 5
      digitalWrite(RHD, LOW);     // Turn off rope heater

      // Displays error message on the LCD panel
      lcd.setCursor(0, 0);
      lcd.print("THERMAL RUNAWAY");
      lcd.setCursor(0, 1);
      lcd.print("Heaters safe:" + (String)safeHeater);
      lcd.setCursor(0, 2);
      lcd.print("Inlet safe: " + (String)safeInletTemp);
      lcd.setCursor(0, 3);
      lcd.print("BS safe: " + (String)safeBoilSurface);
    }
  }

}

// Update the LCD screen
void updateLCD(){
  if (inPhase){
    phaseText = "in phase";
  }
  else{
    phaseText = "out phase";
  }

  lcd.setCursor(0, 0);
  lcd.print((String)frequency1 + "Hz, " + phaseText);
  lcd.setCursor(0, 1);
  lcd.print("Flow rate: " + (String)round(inletFlowRate) + "mL/min");
  lcd.setCursor(0, 2);
  lcd.print("Avg. boil surf: " + (String)round(averageBoilSurfaceTemp) + "C");
  lcd.setCursor(0, 3);
  lcd.print("E Density: " + (String)round(heatEnergyDensity) + "W/cm^3");
}

// Get all sensor data anc calculate the appropriate variables
void getData(){
  // Read and calculate pressure sensor values
  inletPressureUpstream = calcPressure(analogRead(P1));      // psi
  inletPressureDownstream = calcPressure(analogRead(P2));    // psi
  outletPressureVapor = calcPressure(analogRead(P3));        // psi
  outletPressureLiquid = calcPressure(analogRead(P4));       // psi

  // Calculate the inlet flow rate
  inletFlowRate = calcInletFlowRate();        // mL/min

  // Read and calculate heater module temps
  heaterTemperature1 = calcTempHeaterModuleThermistor(analogRead(HMT1));     // degree celcius
  heaterTemperature2 = calcTempHeaterModuleThermistor(analogRead(HMT2));     // degree celcius
  heaterTemperature3 = calcTempHeaterModuleThermistor(analogRead(HMT3));     // degree celcius
  heaterTemperature4 = calcTempHeaterModuleThermistor(analogRead(HMT4));     // degree celcius
  heaterTemperature5 = calcTempHeaterModuleThermistor(analogRead(HMT5));     // degree celcius   

  // Read and calculate boil surface temps
  boilSurfaceTemperature1 = calcTempBoilSurfaceThermistor(analogRead(BST1));    // degree celcius
  boilSurfaceTemperature2 = calcTempBoilSurfaceThermistor(analogRead(BST2));    // degree celcius
  boilSurfaceTemperature3 = calcTempBoilSurfaceThermistor(analogRead(BST3));    // degree celcius
  boilSurfaceTemperature4 = calcTempBoilSurfaceThermistor(analogRead(BST4));    // degree celcius
  averageBoilSurfaceTemp = (boilSurfaceTemperature1 + boilSurfaceTemperature2 + boilSurfaceTemperature3 + boilSurfaceTemperature4)/4; // degree celcius

  // Read and calcualte inlet fluid temp
  inletFluidTemperature = thermocouple.readCelsius();            // degree celcius
}

void sendDataSlave(){
  SLAVE_SERIAL.println(encodeSlaveUART());
}

void setup() {
  // Initialize pinmodes
  pinMode(BLINK, OUTPUT);   // Status LED
  pinMode(P1, INPUT);       // Inlet upstream pressure sensor (from op-amp)
  pinMode(P2, INPUT);       // Inlet downstream pressure sensor (from op-amp)
  pinMode(P3, INPUT);       // Outlet vapor pressure sensor (from op-amp)
  pinMode(P4, INPUT);       // Outlet liquid pressure sensor (from op-amp)
  pinMode(HMT1, INPUT);     // Thermistor from heater module 1
  pinMode(HMT2, INPUT);     // Thermistor from heater module 2
  pinMode(HMT3, INPUT);     // Thermistor from heater module 3
  pinMode(HMT4, INPUT);     // Thermistor from heater module 4
  pinMode(HMT5, INPUT);     // Thermistor from heater module 5
  pinMode(BST1, INPUT);     // Thermistor from boil surface num 1
  pinMode(BST2, INPUT);     // Thermistor from boil surface num 2
  pinMode(BST3, INPUT);     // Thermistor from boil surface num 3
  pinMode(BST4, INPUT);     // Thermistor from boil surface num 4
  pinMode(HMD1, OUTPUT);    // Heater module 1
  pinMode(HMD2, OUTPUT);    // Heater module 2
  pinMode(HMD3, OUTPUT);    // Heater module 3
  pinMode(HMD4, OUTPUT);    // Heater module 4
  pinMode(HMD5, OUTPUT);    // Heater module 5
  pinMode(RHD, OUTPUT);     // Rope heater
  pinMode(IFT_CS, OUTPUT);  // CS pin for inlet fluid thermocouple

  // Setup LCD
  lcd.backlight();
  lcd.begin(20, 4);
  updateLCD();

  // Begin SPI bus
  SPI.begin();

  // Serial initialization
  Serial.begin(115200);     // To MATLAB
  SLAVE_SERIAL.begin(9600);      // To Teensy4.0 Slave

  // Initialize data timer
  dataStartTime = millis(); // change to interupt

  // Initialize test timer
  testTimeStart = millis();

  // Blink interupt
  blinkTimer.begin(blinkLED, blinkDelay*1000);      // Run the blinkLED function at delay speed (ns)
  blinkTimer.priority(1);                           // Priority 0 is highest, 255 is lowest
  dataSlaveSend.begin(sendDataSlave, 500000);
  dataSlaveSend.priority(0);
}


void loop() {
  // Check if time to read and send/log data
  if (millis()-dataStartTime >= dataDelay){
    getData();                                  // Get sensor data
    testTime = millis() - testTimeStart;        // Get current test time
    dataStartTime = millis();                   // Restart timer for data
    sendData();                                 // Send data
    checkThermalRunaway();                      // Check that all heating elements are safe
    //saveDataToSD()                              // Saves the data to a CSV file on the SD card
  }

  // Check if new serial in from MATLAB and send that data to slave Teensy for piezo control
  String incomingByte;
  if (Serial.available() > 0) {
    incomingByte = Serial.readString();
    Serial.print("USB received: ");
    Serial.println(incomingByte);
    // decodeMATLABSerial();
    String sendSlave = encodeSlaveUART();     // Send update piezo bytes to slave
    SLAVE_SERIAL.println(sendSlave);
  }
  // Check if new serial from slave teensy
  if (SLAVE_SERIAL.available() > 0) {
    incomingByte = SLAVE_SERIAL.readString();
    Serial.print("UART received: ");
    Serial.println(incomingByte);
    decodeSlaveUART(incomingByte.c_str());
    SLAVE_SERIAL.print("UART received:");
    SLAVE_SERIAL.println(incomingByte);
  }
  

  // Control heater modules

}


