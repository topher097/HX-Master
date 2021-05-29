
// Include libraries
#include <Arduino.h>
#include <Wire.h>                 // For I2C device communication
#include <LiquidCrystal_I2C.h>    // For LCD I2C communication
#include <Adafruit_MCP4725.h>     // For ADC I2C communication
#include <math.h>                 // Math functions
// #include <SD.h>                   // For logging to SD card
// #include <SPI.h>                  // For SPI card access



// Declare devices
LiquidCrystal_I2C lcd(0x27,20,4);         // Address for LCD
// Adafruit_MCP4725 piezo1;
// Adafruit_MCP4725 piezo2;       
// #define DAC_RESOLUTION    (8)
// SdFs sd;
// FsFile file;


// Define functions
float calcPressure(float);
float calcTempHeaterModuleThermistor(float);
float calcTempBoilSurfaceThermistor(float);
float calcInletFlowRate();
void updateLCD();
void getData();
void sendData();
void checkThermalRunaway();


// Define global constants and variables
// Pressure transducer vars
float tempPressure;
float inletPressureUpstream;
float inletPressureDownstream;
float outletPressureVapor;
float outletPressureLiquid;
float pressureDriveVoltage = 10.0;          // Voltage driving the pressure sensors
float inletFlowRate;
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
float averageBoilSurfaceTemp;
// Logging vars
unsigned int dataDelay = 10;        // time between data read and transfer in milliseconds
unsigned int dataStartTime;         // start time after data read and sent
unsigned long int testTime;         // time since program started in milliseconds
// Controls vars
bool inPhase = true;                // Piezos are in phase with each other
int frequency1 = 0;                 // Frequency of piezo 1 in Hz
int frequency2 = 0;                 // Frequency of piezo 2 in Hz
int heatEnergyDensity = 50;         // Desired effective wattage per cm^2 to be pumped through heater modules
String phaseText = "";
// Protection vars
bool safeHeater = true;
bool safeBoilSurface = true;
float heaterMaxTemp = 120.0;        // degree celcius
float boilSurfaceMaxTemp = 90.0;    // degree celcius
// Serial vars
u_int16_t rx_byte;                  // COM rx
u_int16_t tx_byte;                  // COM tx
u_int16_t rx1_byte;                 // Teensy slave rx
u_int16_t tx1_byte;                 // Teensy slave tx



// Boil surface thermistor interpolation curve coefficients
static const float BScoef0 = 374.55779068854577;
static const float BScoef1 = 17.40816141969277;
static const float BScoef2 = 0.32103832076171;
static const float BScoef3 = 0.0016992484227292349;
static const float BScoef4 = -4.3962486759910275e-05;
static const float BScoef5 = -3.7897772892121576e-07;
static const float BScoef6 = 6.9241378036387565e-09;
static const float BScoef7 = -2.3683363162345936e-11;

// Heater module thermistor interpolation curve coefficients
static const float HMcoef0 = 34.796569085903165;
static const float HMcoef1 = 2.1377274062093172;
static const float HMcoef2 = 0.037710766520922305;
static const float HMcoef3 = 0.00019364441743986369;
static const float HMcoef4 = 1.4000629550389751e-05;
static const float HMcoef5 = 6.521125926259332e-08;
static const float HMcoef6 = -3.0040976596479196e-09;
static const float HMcoef7 = 1.993233371349954e-11;
static const float HMcoef8 = -3.860489367096544e-14;
static const float HMcoef9 = -7.748547176657863e-17;
static const float HMcoef10 = 4.0350228079546027e-19;
static const float HMcoef11 = -4.352448435319378e-22;

// Pressure sensor interpolation curve coefficients
static const float Pcoef0 = -7.125683319833918e-06;
static const float Pcoef1 = 129480.71625391858;

// Analog resolution
int analogRes = 12;               // bit, either 12 or 16
int minAnalog = 0;                // minimum ADC value
int maxAnalog = pow(2, 12)-1;     // maximum ADC value, aka 2^analogRes-1

// Define pins
static const int P1 = A0;      // Inlet upstream pressure transducer
static const int P2 = A1;      // Inlet downstream pressure transducer
static const int P3 = A2;      // Outlet vapor pressure transducer
static const int P4 = A3;      // Outlet liquid pressure transducer
static const int HMT1 = A4;    // Heater module 1 thermistor
static const int HMT2 = A5;    // Heater module 2 thermistor
static const int HMT3 = A6;    // Heater module 3 thermistor
static const int HMT4 = A7;    // Heater module 4 thermistor
static const int HMT5 = A8;    // Heater module 5 thermistor
static const int BST1 = A14;   // Boil surface thermistor 1
static const int BST2 = A15;   // Boil surface thermistor 2
static const int BST3 = A16;   // Boil surface thermistor 3
static const int BST4 = A17;   // Boil surface thermistor 4
static const int HMD1 = 2;     // MOSFET for heater module 1
static const int HMD2 = 3;     // MOSFET for heater module 2
static const int HMD3 = 4;     // MOSFET for heater module 3
static const int HMD4 = 5;     // MOSFET for heater module 4
static const int HMD5 = 6;     // MOSFET for heater module 5
static const int RHD = 7;      // MOSFET for rope heater




void setup() {
  // Initialize pinmodes
  pinMode(P1, INPUT);     // Inlet upstream pressure sensor (from op-amp)
  pinMode(P2, INPUT);     // Inlet downstream pressure sensor (from op-amp)
  pinMode(P3, INPUT);     // Outlet vapor pressure sensor (from op-amp)
  pinMode(P4, INPUT);     // Outlet liquid pressure sensor (from op-amp)
  pinMode(HMT1, INPUT);   // Thermistor from heater module 1
  pinMode(HMT2, INPUT);   // Thermistor from heater module 2
  pinMode(HMT3, INPUT);   // Thermistor from heater module 3
  pinMode(HMT4, INPUT);   // Thermistor from heater module 4
  pinMode(HMT5, INPUT);   // Thermistor from heater module 5
  pinMode(BST1, INPUT);   // Thermistor from boil surface num 1
  pinMode(BST2, INPUT);   // Thermistor from boil surface num 2
  pinMode(BST3, INPUT);   // Thermistor from boil surface num 3
  pinMode(BST4, INPUT);   // Thermistor from boil surface num 4
  pinMode(HMD1, OUTPUT);  // Heater module 1
  pinMode(HMD2, OUTPUT);  // Heater module 2
  pinMode(HMD3, OUTPUT);  // Heater module 3
  pinMode(HMD4, OUTPUT);  // Heater module 4
  pinMode(HMD5, OUTPUT);  // Heater module 5
  pinMode(RHD, OUTPUT);   // Rope heater

  // Setup LCD
  lcd.backlight();
  lcd.begin(20, 4);
  updateLCD();

  // Serial initialization
  Serial.begin(115200);     // To MATLAB
  Serial1.begin(9600);      // To Teensy4.0 Slave

  // Set analog read resolution for pins
  analogReadResolution(analogRes);

  // Initialize data timer
  dataStartTime = millis();
}


void loop() {
  // Check if time to read and send/log data
  if (millis()-dataStartTime >= dataDelay){
    // Get sensor data
    getData();

    // Restart timer for data
    dataStartTime = millis();

    // Send data
    sendData();

    // Check that all heating elements are safe
    checkThermalRunaway();
  }

  // Check if new serial in from MATLAB and send that data to slave Teensy for piezo control
  if (Serial.available() > 0) { 
    rx_byte = Serial.read();       // Get the code
    

    
  } // end: if (Serial.available() > 0)
  

  // Control heater modules
  


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

  // If thermal runaway, run safety shut off and display error
  while (!safeBoilSurface || !safeHeater){

  }

}


// Update the LCD screen
void updateLCD(){
  if (inPhase){
    phaseText = "in phase";
  }
  else{
    String phaseText = "out phase";
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