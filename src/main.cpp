
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
#include <i2c_t3.h>               // For multiple I2C channels

// Hardware serial (UART)
#define SLAVE_SERIAL Serial8

// Hardware I2C




// Header files
#include <coeffDefinitions.h>       // Header file with interpolation function coefficients
#include <pinDefinitions.h>         // Header file with all master teensy pin definitions
#include <varDefinitions.h>         // Header file with all other variables used
#include <calculations.h>           // Header file with calculations of temps and pressures
#include <slaveUART.h>              // Header file for encoding and decoding UART to/from slave
#include <defaultPiezo.h>           // Header file with default piezo driving properties

// Namespaces
using namespace varDefinitions;
using namespace pinDefinitions;
using namespace coeffDefinitions;
using namespace defaultPiezoProperties;
using namespace std;


// I2C devices
LiquidCrystal_I2C lcd(0x27,20,4);         // Address for LCD

// SPI devices
//Adafruit_MAX31855 thermocouple(IFT_SCK, IFT_CS, IFT_MISO);      // Software SPI implementation
Adafruit_MAX31855 thermocouple(IFT_CS);                         // Hardware SPI implementation

// Timer objects
IntervalTimer blinkTimer;
IntervalTimer dataSlaveSend;
IntervalTimer updateLCDTimer;

// Special characters for LCD
uint8_t backSlashLCD [8]= { 0x00, 0x10, 0x08, 0x04, 0x02, 0x01, 0x00, 0x00 };

// Analog resolution
int16_t analogResolution = 12;                          // 12 bit resolution
int16_t maxAnalog = pow(2, analogResolution)-1;          // Max analog resolution 2^(analogResolution)-1

// Blink the status LED
void blinkLED() {
  if (ledState == LOW) {
      ledState = HIGH;
  } 
  else {
      ledState = LOW;
  }
  digitalWrite(BLINK, ledState);
  //Serial.println("BLINK");
}

// Reset the piezo properties to the default values
void resetPiezoProperties() {
    frequency1 = default_frequency1;    // Frequency of left channel piezo in Hz
    frequency2 = default_frequency2;    // Frequency of right channel piezo in Hz
    amplitude1 = default_amplitude1;    // Amplitude of sine wave 1 (left cahnnel); 0-1
    amplitude2 = default_amplitude2;    // Amplitude of sine wave 2 (right channel); 0-1
    phase1 = default_phase1;            // Phase of left channel signal in degrees
    phase2 = default_phase2;            // Phase of right channel signal in degrees
    enable1 = default_enable1;            // Enable pin for piezo driver 1
    enable2 = default_enable2;            // Enable pin for piezo driver 2
}

// Send current data to serial port
void sendData(){
    Serial.print(testTimePrint);
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
  if (heaterTemperature1 >= heaterMaxTemp || heaterTemperature2 >= heaterMaxTemp || heaterTemperature3 >= heaterMaxTemp || heaterTemperature4 >= heaterMaxTemp || heaterTemperature5 >= heaterMaxTemp){safeHeater = false;}

  // Boil surface
  if (boilSurfaceTemperature1 >= boilSurfaceMaxTemp || boilSurfaceTemperature2 >= boilSurfaceMaxTemp || boilSurfaceTemperature3 >= boilSurfaceMaxTemp || boilSurfaceTemperature4 >= boilSurfaceMaxTemp){safeBoilSurface = false;}
  
  // Rope heater/inlet temp
  if (inletFluidTemperature >= inletMaxTemp){safeInletTemp = false;}

  // If thermal runaway, run safety shut off and display error
  if (!safeBoilSurface || !safeHeater || !safeInletTemp){
    // Turn off updateLCDTimer
    updateLCDTimer.end();
    
    // Clear the LCD for thermal runaway error message
    lcd.clear();

    // Turns off all heaters when thermal runaway occurs
    digitalWrite(HMD1, LOW);    // Turn off heater module 1
    digitalWrite(HMD2, LOW);    // Turn off heater module 2
    digitalWrite(HMD3, LOW);    // Turn off heater module 3
    digitalWrite(HMD4, LOW);    // Turn off heater module 4
    digitalWrite(HMD5, LOW);    // Turn off heater module 5
    digitalWrite(RHD, LOW);     // Turn off rope heater

    // Set true or false strings for printing
    String _true = "True";
    String _false = "False";
    String safeHeaterString = "";
    String safeBoilSurfaceString = "";
    String safeInletTempString = "";
    if (safeHeater){safeHeaterString = _true;} else {safeHeaterString = _false;}
    if (safeBoilSurface){safeBoilSurfaceString = _true;} else {safeBoilSurfaceString = _false;}
    if (safeInletTemp){safeInletTempString = _true;} else {safeInletTempString = _false;}
    


    // Run until system is reset, refresh screen every second
    u_int16_t errorTimeStart = millis();
    u_int16_t errorDisplayRefresh = 1000;     // Refresh time in ms
    char loadingSymbolVect[4] = {"|/-"};
    char loadingSybol;
    u_int16_t step = 0;
    while (true){
      // Displays error message on the LCD panel
      if (millis() - errorTimeStart >= errorDisplayRefresh){
        // Get current loading symbol
        if (step < 3){loadingSybol = loadingSymbolVect[step]; step++;}
        else {loadingSybol = char(0); step = 0;}
        
        // Display to screen
        lcd.setCursor(0, 0);
        lcd.print("THERMAL RUNAWAY  ");
        lcd.print(loadingSybol);
        lcd.setCursor(0, 1);
        lcd.print("Heaters safe: " + safeHeaterString);
        lcd.setCursor(0, 2);
        lcd.print("In temp safe: " + safeInletTempString);
        lcd.setCursor(0, 3);
        lcd.print("BS temp safe: " + safeBoilSurfaceString);
        errorTimeStart = millis();

      }
    }
  }

}

// Update the LCD screen
void updateLCD(){
  lcd.setCursor(0, 0);
  lcd.print("Flow rate: " + (String)round(inletFlowRate) + "mL/min");
  lcd.setCursor(0, 1);
  lcd.print("Fluid temp: " + (String)inletFluidTemperature + "C");
  lcd.setCursor(0, 2);
  lcd.print("Avg. BS temp: " + (String)round(averageBoilSurfaceTemp) + "C");
  lcd.setCursor(0, 3);
  lcd.print("E flux: " + (String)round(heatEnergyDensity) + "W/cm^2");
  //Serial.println("LCD UPDATE");
}

// Get all sensor data anc calculate the appropriate variables
void getData(){
  // Read and calculate pressure sensor values
  inletPressureUpstream = calcPressure((float)analogRead(P1)/maxAnalog*3.3);      // psi
  inletPressureDownstream = calcPressure((float)analogRead(P2)/maxAnalog*3.3);    // psi
  outletPressureVapor = calcPressure((float)analogRead(P3)/maxAnalog*3.3);        // psi
  outletPressureLiquid = calcPressure((float)analogRead(P4)/maxAnalog*3.3);       // psi
  Serial.print(inletPressureUpstream);
  Serial.print(", ");
  Serial.print(inletPressureUpstream);
  Serial.print(", ");
  Serial.print(outletPressureVapor);
  Serial.print(", ");
  Serial.print(outletPressureLiquid);
  Serial.println("");

  // Calculate the inlet flow rate
  inletFlowRate = calcInletFlowRate();        // mL/min

  // Read and calculate heater module temps
  heaterTemperature1 = calcTempHeaterModuleThermistor(analogRead(HMT1)/maxAnalog);     // degree celcius
  heaterTemperature2 = calcTempHeaterModuleThermistor(analogRead(HMT2)/maxAnalog);     // degree celcius
  heaterTemperature3 = calcTempHeaterModuleThermistor(analogRead(HMT3)/maxAnalog);     // degree celcius
  heaterTemperature4 = calcTempHeaterModuleThermistor(analogRead(HMT4)/maxAnalog);     // degree celcius
  heaterTemperature5 = calcTempHeaterModuleThermistor(analogRead(HMT5)/maxAnalog);     // degree celcius   

  // Read and calculate boil surface temps
  boilSurfaceTemperature1 = calcTempBoilSurfaceThermistor((float)analogRead(BST1)/maxAnalog*3.3);    // degree celcius
  boilSurfaceTemperature2 = calcTempBoilSurfaceThermistor((float)analogRead(BST2)/maxAnalog*3.3);    // degree celcius
  boilSurfaceTemperature3 = calcTempBoilSurfaceThermistor((float)analogRead(BST3)/maxAnalog*3.3);    // degree celcius
  boilSurfaceTemperature4 = calcTempBoilSurfaceThermistor((float)analogRead(BST4)/maxAnalog*3.3);    // degree celcius
  // Serial.print(boilSurfaceTemperature1);
  // Serial.print(", ");
  // Serial.print(boilSurfaceTemperature2);
  // Serial.print(", ");
  // Serial.print(boilSurfaceTemperature3);
  // Serial.print(", ");
  // Serial.print(boilSurfaceTemperature4);
  // Serial.println("");


  averageBoilSurfaceTemp = (boilSurfaceTemperature1 + boilSurfaceTemperature2 + boilSurfaceTemperature3 + boilSurfaceTemperature4)/4; // degree celcius

  // Read and calcualte inlet fluid temp
  inletFluidTemperature = thermocouple.readCelsius();            // degree celcius
}

// Send data to the slave teensy (on interupt)
void sendDataSlave(){
  SLAVE_SERIAL.println(encodeSlaveUART());
}


// End all data collection and sending and save data
void endTest(){
  // Stop the interval timers
  //blinkTimer.end();
  updateLCDTimer.end();
  dataSlaveSend.end();

  // Turn off piezo 1 and 2
  enable1 = false;
  enable2 = false;
  sendDataSlave();

  // Save data to the SD card


  // Display to LCD screen
  lcd.clear();

  // Run until system is reset
  uint16_t blinkCharacterDelay = 1000;
  uint64_t blinkCharacterTimer = millis()+blinkCharacterDelay;
  bool blink = true;
  while (true){
    lcd.setCursor(0, 0);
    if (millis() - blinkCharacterTimer >= blinkCharacterDelay){
      if (blink){lcd.print("*TEST HAS CONCLUDED*"); blink = !blink;}
      else {lcd.print(" TEST HAS CONCLUDED "); blink = !blink;}
      blinkCharacterTimer = millis();
    }
    
    lcd.setCursor(0, 2);
    lcd.print(" RESTART SYSTEM FOR");
    lcd.setCursor(0, 3);
    lcd.print("     NEXT  TEST");
  }
}

void setup() {
  // Setup default piezo properties
  resetPiezoProperties();

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

  // Begin SPI bus
  SPI.begin();

  // Begin I2C bus
  Wire.begin();

  // Setup LCD
  lcd.init();
  lcd.backlight();
  lcd.begin(20, 4);
  lcd.createChar(0, backSlashLCD);
  updateLCD();

  // Serial initialization
  Serial.begin(115200);     // To MATLAB
  SLAVE_SERIAL.begin(115200);      // To Teensy4.0 Slave

  // Initialize data timer
  dataStartTime = millis(); // change to interupt

  // Initialize test timer
  testTimeStart = millis();

  // LCD update interupt
  unsigned int updateDelay;
  unsigned int minDelay = 250;
  if (dataDelay > minDelay){updateDelay = dataDelay;}
  else {updateDelay = minDelay;}
  updateLCDTimer.begin(updateLCD, updateDelay*1000);
  updateLCDTimer.priority(2);

  // Blink interupt
  blinkTimer.begin(blinkLED, blinkDelay*1000);      // Run the blinkLED function at delay speed (ns)
  blinkTimer.priority(1);                           // Priority 0 is highest, 255 is lowest
  
  // Send data to slave interupt
  dataSlaveSend.begin(sendDataSlave, 500000);
  dataSlaveSend.priority(0);

  // Set analog resolution
  analogReadResolution(analogResolution);

}

int stopCount = 0;
int stopLimit = 25;
void loop() {
  // Check if time to read and send/log data
  if (millis()-dataStartTime >= dataDelay){
    stopCount++;
    getData();                                            // Get sensor data
    //inletFluidTemperature = thermocouple.readCelsius();   // degree celcius
    testTime = (millis() - testTimeStart)/100;           // Get current test time in seconds
    testTimePrint = testTime * 0.1;
    dataStartTime = millis();                             // Restart timer for data
    //sendData();                                           // Send data
    //checkThermalRunaway();                                // Check that all heating elements are safe
    //saveDataToSD()                                        // Saves the data to a CSV file on the SD card
  }

  // If stop test condition met, stop test
  // if (stopCount >= stopLimit){
  //   endTest();
  // }

  // Check if new serial in from MATLAB and send that data to slave Teensy for piezo control
  // String incomingByte;
  // if (Serial.available() > 0) {
  //   incomingByte = Serial.readString();
  //   Serial.print("USB received: ");
  //   Serial.println(incomingByte);
  //   // decodeMATLABSerial();
  //   String sendSlave = encodeSlaveUART();     // Send update piezo bytes to slave
  //   SLAVE_SERIAL.println(sendSlave);
  // }
  // // Check if new serial from slave teensy
  // if (SLAVE_SERIAL.available() > 0) {
  //   Serial.println("HI");
  //   incomingByte = SLAVE_SERIAL.readString();
  //   Serial.print("UART received: ");
  //   Serial.println(incomingByte);
  //   decodeSlaveUART(incomingByte.c_str());
  //   SLAVE_SERIAL.print("UART received:");
  //   SLAVE_SERIAL.println(incomingByte);
  // }
  

  // Control heater modules

}


