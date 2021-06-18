
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
#include <EasyTransfer.h>         // For slave-master communication

// Hardware serial (UART) to slave
#define SLAVE_SERIAL Serial8

// Header files
#include <coeffDefinitions.h>       // Header file with interpolation function coefficients
#include <pinDefinitions.h>         // Header file with all master teensy pin definitions
#include <varDefinitions.h>         // Header file with all other variables used
#include <calculations.h>           // Header file with calculations of temps and pressures
#include <defaultPiezo.h>           // Header file with the default piezo properties

// Namespaces
using namespace varDefinitions;
using namespace pinDefinitions;
using namespace coeffDefinitions;
using namespace defaultPiezoProperties;
using namespace std;

// EasyTransfer objects
EasyTransfer ETout;

// I2C devices
LiquidCrystal_I2C lcd(0x27,20,4);         // Address for LCD

// SPI devices
//Adafruit_MAX31855 thermocouple(IFT_SCK, IFT_CS, IFT_MISO);      // Software SPI implementation
Adafruit_MAX31855 thermocouple(IFT_CS);                         // Hardware SPI implementation

// Timer objects
IntervalTimer blinkTimer;
IntervalTimer updateLCDTimer;

// Special characters for LCD
uint8_t backSlashLCD [8]= { 0x00, 0x10, 0x08, 0x04, 0x02, 0x01, 0x00, 0x00 };

// Analog resolution
int16_t analogResolution = 12;                          // 12 bit resolution
int16_t maxAnalog = pow(2, analogResolution)-1;          // Max analog resolution 2^(analogResolution)-1

// Struct for communicating with slave
struct SLAVE_DATA_STRUCTURE{
  float frequency1;       // Frequency of left channel piezo in Hz
  float frequency2;       // Frequency of right channel piezo in Hz
  float amplitude1;       // Amplitude of sine wave 1 (left cahnnel); 0-1
  float amplitude2;       // Amplitude of sine wave 2 (right channel); 0-1
  float phase1;           // Phase of left channel signal in degrees
  float phase2;           // Phase of right channel signal in degrees
  int enable1;            // Enable pin for piezo driver 1
  int enable2;            // Enable pin for piezo driver 2
};
SLAVE_DATA_STRUCTURE slaveData;

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
  slaveData.frequency1 = default_frequency1;    // Frequency of left channel piezo in Hz
  slaveData.frequency2 = default_frequency2;    // Frequency of right channel piezo in Hz
  slaveData.amplitude1 = default_amplitude1;    // Amplitude of sine wave 1 (left cahnnel); 0-1
  slaveData.amplitude2 = default_amplitude2;    // Amplitude of sine wave 2 (right channel); 0-1
  slaveData.phase1 = default_phase1;            // Phase of left channel signal in degrees
  slaveData.phase2 = default_phase2;            // Phase of right channel signal in degrees
  slaveData.enable1 = default_enable1;            // Enable pin for piezo driver 1
  slaveData.enable2 = default_enable2;            // Enable pin for piezo driver 2
}

// Send current data to serial port for MATLAB
void sendData(){
    Serial.print(testTimePrint);                Serial.print(',');
    Serial.print(inletPressureUpstream);        Serial.print(',');
    Serial.print(inletPressureDownstream);      Serial.print(',');
    Serial.print(outletPressureVapor);          Serial.print(',');
    Serial.print(outletPressureLiquid);         Serial.print(',');
    Serial.print(heaterTemperature1);           Serial.print(',');
    Serial.print(heaterTemperature2);           Serial.print(',');
    Serial.print(heaterTemperature3);           Serial.print(',');
    Serial.print(heaterTemperature4);           Serial.print(',');
    Serial.print(heaterTemperature5);           Serial.print(',');
    Serial.print(boilSurfaceTemperature1);      Serial.print(',');
    Serial.print(boilSurfaceTemperature2);      Serial.print(',');
    Serial.print(boilSurfaceTemperature3);      Serial.print(',');
    Serial.print(boilSurfaceTemperature4);      Serial.print(',');
    Serial.print(averageBoilSurfaceTemp);       Serial.print(',');
    Serial.print(inletFlowRate);                Serial.print(',');
    Serial.print(inletFluidTemperature);        Serial.print(',');
    Serial.print(slaveData.frequency1);         Serial.print(',');
    Serial.print(slaveData.frequency2);         Serial.print(',');
    Serial.print(slaveData.amplitude1);         Serial.print(',');
    Serial.print(slaveData.amplitude2);         Serial.print(',');
    Serial.print(slaveData.phase1);             Serial.print(',');
    Serial.print(slaveData.phase2);             Serial.print(',');
    Serial.print(slaveData.enable1);            Serial.print(',');
    Serial.print(slaveData.enable2);            Serial.print(',');
    Serial.print(endTesting);                   
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
  lcd.print((String)(inletFlowRate));
  lcd.setCursor(6, 0);
  lcd.print("mL/min, " + (String)(round((valveRotation/((float)maxRotation - (float)minRotation))*100)) + "%   ");
  lcd.setCursor(0, 1);
  lcd.print("Fluid temp: " + (String)inletFluidTemperature + " C");
  lcd.setCursor(0, 2);
  lcd.print("Avg. BS temp: " + (String)round(averageBoilSurfaceTemp) + " C");
  lcd.setCursor(0, 3);
  lcd.print("E flux: " + (String)round(heatEnergyDensity) + " W/cm^2");
}

// Get all sensor data and calculate the appropriate variables
void getData(){
  // Read and calculate pressure sensor values
  float instantInletPressureUpstream = calcPressure((float)analogRead(P1)/maxAnalog*3.3);      // psi
  float instantInletPressureDownstream = calcPressure((float)analogRead(P2)/maxAnalog*3.3);    // psi
  float instantOutletPressureVapor = calcPressure((float)analogRead(P3)/maxAnalog*3.3);        // psi
  float instantOutletPressureLiquid = calcPressure((float)analogRead(P4)/maxAnalog*3.3);       // psi

  // Take weighted average of pressure readings
  float weight = 0.1;
  inletPressureUpstream = inletPressureUpstream*weight + instantInletPressureUpstream*(1-weight);
  inletPressureUpstream = 45 + rand()/RAND_MAX;     // for testing
  inletPressureDownstream = inletPressureDownstream*weight + instantInletPressureDownstream*(1-weight);
  inletPressureDownstream = 14.5 + rand()/RAND_MAX;    // for testing
  outletPressureVapor = outletPressureVapor*weight + instantOutletPressureVapor*(1-weight);
  outletPressureLiquid = outletPressureLiquid*weight + instantOutletPressureLiquid*(1-weight);
  
  // Serial.print(inletPressureUpstream);
  // Serial.print(", ");
  // Serial.print(inletPressureDownstream);
  // Serial.print(", ");
  // Serial.print(outletPressureVapor);
  // Serial.print(", ");
  // Serial.print(outletPressureLiquid);
  // Serial.println("");

  // Calculate the inlet flow rate
  weight = 0.1;
  // Note, if need to flip direction, do maxAnalog-analogRead(POT)
  int16_t potRead = analogRead(POT);                                        // Get reading from valve potentiometer
  valveRotation = valveRotation*weight + potRead*(1-weight);                // Take weighted average of pot of reading to smooth
  float instantFlowRate = calcInletFlowRate((float)valveRotation/maxAnalog*3.3, inletPressureUpstream, inletPressureDownstream);    // mL/min
  inletFlowRate = inletFlowRate*weight + instantFlowRate*(1-weight);
  
  // Serial.print(testTimePrint);
  // Serial.print(", ");
  // Serial.print(inletFlowRate);
  // Serial.print(", ");
  // Serial.println(inletPressureUpstream-inletPressureDownstream);


  // Read and calculate heater module temps
  heaterTemperature1 = calcTempHeaterModuleThermistor((float)analogRead(HMT1)/maxAnalog*3.3);     // degree celcius
  
  //heaterTemperature2 = calcTempHeaterModuleThermistor((float)analogRead(HMT2)/maxAnalog*3.3);     // degree celcius
  //heaterTemperature3 = calcTempHeaterModuleThermistor((float)analogRead(HMT3)/maxAnalog*3.3);     // degree celcius
  //heaterTemperature4 = calcTempHeaterModuleThermistor((float)analogRead(HMT4)/maxAnalog*3.3);     // degree celcius
  //heaterTemperature5 = calcTempHeaterModuleThermistor((float)analogRead(HMT5)/maxAnalog*3.3);     // degree celcius   

  // Read and calculate boil surface temps
  float instantBoilSurfaceTemperature1 = calcTempBoilSurfaceThermistor((float)analogRead(BST1)/maxAnalog*3.3);    // degree celcius
  float instantBoilSurfaceTemperature2 = calcTempBoilSurfaceThermistor((float)analogRead(BST2)/maxAnalog*3.3);    // degree celcius
  float instantBoilSurfaceTemperature3 = calcTempBoilSurfaceThermistor((float)analogRead(BST3)/maxAnalog*3.3);    // degree celcius
  float instantBoilSurfaceTemperature4 = calcTempBoilSurfaceThermistor((float)analogRead(BST4)/maxAnalog*3.3);    // degree celcius

  // Take weighted average of boil surface temps
  weight = 0.1;
  boilSurfaceTemperature1 = boilSurfaceTemperature1*weight + instantBoilSurfaceTemperature1*(1-weight);
  boilSurfaceTemperature2 = boilSurfaceTemperature2*weight + instantBoilSurfaceTemperature2*(1-weight);
  boilSurfaceTemperature3 = boilSurfaceTemperature3*weight + instantBoilSurfaceTemperature3*(1-weight);
  boilSurfaceTemperature4 = boilSurfaceTemperature4*weight + instantBoilSurfaceTemperature4*(1-weight);
  
  // Serial.print(boilSurfaceTemperature1);
  // Serial.print(", ");
  // Serial.print(boilSurfaceTemperature2);
  // Serial.print(", ");
  // Serial.print(boilSurfaceTemperature3);
  // Serial.print(", ");
  // Serial.print(boilSurfaceTemperature4);
  // Serial.println("");

  // Calculate the average boil surface temperature
  averageBoilSurfaceTemp = (boilSurfaceTemperature1 + boilSurfaceTemperature2 + boilSurfaceTemperature3 + boilSurfaceTemperature4)/4; // degree celcius

  // Read and calcualte inlet fluid temp
  inletFluidTemperature = thermocouple.readCelsius();            // degree celcius, instant reading
}

// End all data collection and sending and save data
void endTest(){
  // Stop the interval timers
  //blinkTimer.end();
  updateLCDTimer.end();

  // Turn off piezo 1 and 2
  slaveData.enable1 = false;
  slaveData.enable2 = false;
  ETout.sendData();

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

// Decode the serial from MATLAB and update variable values
void decodeMATLABSerial(string inputString){
  vector<float> v;
  stringstream ss(inputString);
  
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
  slaveData.frequency1 = v[i]; i++;
  slaveData.frequency2 = v[i]; i++;
  slaveData.amplitude1 = v[i]; i++;
  slaveData.amplitude2 = v[i]; i++;
  slaveData.phase1     = v[i]; i++;
  slaveData.phase2     = v[i]; i++;
  slaveData.enable1    = (int)v[i]; i++;
  slaveData.enable2    = (int)v[i]; i++;
  endTesting = (int)v[i];
}


void setup() {
  ETout.begin(details(slaveData), &SLAVE_SERIAL);        // Serial comminucation with the slave teensy

  // Setup piezo properties
  resetPiezoProperties();

  // Send the piezo properties to the slave teensy
  ETout.sendData();

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
  pinMode(POT, INPUT);      // Valve rotation potentiometer
  pinMode(HMD1, OUTPUT);    // Heater module 1
  pinMode(HMD2, OUTPUT);    // Heater module 2
  pinMode(HMD3, OUTPUT);    // Heater module 3
  pinMode(HMD4, OUTPUT);    // Heater module 4
  pinMode(HMD5, OUTPUT);    // Heater module 5
  pinMode(RHD, OUTPUT);     // Rope heater
  pinMode(IFT_CS, OUTPUT);  // CS pin for inlet fluid thermocouple

  SPI.begin();              // Begin SPI bus
  Wire.begin();             // Begin I2C bus

  // Setup LCD
  lcd.init();
  lcd.backlight();
  lcd.begin(20, 4);
  lcd.createChar(0, backSlashLCD);
  updateLCD();

  // Serial initialization
  Serial.begin(115200);             // To MATLAB
  SLAVE_SERIAL.begin(115200);       // To Teensy4.0 Slave

  // Initialize data and test timers
  dataStartTime = millis();
  testTimeStart = millis();

  // LCD update interupt
  uint16_t updateDelay;
  uint16_t minDelay = 250;
  if (dataDelay > minDelay){updateDelay = dataDelay;}
  else {updateDelay = minDelay;}
  updateLCDTimer.begin(updateLCD, updateDelay*1000);
  updateLCDTimer.priority(2);

  // Blink interupt
  blinkTimer.begin(blinkLED, blinkDelay*1000);      // Run the blinkLED function at delay speed (ns)
  blinkTimer.priority(1);                           // Priority 0 is highest, 255 is lowest
  
  analogReadResolution(analogResolution);           // Set analog resolution
}

void loop() {
  // Check if time to read and send/log data
  if (millis()-dataStartTime >= dataDelay){
    getData();                                              // Get sensor data
    testTime = (millis() - testTimeStart);                  // Get current test time in milliseconds
    testTimePrint = testTime * 0.001;                       // Current test time in seconds 
    dataStartTime = millis();                               // Restart timer for data
    //sendData();                                             // Send data to MATLAB
    //checkThermalRunaway();                                  // Check that all heating elements are safe
    //saveDataToSD()                                          // Saves the data to a CSV file on the SD card
    //ETout.sendData();
    //Serial.println("Sent to slave");
  }

  // If stop test condition met, stop test
  if (endTesting){
    endTest();
  }

  // Check if new serial in from MATLAB and send that data to slave Teensy for piezo control
  String incomingString;
  if (Serial.available() > 0) {
    incomingString = Serial.readString();
    Serial.print("USB received: "); Serial.println(incomingString);
    decodeMATLABSerial(incomingString.c_str());   // Unpacks incoming string and updates variables
    for (byte i=0; i < 3; i++){
      ETout.sendData();                           // Send updated data to the slave teensy n times to make sure slave got it
      delay(20);
    }
  }
  

  // Control heater modules

}


