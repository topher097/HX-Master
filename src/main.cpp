
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
#include <RTClib.h>               // For PCF8523 RTC module
#include <Bounce2.h>              // For reseting piezo properties
#include <PID_v1.h>               // Library for PID control of rope heater

// Hardware serial (UART) to slave
#define SLAVE_SERIAL Serial8

// Header files
#include <coeffDefinitions.h>       // Header file with interpolation function coefficients
#include <pinDefinitions.h>         // Header file with all master teensy pin definitions
#include <varDefinitions.h>         // Header file with all other variables used
#include <calculations.h>           // Header file with calculations of temps and pressures
#include <defaultPiezo.h>           // Header file with the default piezo properties
#include <ropePIDvars.h>            // Header file with PID values for control loop of rope heater

// Namespaces
using namespace varDefinitions;
using namespace pinDefinitions;
using namespace coeffDefinitions;
using namespace defaultPiezoProperties;
using namespace ropePIDvars;
using namespace std;

// EasyTransfer objects
EasyTransfer ETout;

// I2C devices
LiquidCrystal_I2C lcd(0x27,20,4);         // Address for LCD
RTC_PCF8523 rtc;                          // RTC module object

// SPI devices
Adafruit_MAX31855 thermocouple(IFT_CS);   // Hardware SPI implementation

// Timer objects
IntervalTimer blinkTimer;
IntervalTimer updateLCDTimer;

// Bounce button
Bounce restartButton = Bounce();        // Initialate restart button

// PID object for rope heater
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, P_ON_M, DIRECT);

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
    Serial.print(heatEnergyDensity);            Serial.print(',');
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
    Serial.print(targetFluidTemperature);       Serial.print(',');
    Serial.print(slaveData.frequency1);         Serial.print(',');
    Serial.print(slaveData.frequency2);         Serial.print(',');
    Serial.print(slaveData.amplitude1*v_mult);  Serial.print(',');
    Serial.print(slaveData.amplitude2*v_mult);  Serial.print(',');
    Serial.print(slaveData.phase1);             Serial.print(',');
    Serial.print(slaveData.phase2);             Serial.print(',');
    Serial.print(slaveData.enable1);            Serial.print(',');
    Serial.print(slaveData.enable2);            Serial.print(',');
    Serial.print(Kp);                           Serial.print(',');
    Serial.print(Kd);                           Serial.print(',');
    Serial.print(Ki);                           Serial.print(',');
    Serial.print(endTesting);                 
    Serial.write(13);   // Carriage return "CR"
    Serial.write(10);   // Linefeed "LF"
}

// End all data collection and sending and save data
void endTest(){
  digitalWrite(RUNNING, LOW);
  // Turn off piezo 1 and 2
  int tempEnable1 = slaveData.enable1;
  int tempEnable2 = slaveData.enable2;
  slaveData.enable1 = false;
  slaveData.enable2 = false;
  ETout.sendData();
  slaveData.enable1 = tempEnable1;
  slaveData.enable2 = tempEnable2;
}

void startTest(){
  digitalWrite(RUNNING, HIGH);
  ETout.sendData();
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

    // Turn off heaters
    analogWrite(RHD, 0);
    analogWrite(HMD1, 0);
    analogWrite(HMD2, 0);
    analogWrite(HMD3, 0);
    analogWrite(HMD4, 0);
    analogWrite(HMD5, 0);

    // Send data to MATLAB
    enableHeaters = false;
    enableRopeHeater = false;
    targetFluidTemperature = 0;
    heatEnergyDensity = 0;
    sendData();

    // End test
    endTest();
    
    // Run until system is reset, refresh screen every second
    u_int16_t errorTimeStart = millis();
    u_int16_t errorDisplayRefresh = 1000;     // Refresh time in ms
    char loadingSymbolVect[4] = {"|/-"};
    char loadingSybol;
    u_int16_t step = 0;
    while (true){
      //Serial.println(thermocouple.readCelsius());
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
  lcd.print("mL/min, " + (String)(int(percentOpen*100)) + "%   ");
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
  float weight = 0.3;
  inletPressureUpstream = inletPressureUpstream*weight + instantInletPressureUpstream*(1-weight);
  inletPressureUpstream = 45 + rand()/RAND_MAX;         // for testing
  inletPressureDownstream = inletPressureDownstream*weight + instantInletPressureDownstream*(1-weight);
  inletPressureDownstream = 14.5 + rand()/RAND_MAX;     // for testing
  outletPressureVapor = outletPressureVapor*weight + instantOutletPressureVapor*(1-weight);
  outletPressureLiquid = outletPressureLiquid*weight + instantOutletPressureLiquid*(1-weight);

  // Calculate the inlet flow rate
  weight = 0.3;
  int16_t potRead = analogRead(POT);                                        // Get reading from valve potentiometer
  //Serial.println(potRead);
  valveRotation = valveRotation*weight + potRead*(1-weight);                // Take weighted average of pot of reading to smooth
  float instantFlowRate = calcInletFlowRate((float)valveRotation/maxAnalog*3.3, inletPressureUpstream, inletPressureDownstream, valveRotation);    // mL/min
  inletFlowRate = inletFlowRate*weight + instantFlowRate*(1-weight);

  // Read and calculate heater module temps
  float instantHeaterTemperature1 = calcTempHeaterModuleThermistor((float)analogRead(HMT1)/maxAnalog*3.3);     // degree celcius
  
  // Take weighted average of heater temps
  weight = 0.3;
  heaterTemperature1 = heaterTemperature1*weight + instantHeaterTemperature1*(1-weight);
  heaterTemperature2 = heaterTemperature1;
  heaterTemperature3 = heaterTemperature1;
  heaterTemperature4 = heaterTemperature1;
  heaterTemperature5 = heaterTemperature1;
  
  // Read and calculate boil surface temps
  float instantBoilSurfaceTemperature1 = calcTempBoilSurfaceThermistor((float)analogRead(BST1)/maxAnalog*3.3);    // degree celcius
  float instantBoilSurfaceTemperature2 = calcTempBoilSurfaceThermistor((float)analogRead(BST2)/maxAnalog*3.3);    // degree celcius
  float instantBoilSurfaceTemperature3 = calcTempBoilSurfaceThermistor((float)analogRead(BST3)/maxAnalog*3.3);    // degree celcius
  float instantBoilSurfaceTemperature4 = calcTempBoilSurfaceThermistor((float)analogRead(BST4)/maxAnalog*3.3);    // degree celcius

  // Take weighted average of boil surface temps
  weight = 0.3;
  boilSurfaceTemperature1 = boilSurfaceTemperature1*weight + instantBoilSurfaceTemperature1*(1-weight);
  boilSurfaceTemperature2 = boilSurfaceTemperature2*weight + instantBoilSurfaceTemperature2*(1-weight);
  boilSurfaceTemperature3 = boilSurfaceTemperature3*weight + instantBoilSurfaceTemperature3*(1-weight);
  boilSurfaceTemperature4 = boilSurfaceTemperature4*weight + instantBoilSurfaceTemperature4*(1-weight);
  
  // Calculate the average boil surface temperature
  averageBoilSurfaceTemp = (boilSurfaceTemperature1 + boilSurfaceTemperature2 + boilSurfaceTemperature3 + boilSurfaceTemperature4)/4; // degree celcius

  // Read and calcualte inlet fluid temp
  weight = 0.1;
  float instantInletFluidTemperature = thermocouple.readCelsius();            // degree celcius, instant reading
  if (isnan(instantInletFluidTemperature)){
    instantInletFluidTemperature = inletFluidTemperature;                     // If no signal from thermocouple set temp to avg inlet temp
  }
  inletFluidTemperature = inletFluidTemperature*weight + instantInletFluidTemperature*(1-weight);
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
  int i = 0;
  slaveData.enable1       = (int)v[i];    i++;
  slaveData.frequency1    = v[i];         i++;
  slaveData.amplitude1    = v[i]/v_mult;  i++;
  slaveData.phase1        = v[i];         i++;
  slaveData.enable2       = (int)v[i];    i++;
  slaveData.frequency2    = v[i];         i++;
  slaveData.amplitude2    = v[i]/v_mult;  i++;
  slaveData.phase2        = v[i];         i++;
  heatEnergyDensity       = v[i];         i++;
  targetFluidTemperature  = v[i];         i++;
  enableHeaters           = (int)v[i];    i++;
  enableRopeHeater        = (int)v[i];    i++;
  Kp                      = v[i];         i++;
  Kd                      = v[i];         i++;
  Ki                      = v[i];         i++;
  endTesting              = (int)v[i];
}

void setup() {
  ETout.begin(details(slaveData), &SLAVE_SERIAL);     // Serial comminucation with the slave teensy
  resetPiezoProperties();                             // Setup piezo properties
  ETout.sendData();                                   // Send the piezo properties to the slave teensy

  // Initialize pinmodes
  pinMode(BLINK, OUTPUT);   // Status LED
  pinMode(RUNNING, OUTPUT); // Running test LED
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
  uint16_t minDelay = 500;
  if (dataDelay > minDelay){updateDelay = dataDelay;}
  else {updateDelay = minDelay;}
  updateLCDTimer.begin(updateLCD, updateDelay*1000);
  updateLCDTimer.priority(2);

  // Blink interupt
  blinkTimer.begin(blinkLED, blinkDelay*1000);      // Run the blinkLED function at delay speed (ns)
  blinkTimer.priority(1);                           // Priority 0 is highest, 255 is lowest
  
  // Analog setup
  analogReadResolution(analogResolution);           // Set analog resolution
  analogWriteFrequency(RHD, 5);                    // Change PWM frequency to ~1/2*VAC Hz = 30Hz
  analogWriteFrequency(HMD1, 5);                   // Change PWM frequency to ~1/2*VAC Hz = 30Hz
  analogWriteFrequency(HMD2, 5);                   // Change PWM frequency to ~1/2*VAC Hz = 30Hz
  analogWriteFrequency(HMD3, 5);                   // Change PWM frequency to ~1/2*VAC Hz = 30Hz
  analogWriteFrequency(HMD4, 5);                   // Change PWM frequency to ~1/2*VAC Hz = 30Hz
  analogWriteFrequency(HMD5, 5);                   // Change PWM frequency to ~1/2*VAC Hz = 30Hz

  // Turn on the PID controller
  myPID.SetMode(AUTOMATIC);
  myPID.SetTunings(Kp, Ki, Kd);
  Setpoint = targetFluidTemperature;

  // Setup reset piezo properties button
  restartButton.attach(DEFAULT, INPUT);     // Attach the debouncer to the pins
  restartButton.interval(25);               // Bounce delay in ms

  // RTC module for keeping time
  if (! rtc.initialized() || rtc.lostPower()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    DateTime t = DateTime(rtc.now().unixtime()+21);   // Adds 21 seconds to time to adjust for compile+upload time
    rtc.adjust(t);
  }
  rtc.start();
  float drift         = 0;                             // seconds plus or minus over oservation period - set to 0 to cancel previous calibration.
  float period_sec    = (7 * 86400);                    // total obsevation period in seconds (86400 = seconds in 1 day:  7 days = (7 * 86400) seconds )
  float deviation_ppm = (drift / period_sec * 1000000); // deviation in parts per million (μs)
  float drift_unit    = 4.34;                           // use with offset mode PCF8523_TwoHours
  int offset = round(deviation_ppm / drift_unit);
  rtc.calibrate(PCF8523_TwoHours, offset);
}

void loop() {
  // Check if time to read and send/log data
  if (millis()-dataStartTime >= dataDelay){
    getData();                                              // Get sensor data
    testTime = (millis() - testTimeStart);                  // Get current test time in milliseconds
    testTimePrint = testTime * 0.001;                       // Current test time in seconds 
    dataStartTime = millis();                               // Restart timer for data
    sendData();                                             // Send data to MATLAB
    checkThermalRunaway();                                  // Check that all heating elements are safe
  }

  // Start or stop test
  if (tempEnd != endTesting){
    if (endTesting){endTest();}
    else{startTest();}
  }
  tempEnd = endTesting;

  // Check if new serial in from MATLAB and send that data to slave Teensy for piezo control
  string incomingString;
  if (Serial.available() > 5) {
    incomingString = Serial.readStringUntil('?').c_str();
    decodeMATLABSerial(incomingString);           // Unpacks incoming string and updates variables
    for (byte i=0; i < 2; i++){
      ETout.sendData();                           // Send updated data to the slave teensy n times to make sure slave got it
      delay(10);
    }
    Setpoint = targetFluidTemperature;
  }
  
  // Check if reset piezo properties button was pushed
  restartButton.update();
  if (restartButton.fell()){
    resetPiezoProperties();
    ETout.sendData();
  }

  // Control heater modules
  if (enableHeaters && endTesting==0){
    if (heatEnergyDensity > heatEnergyDensityMax){heatEnergyDensity = heatEnergyDensityMax;}
    int PWM_value = map(heatEnergyDensity, 0, heatEnergyDensityMax, 0, 255);    // map(value, fromLow, fromHigh, toLow, toHigh)
    analogWrite(HMD1, PWM_value);
    analogWrite(HMD2, PWM_value);
    analogWrite(HMD3, PWM_value);
    analogWrite(HMD4, PWM_value);
    analogWrite(HMD5, PWM_value);
  }
  else{
    analogWrite(HMD1, 0);   // Turn off the heaters
    analogWrite(HMD2, 0);
    analogWrite(HMD3, 0);
    analogWrite(HMD4, 0);
    analogWrite(HMD5, 0);
  }

  // Control rope heater
  if (enableRopeHeater && endTesting==0){
    float tempDifference = targetFluidTemperature - inletFluidTemperature;
    if (tempDifference >= 7.5){
      analogWrite(RHD, 255);
    }
    else if (tempDifference >= 5){
      analogWrite(RHD, 240);
    }
    else if (tempDifference >= 1){
      analogWrite(RHD, 225);
    }
    else if (tempDifference >= 0.5){
      analogWrite(RHD, 210);
    }
    else if (tempDifference >= 0.25){
      analogWrite(RHD, 200);
    }
    else if (tempDifference >= 0){
      analogWrite(RHD, 175);
    }
    else if (tempDifference >= -0.5){
      analogWrite(RHD, 160);
    }
    else{
      analogWrite(RHD, 0);
    }

      
    // myPID.SetTunings(Kp, Ki, Kd);
    // Setpoint = targetFluidTemperature;
    // Input = inletFluidTemperature;
    // myPID.Compute();
    // analogWrite(RHD, Output);

    // if (targetFluidTemperature > inletFluidTemperature){
    //   analogWrite(RHD, 255);
    // }
    // else{
    //   analogWrite(RHD, 0);
    // }
  }
  else{
    analogWrite(RHD, 0);    // Turn off the rope heater
  }
}


