/*******************************************************************************
   UW Sheboygan RocksatC 2017 Flight Code
   Peter Aloisi, Bob Aloisi
   Based on RockOn 2016 Workshop Flight Code Revision 44.0.0

   Revision 3.5
   -Ground Mode uses serial print and writes to SD Card
   -Flight Mode only writes to SD card and does not serial print to increase speed
   -Calibration constants retrieved from EEPROM
   Revision by: Peter Aloisi, Bob Aloisi
   Date: 6/1/17
 *******************************************************************************/

// [SENSOR LIST]
/*
   Geiger 1,2,3  ports 2,3,19 (GM Tubes LND 71217, LND 712, LND 71217 (shielded)  
   SD Card   (Chip Select Pin: 53)
   ADXL 377
   BNO055
*/

// MAIN PROGRAM /////////////////////////////////////////////////////////////////
// Libraries //
// The libraries below are needed so
// that we can use functions/variable definitions defined in them.

// The included Arduino libraries installed in main Arduino library folder
   #include <SPI.h>   // Serial Peripheral Interface (SPI) communication library for SPI sensors
   #include <Wire.h>  // Inter Integrated Circuit (I2C) communcation libary for I2C sensors 
   #include <SD.h>    // Secure Digital (SD) function library for the SD card        
   #include <Adafruit_Sensor.h>
   #include <Adafruit_BNO055.h>
   #include <utility/imumaths.h>
   #include <EEPROM.h>

// Global Variables //
   const int LEDA = 48; //defined in blink_pattern_code.h
   const int LEDB = 49; //defined in blink_pattern_code.h
   const int PROG = 47;
   boolean groundMode = 0;

long count1 = 0;
long count2 = 0;
long count3 = 0;

// The included Space Shield libraries installed in sketch folder
   #include "startTasks.h"
   #include "SDCard.h"


// define
// variables for the bno055 senosor, it sets addresses.
#define SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(55);
char bno055_Address = 0x28;
char bno055_axis_map_config_addr = 0x41;
char bno055_axis_map_sign_addr = 0x42;
char bno055_opr_mode_addr = 0x3d;
char remap_config_p2 = 0x24;
char remap_sign_p2 = 0x06;
char Page_ID = 0x07;
char Page_ID_0 = 0x00;
char Page_ID_1 = 0x01;
char acc_config_addr = 0x08;
char acc_config_value = 0x03; //+/- 16G
char OPERATION_MODE_CONFIG = 0x00;
char OPERATION_MODE_AMG = 0x07;
char OPERATION_MODE_NDOF = 0x0c;
char register_data=0;

// [GEIGER]
int geiger_input_A = 2;
int geiger_input_B = 19;
int geiger_input_C = 3;


void setup() {

// [LED_MODES]
   // The built in "pinMode" function tells Arduino whether it should expect
   // to receive inputs or send outputs over different pins
   // LEDA, LEDB, and PROG defined in blink_pattern_code.h
   pinMode(LEDA, OUTPUT); // LEDA variable set to output
   pinMode(LEDB, OUTPUT); // LEDB variable set to output
   pinMode(PROG, INPUT); // PROG variable set to input



// [GROUND/FLIGHT_MODES]

   // If PROG is high, board is in ground mode, else flight mode
   digitalWrite(LEDA, LOW);
   digitalWrite(LEDB, LOW);
   if (digitalRead(PROG)) {
      // In Ground Mode
      // [SERIAL]
      // Starts serial (USB) communication at a 9600 baud rate
     Serial.begin(9600);
     while (!Serial) {;
     // wait for serial port to connect. Needed for native USB port only
     }
      Serial.println("Arduino Started in ground mode");
      // Indicates we are in Ground Mode
      groundMode = 1;
      digitalWrite(LEDA, HIGH);
   }
   else {
      //In flight mode. No Serial printing.
      groundMode = 0;
   }


// [START_I2C]
   //References "Wire.begin" function in the wire.h library above.
   if (groundMode) Serial.println("START WIRE");
   // Go to StartTasks Namespace and start I2C communication for the BNO055 Orientation Sensor
   StartTasks::startI2C();


// [START_Orientation Sensor]

// Starts I2C Orientation Sensor
  /* Initialise the sensor */
     if(!bno.begin()) {
        if (groundMode) {
          Serial.println("Orientation Sensor Test");
    /* There was a problem detecting the BNO055 ... check your connections */
     Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
     while(1);
     }
   }
 
    int eeAddress = 0;
    long bnoID;
    bool foundCalib = false;

    EEPROM.get(eeAddress, bnoID);
  
    adafruit_bno055_offsets_t calibrationData;
    sensor_t sensor;

    /*
    *  Look for the sensor's unique ID at the beginning oF EEPROM.
    *  This isn't foolproof, but it's better than nothing.
    */
    
    bno.getSensor(&sensor);
 
    if (bnoID != sensor.sensor_id)
    {
      if (groundMode) {Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");}
    }
    else
    {
        if (groundMode) {Serial.println("\nFound Calibration for this sensor in EEPROM.");}
        eeAddress += sizeof(long);
        EEPROM.get(eeAddress, calibrationData);

     // if (groundMode) {displaySensorOffsets(calibrationData);

        if (groundMode) {Serial.println("\nRestoring Calibration data to the BNO055...");}
        bno.setSensorOffsets(calibrationData);

        if (groundMode) {Serial.println("\nCalibration data loaded into BNO055");}
        foundCalib = true;
        }

  bno.setExtCrystalUse(true);
  
 

// [START_SPI]
   // Start SPI communication for SD and Flash
   if (groundMode) Serial.println("START SPI");
   // Go to StartTasks Namespace and start SPI communication for the SD
   // and Flash sensors
   StartTasks::startSPI();


// [START_SD]
   // Start SD (Requires SPI to be started)]
   if (groundMode) Serial.println("START SD");
   pinMode(chipSelect, OUTPUT); // Set the Chip Select pin for the SD card to an Output.

// This function will set up the SD card so we can write to it.
   // Also the header of the log file will be written to the file.
   if (SDCardInit()) {
      if (groundMode) Serial.println("SD Card Initialized");
   }
   else {
      if (groundMode) Serial.println("SD Card Init Error");
   }


// [START_GEIGER]
  attachInterrupt(digitalPinToInterrupt(3),countPulse_A,FALLING);
  attachInterrupt(digitalPinToInterrupt(19),countPulse_B,FALLING);
  attachInterrupt(digitalPinToInterrupt(2),countPulse_C,FALLING);

// Startup Complete
   if (groundMode) Serial.println("Startup Complete->Start Loop");
   
}
//END of setup

void loop() {
// [TIME STAMP]
   // millis returns number of milliseconds since program
   // started; writes this data to buffer
   // Clear data string before logging the time
   dataString = "";
   timeStamp = millis();
   dataString = String(timeStamp);
   dataString += ",";

// [ACCELEROMETER_READ]
 // read x, y, and z sensors and append to the string:
  for (int analogPin = 0; analogPin < 3; analogPin++) {
    int sensor = analogRead(analogPin);
    dataString += String(sensor);
    if (analogPin < 3) {
      dataString += ",";
    }
  }

//Orientation Sensor
  sensors_event_t event;
  bno.getEvent(&event);
  dataString += String((float)event.orientation.x);
  dataString += ",";
  dataString += String((float)event.orientation.y);
  dataString += ",";
  dataString += String((float)event.orientation.z);
  dataString += ",";
  
//Accelerometer
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  dataString += String((float)accel.x());
  dataString += ",";
  dataString += String((float)accel.y());
  dataString += ",";
  dataString += String((float)accel.z());
  dataString += ",";

//Gyro
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  dataString += String((float)gyro.x());
  dataString += ",";
  dataString += String((float)gyro.y());
  dataString += ",";
  dataString += String((float)gyro.z());
  dataString += ",";

//Magnetometer
  imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  dataString += String((float)mag.x());
  dataString += ",";
  dataString += String((float)mag.y());
  dataString += ",";
  dataString += String((float)mag.z());
  dataString += ",";
  
//Tempature
  dataString += String((float)bno.getTemp());

// [GIEGER_COUNTER_READ]
dataString += ",";
dataString += String(count1);
dataString += ",";
dataString += String(count2);
dataString += ",";
dataString += String(count3);


// [SD_WRITE]
  // Write data to SD file prepared by dataSD::setupMemory().
  // This appends this data to the end of the file
  if (writeDataToSD()) { 
  //  sdError = false; // If TRUE, write to SDCard and Blink LED B
    if (!groundMode) digitalWrite(LEDA, LOW);
    if (groundMode) digitalWrite(LEDA, HIGH);
    //  Change the State of the LED from OFF->ON, or ON->OFF
       if (ledState == 0) {
          digitalWrite(LEDB, HIGH);
          ledState = 1;
       }
    //  Handle the case where the LED state is now 2 and set it back to a valid LED binary value of 0
       else {
           digitalWrite(LEDB, LOW);
           ledState = 0;
       }
  }
  else {
    // IF False SDCard error, NO write to SDCard and Blink LED A
    //   sdError = true;
    digitalWrite(LEDB, LOW);
    
    if (ledState == 0) {
      digitalWrite(LEDA, HIGH);
      ledState = 1;
    }
    //  Handle the case where the LED state is now 2 and set it back to a valid LED binary value of 0
    else {
      digitalWrite(LEDA, LOW);
      ledState = 0;
    }
  }

// Print data to Serial monitor
if (groundMode) Serial.println(dataString);

count1 = 0;
count2 = 0;
count3 = 0;
delay SAMPLERATE_DELAY_MS;

}
// End of main LOOP

 
// This code reads signals from Geiger tube A and increments count1
void countPulse_A(){
  detachInterrupt(digitalPinToInterrupt(3));
  count1 ++;
  while(digitalRead(3)==0){
  }
  attachInterrupt(digitalPinToInterrupt(3),countPulse_A,FALLING);
 } 

 // This code reads signals from Geiger tube B and increments count2
void countPulse_B(){
  detachInterrupt(digitalPinToInterrupt(19));
  count2 ++;
  while(digitalRead(19)==0){
  }
  attachInterrupt(digitalPinToInterrupt(19),countPulse_B,FALLING);
 } 

 // This code reads signals from Geiger tube C and increments count3
void countPulse_C(){
  detachInterrupt(digitalPinToInterrupt(2));
  count3 ++;
  while(digitalRead(2)==0){
  }
  attachInterrupt(digitalPinToInterrupt(2),countPulse_C,FALLING);
 }  


