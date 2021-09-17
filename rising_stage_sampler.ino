
/*
  rising_stage_sampler.ino

  Simple script for logging water level using DFRobots Gravity pressure transducer
  and Turbidity sensor. Written for use with Adalogger M0 MCU. Power managment is
  carried out using the SparkFun TI TPL5110 breakout, be sure set the desired logging
  interval [https://www.sparkfun.com/products/15353]. Water level and Turbidty values
  are logged to a Micro SD card. See 'siphon_sampler.svg' for wireing diagram. User must
  provide logging file name, number of samples to average and a voltage to NTU calibration
  function.
  FLNRORD
  Date: Sept, 2021
  Author: Hunter Gleason
*/

//Include required libraries
#include <IridiumSBD.h> // Click here to get the library: http://librarymanager/All#IridiumSBDI2C
#include <Wire.h> //Needed for I2C communication
#include "RTClib.h" //Needed for communication with Real Time Clock
#include <SPI.h>//Needed for working with SD card
#include <SD.h>//Needed for working with SD card



RTC_PCF8523 rtc; //Instantiate PCF8523 RTC

File dataFile;//instantiate a logging file


//Define Pins
int led = 13; // Pin 13 LED
int donePin = 5; // TPL5110 done pin.
int h2oSet = 10; // Power set pin for H2O level sensor
const int h2oUnset = 11; // Power unset pin for H2O level sensor
const byte H2O_LEVL_PIN = A0; //Analog pin for reading liquid level sensor output
const byte TURB_PIN = A1; //Analog pin for reading turbidity sensor output
const int TURB_SWITCH = 12; //Pin for switching power to turbidity sensor using 4N37 optocoupler
const byte chipSelect = 4; //Chip select pin for MicroSD breakout

//Define Global constants, change as required
const float RANGE = 5000.0; // Depth measuring range 5000mm (for water).
const float CURRENT_INIT = 4.10; // Current @ 0mm (unit: mA), can be adjusted for each setup
const float DENSITY_WATER = 1.00;  // Pure water density.
const float H2O_VREF = 3300.0; //Reference voltage, 3.3V for Adalogger M0, measure with multimeter for better accuracy
const float TURB_VREF = 5000.0; //Reference voltage for turbidity sensor, measure with multimeter for better accuracy
const int ANLG_RES = 12; //Desired analog resolution 10,12 or 16.
const float MAX_ANALOG_VAL = 4096.0; // Maximum analog value at specified ADC resolution.
const String filename = "LOGFILE.TXT";//Desired name for logfile !!!must be less than 8 char!!!
const int N = 5; //Number of sensor readings to average.

/*Function for converting voltage read from turbidity sensor 'turb_volt' to NTU units (from calibration),
   change to observed calbration for setup. With a multimeter adjust PCB potentiometer so that output voltage 
   is 3.0V when NTU equals zero.
*/

float Volt_to_NTU(float turb_volt)
{
  return turb_volt;
}


//Function for obtaining mean water level from n sensor readings
float avgWaterLevl(int n)
{
  float avg_depth = 0.0;

  for (int i = 0; i < n; i++)
  {
    //Read voltage output of H2O level sensor
    float level_voltage =  analogRead(H2O_LEVL_PIN) * (H2O_VREF / MAX_ANALOG_VAL);

    //Convert to current
    float level_current = level_voltage / 120.0; //Sense Resistor:120ohm

    //Calculate water depth (mm) from current readings (see datasheet)
    float depth = (level_current - CURRENT_INIT) * (RANGE / DENSITY_WATER / 16.0);

    avg_depth = avg_depth + depth;

    delay(100);

  }

  avg_depth = avg_depth / (float)n;

  return avg_depth;

}


//Function for obtaining mean turbidity from n sensor readings
float avgTurb(int n)
{
  float avg_turb = 0.0; //Average turbidity voltage

  for (int i = 0; i < n; i++)
  {

    //Compute average voltage output of turbidity probe (need to establish / verify voltage NTU curve)
    float turb_voltage = analogRead(TURB_PIN) * (TURB_VREF / MAX_ANALOG_VAL); // Convert the analog reading (which goes from 0 - MAX_ANALOG_VAL) to a voltage (0 - 5V):

    avg_turb = avg_turb + turb_voltage;

    delay(50);

  }

  avg_turb = avg_turb / (float)n; //Calculate average voltage of n readings

  avg_turb = Volt_to_NTU(avg_turb);//Need to convert voltage to NTU here (calibration)

  return avg_turb;

}

//Runs once
void setup() {

  //Make sure a SD is available, otherwise blink onboard LED
  while (!SD.begin(chipSelect)) {
    digitalWrite(led, HIGH);
    delay(500);
    digitalWrite(led, LOW);
  }

  //Set correct pin modes
  pinMode(led, OUTPUT);
  pinMode(donePin, OUTPUT);

  pinMode(h2oSet, OUTPUT);
  pinMode(h2oUnset, OUTPUT);

  pinMode(TURB_SWITCH, OUTPUT);

  // Start the I2C wire port connected to the PCF8523 RTC
  Wire.begin(0x68, 100000);

  // Start RTC
  rtc.begin();

  //Set analog resolution to ANLG_RES bit
  analogReadResolution(ANLG_RES);

  //Get current logging time from RTC
  DateTime now = rtc.now();

  //Set the latching relay for level logger
  digitalWrite(h2oSet, HIGH);
  delay(10);
  //Set low to save power (latching)
  digitalWrite(h2oSet, LOW);

  //Minimum delay of 1-sec to let level sensor stabalize
  delay(1000);

  //Get a N average H2O level reading
  float depth_mm = avgWaterLevl(N);

  //Unset power to level sensor
  digitalWrite(h2oUnset, HIGH);
  delay(10);
  //Set low to save power (latching)
  digitalWrite(h2oUnset, LOW);

  //Set 5V power to turbidity sensor
  digitalWrite(TURB_SWITCH, HIGH);

  //Minimum of 500 ms for turbidity sensor to stabalize
  delay(600);

  //Get a N average NTU reading
  float turb_ntu = avgTurb(N);

  //Unset 5V power to turbidity sensor
  digitalWrite(TURB_SWITCH, LOW);

  //Assemble a data string for logging to SD, with time and average level NTU values
  String datastring = String(now.year()) + ":" + String(now.month()) + ":" + String(now.day()) + " " + String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second()) + "," + String(depth_mm) + " mm ," + String(turb_ntu) + "NTU";

  //Write datastring and close logfile on SD card
  dataFile = SD.open(filename, FILE_WRITE);
  if (dataFile)
  {
    dataFile.println(datastring);
    dataFile.close();
  }

}

void loop() {

  // We're done!
  // It's important that the donePin is written LOW and THEN HIGH. This shift
  // from low to HIGH is how the TPL5110 Nano Power Timer knows to turn off the
  // microcontroller.
  digitalWrite(donePin, LOW);
  digitalWrite(donePin, HIGH);
  delay(10);

}
