
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
#include <OneWire.h>
#include <DallasTemperature.h>
#include <CSV_Parser.h>//Needed for parsing CSV data 



RTC_PCF8523 rtc; // PCF8523 RTC
File dataFile;// Logging file

#define IridiumWire Wire
IridiumSBD modem(IridiumWire);

//Define Pins
const byte led = 13; // Pin 13 LED
const byte donePin = 5; // TPL5110 done pin.
const byte h2oSet = 10; // Power set pin for H2O level sensor
const byte h2oUnset = 11; // Power unset pin for H2O level sensor
const byte H2O_LEVL_PIN = A0; //Analog pin for reading liquid level sensor output
const byte TURB_PIN = A1; //Analog pin for reading turbidity sensor output
const byte IRID_SWITCH = 12; //Pin for switching power to Iridium modem using 4N37 optocoupler
const byte chipSelect = 4; //Chip select pin for MicroSD breakout
const byte tempPin = 6; // One wire pin for communicating with DS18B20 temperature probe 

//Define Global constants, change as required
const float RANGE = 5000.0; // Depth measuring range 5000mm (for water).
const float CURRENT_INIT = 4.10; // Current @ 0mm (unit: mA), can be adjusted for each setup
const float DENSITY_WATER = 1.00;  // Pure water density.
const float H2O_VREF = 3300.0; //Reference voltage, 3.3V for Adalogger M0, measure with multimeter for better accuracy
const float TURB_VREF = 5000.0; //Reference voltage for turbidity sensor, measure with multimeter for better accuracy
const int ANLG_RES = 12; //Desired analog resolution 10,12 or 16.
const float MAX_ANALOG_VAL = 4096.0; // Maximum analog value at 12-bit ADC resolution.
char **filename;//Desired name for data file !!!must be less than equal to 8 char!!!
char **N_str; //Number of ultrasonic reange sensor readings to average.
char **h2otemp; // Binary int, either '0: false' for no water temperture measuremnt, or '1: true'.
char **satcom; // Binary int, either '0: false' for no satallite communication, or '1: true'.
char **cal_slope; // Calibration slope coefficent for converting mV to NTU (i.e., from calibration)
char **cal_intercept; // Calibration intercept term for converting mV to NTU (i.e., from calibration)

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(tempPin);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

/*Function for converting voltage read from turbidity sensor 'turb_volt' to NTU units (from calibration),
   change to observed calbration for setup. !!Note!! With a multimeter adjust PCB potentiometer so that output voltage 
   is ~2.8V when NTU equals zero (i.e., deinoized H2O). Then perform calibration. 
*/

float Volt_to_NTU(float turb_volt_mV,float beta,float intercept)
{

  float ntu = (turb_volt_mV*beta)+intercept;
  
  return ntu;
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
float avgTurb(int n,float beta, float intercept)
{
  float avg_turb = 0.0; //Average turbidity voltage

  for (int i = 0; i < n; i++)
  {

    //Compute average voltage output of turbidity probe (need to establish / verify voltage NTU curve)
    float turb_voltage = analogRead(TURB_PIN) * (TURB_VREF / MAX_ANALOG_VAL); // Convert the analog reading (which goes from 0 - MAX_ANALOG_VAL) to a voltage (0 - 5V):

    avg_turb = avg_turb + turb_voltage;

    delay(100);

  }

  avg_turb = avg_turb / (float)n; //Calculate average voltage of n readings

  avg_turb = Volt_to_NTU(avg_turb,beta,intercept);//Need to convert voltage to NTU here (calibration)

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

  pinMode(IRID_SWITCH, OUTPUT);

  // Start the I2C wire port connected to the PCF8523 RTC
  Wire.begin(0x68, 100000);

  // Start RTC
  rtc.begin();

  //Set analog resolution to ANLG_RES bit
  analogReadResolution(ANLG_RES);

  //Set paramters for parsing the parameter file
  CSV_Parser cp(/*format*/ "ss", /*has_header*/ true, /*delimiter*/ ',');

  //Read the parameter file off SD card (params.csv), 1/4-sec flash means file is not available
  while (!cp.readSDfile("/params.csv"))
  {
    digitalWrite(led, HIGH);
    delay(250);
    digitalWrite(led, LOW);
    delay(250);
  }

  //Read values from SNOW_PARAM.TXT into global varibles
  filename = (char**)cp["filename"];
  N_str = (char**)cp["N"];
  int N = String(N_str[0]).toInt();

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


  cal_slope = (char**)cp["slope"];
  cal_intercept = (char**)cp["intercept"];
  float slope = String(cal_slope[0]).toFloat(); 
  float intercept = String(cal_intercept[0]).toFloat();

  //Uncomment for setting PCB poteniometer
  //delay(60000);
  
  //Get a N average NTU reading
  float turb_ntu = avgTurb(N,slope,intercept);

  //See if temperature measurment is to be made
  h2otemp = (char**)cp["h2o_temp"];

  // NA value = -99.0
  float tempC = -99.0;
  
  //If h20_temp parameter is true make a measurment 
  if(String(h2otemp[0]).toInt() == 1)
  {  
    sensors.requestTemperatures(); // Send the command to get temperatures
    // We use the function ByIndex, and as an example get the temperature from the first (and only) sensor only.
    tempC = sensors.getTempCByIndex(0);
  }
  

  //Assemble a data string for logging to SD, with time and average level NTU values
  String datastring = String(now.year()) + ":" + String(now.month()) + ":" + String(now.day()) + " " + String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second()) + "," + String(depth_mm) + "," + String(turb_ntu) + "," + String(tempC);

  //Write header if first time writing to the file 
  if(!SD.exists(filename[0]))
  {
    dataFile = SD.open(filename[0], FILE_WRITE);
    if (dataFile)
    {
      dataFile.println("datetime,h2o_depth_mm,turbidity_ntu,h2o_temp_degC");
      dataFile.close();
    }

  }
  

  //Write datastring and close logfile on SD card
  dataFile = SD.open(filename[0], FILE_WRITE);
  if (dataFile)
  {
    dataFile.println(datastring);
    dataFile.close();
  }

  //Get satcom parameter from file
  satcom = (char**)cp["satcom"];
  
  //Check that satcoms are being used, the iridium modem is connected and the the clock has just reached midnight (i.e.,current time is within one logging interval of midnight)
  if ((int) now.hour() == 0 && String(satcom[0]).toInt() == 0)
  {

    if (!SD.exists("IRID.CSV"))
    {
      dataFile = SD.open("IRID.CSV", FILE_WRITE);
      dataFile.println("day,day1");
      dataFile.println(String(now.day())+","+String(now.day()));
      dataFile.close();

    }


    CSV_Parser cp(/*format*/ "s-", /*has_header*/ true, /*delimiter*/ ',');

    
    while (!cp.readSDfile("/IRID.CSV"))
    {
      digitalWrite(led, HIGH);
      delay(500);
      digitalWrite(led, LOW);
      delay(500);
    }
    
    char **irid_day = (char**)cp["day"];

    if (String(irid_day[0]).toInt() == (int) now.day())
    {

      //Update IRID.CSV with new day
      SD.remove("IRID.CSV");
      dataFile = SD.open("IRID.CSV", FILE_WRITE);
      dataFile.println("day,day1");
      DateTime next_day = (DateTime(now.year(),now.month(),now.day()) + TimeSpan(1,0,0,0));
      dataFile.println(String(next_day.day())+","+String(next_day.day()));
      dataFile.close();

      //Provide power to Iridium modem via optocoupler
      digitalWrite(IRID_SWITCH,HIGH);
      delay(100);

      modem.enableSuperCapCharger(true); // Enable the super capacitor charger
      while (!modem.checkSuperCapCharger()) ; // Wait for the capacitors to charge
      modem.enable9603Npower(true); // Enable power for the 9603N
      modem.begin(); // Wake up the 9603N and prepare it for communications.
      modem.sendSBDText(datastring.c_str()); // Send datastring message
      modem.sleep(); // Put the modem to sleep
      modem.enable9603Npower(false); // Disable power for the 9603N
      modem.enableSuperCapCharger(false); // Disable the super capacitor charger
      modem.enable841lowPower(true); // Enable the ATtiny841's low power mode (optional)

      digitalWrite(IRID_SWITCH,LOW);
      
    }
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
