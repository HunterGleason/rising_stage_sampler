/***********************************************************
  Author: Hunter Gleason
  Date:June 3, 2021
  Description: This script is a data logging script intended
  for use with rising stage samplers. Using a Adalogger M0
  MCU, a DFRobot Gravity Throw-in type liquid level tansmitter
  and a DFRobot Gravity Analog Turbidity Sensor for reading
  and recording stage and turbidty to a SD card. Note, because
  a voltage divider is used to scale the 5V analog output
  from the turbidity to ~0-3.3V, it is assumed that
  the user provide their own calibration. At the end of each 
  day the average water level and turbidity computed from the
  daily readings is sent over satalitte via the RockBlock modem. 

  Code based on:
  https://wiki.dfrobot.com/Gravity__Analog_Current_to_Voltage_Converter_for_4~20mA_Application__SKU_SEN0262
  https://wiki.dfrobot.com/Throw-in_Type_Liquid_Level_Transmitter_SKU_KIT0139
  https://www.arduino.cc/en/Tutorial/SleepRTCAlarm
  https://github.com/mikalhart/IridiumSBD/blob/master/examples/Sleep/Sleep.ino

  See wiring diagram:
  https://github.com/HunterGleason/rising_stage_sampler/blob/with_turb/siphon_sampler.svg

 ****************************************************/

/*PINS
   Liquid level sensor analog pin -> A3
   Turbidty sensor analog pin -> A4
   Liquid level sensor switch pin -> D9
   Turbidty sensor switch pin -> D6
   RockBlock RX -> RX
   RockBlock TX -> TX
   RockBlock OnOff -> D10

*/


//Load required libriries
#include <SPI.h>
#include <SD.h>
#include <RTCZero.h>
#include <ArduinoLowPower.h>
#include <IridiumSBD.h>
#include <time.h>

//Define pins
const byte H2O_LEVL_PIN = A3; //Define pin for reading liquid level sensor
const byte TURB_PIN = A4; //Define pin for reading turbidity sensor
const byte H2O_LEVL_SWITCH = 9; //Attach transister gate to digtial pin 10 for switching power level sensor
const byte TURB_SWITCH = 6; //Attach optocoupler
const byte ROCKBLOCK_SWITCH = 10; //Attach RockBlock OnOff pin to D10
const byte RED_LED = 13; //Turn of built in LED to conserve power
const byte chipSelect = 4;  //** CS - pin 4 (for MKRZero SD: SDCARD_SS_PIN)

//Define Global constants
const int RANGE = 5000; // Depth measuring range 5000mm (for water)
const double CURRENT_INIT = 4.00; // Current @ 0mm (uint: mA), may need to adjust ...
const double DENSITY_WATER = 1.00;  // Pure water density
const int H2O_VREF = 3300; //Refrence voltage, 3.3V for Adalogger M0
const int TURB_VREF = 5000; //Refrence voltage, 5.0 for turbidity sensor (output from usb pin)
const int MAX_ANALOG_VAL = 4096; // Maximum analog value at provided ADC resolution

#define IridiumSerial Serial1 // Serial for communicating with RockBlock
#define DIAGNOSTICS true // Change this to see diagnostics

//Define Global variables
const String filename = "HAY_CREEK.TXT";//Desired name for logfile, change as needed.

// Change these values to set the current initial time
const byte hours = 10;
const byte minutes = 30;
const byte seconds = 00;

// Change these values to set the current initial date
const byte day = 23;
const byte month = 7;
const byte year = 21;

const int alarmIncMin = 5;//Number of minutes between sleep / read / log cycles, change as needed

const int N = 3;//Number of sensor readings to average, change as needed

bool matched = false;//Boolean variable for indicating alarm match
bool daily_satcom = false;//Boolean varialbe indicating if daily satilite communication has been sent 

//Function for converting voltage read from turbidty sensor 'turb_volt' to NTU units (from calibration), change as needed.
float Volt_to_NTU(float turb_volt)
{
  return turb_volt;
}

//Vars for computing daily average statistics for posting over satallite
float DAY_AVG_LEVL=0.0;
float DAY_AVG_TURB=0.0;
int DAY_AVG_N=0;



//Instantiate rtc, dataFile and IridiumSBD
RTCZero rtc;
File dataFile;
IridiumSBD modem(IridiumSerial, ROCKBLOCK_SWITCH);



//Runs Once
void setup()
{
  delay(10000);

  int signalQuality = -1;
  int err;

  // Start the console serial port
  Serial.begin(115200);
  while (!Serial);

  // Start the serial port connected to the satellite modem
  IridiumSerial.begin(19200);

  // Begin satellite modem operation
  Serial.println("Starting modem...");
  err = modem.begin();
  if (err != ISBD_SUCCESS)
  {
    Serial.print("Begin failed: error ");
    Serial.println(err);
    if (err == ISBD_NO_MODEM_DETECTED)
      Serial.println("No modem detected: check wiring.");
    return;
  }

  // Example: Test the signal quality.
  // This returns a number between 0 and 5.
  // 2 or better is preferred.
  err = modem.getSignalQuality(signalQuality);
  if (err != ISBD_SUCCESS)
  {
    Serial.print("SignalQuality failed: error ");
    Serial.println(err);
    return;
  }
  Serial.print("On a scale of 0 to 5, signal quality is currently ");
  Serial.print(signalQuality);
  Serial.println(".");

  struct tm t;
  int err = modem.getSystemTime(t);
  if (err == ISBD_SUCCESS)
  {
    char buf[32];
    sprintf(buf, "%d-%02d-%02d %02d:%02d:%02d", t.tm_year + 1900, t.tm_mon + 1, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec);
    Serial.print("Iridium time/date is ");
    Serial.println(buf);
  }

  else if (err == ISBD_NO_NETWORK)
  {
    Serial.println("No network detected.  Waiting 10 seconds.");
  }

  else
  {
    Serial.print("Unexpected error ");
    Serial.println(err);
    return;
  }


  rtc.begin();    // Start the RTC in 24hr mode UTC from iridium network time
  rtc.setTime(t.tm_hour, t.tm_min, t.tm_sec);  // Set the time
  rtc.setDate(t.tm_mday, t.tm_mon + 1, (t.tm_year + 1900) - 100); // Set the date

  rtc.setAlarmTime(hours, minutes + alarmIncMin , seconds); //Set the RTC alarm
  rtc.enableAlarm(rtc.MATCH_MMSS);

  //Attach an alarm interupt routine
  rtc.attachInterrupt(alarmMatch);

  // Clear the Mobile Originated message buffer
  Serial.println(F("Clearing the MO buffer."));
  err = modem.clearBuffers(ISBD_CLEAR_MO); // Clear MO buffer
  if (err != ISBD_SUCCESS)
  {
    Serial.print(F("clearBuffers failed: error "));
    Serial.println(err);
  }

  // Put modem to sleep
  Serial.println(F("Putting modem to sleep."));
  err = modem.sleep();
  if (err != ISBD_SUCCESS)
  {
    Serial.print(F("sleep failed: error "));
    Serial.println(err);
  }

  //Set analog resolution to 12 bit
  analogReadResolution(12);

  //Initlize water level input pin as input
  pinMode(H2O_LEVL_PIN, INPUT);

  //Initlize turbidity input pin as input
  pinMode(TURB_PIN, INPUT);

  //Initlize water level switch pin as output and switch off 12V power until next reading
  pinMode(H2O_LEVL_SWITCH, OUTPUT);
  digitalWrite(H2O_LEVL_SWITCH, LOW);

  //Initilize turbidity switch pin as output and turn off 5V from USB pin until next reading
  pinMode(TURB_SWITCH, OUTPUT);
  digitalWrite(TURB_SWITCH, LOW);

  //Turn off RED LED to save power
  pinMode(RED_LED, OUTPUT);
  digitalWrite(RED_LED, LOW);


  //See if the SD card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    // don't do anything more:
    while (1);
    Serial.println("SD Card failed to initialize ...");
  }

  Serial.end();

  //Write header to logfile
  dataFile = SD.open(filename, FILE_WRITE);
  dataFile.println("DateTime,Level_mm,NTU");
  dataFile.close();

  //Set Adalogger into low power standby mode
  rtc.standbyMode();
}

//Runs repeatedly
void loop()
{
  //If alarm time has been matched
  if (matched)
  {
    //Reset 'matched' bool
    matched == false;

    if (rtc.getHours() == 24 && daily_satcom == false)
    {

      DAY_AVG_LEVL = DAY_AVG_LEVL / (float)DAY_AVG_N;
      DAY_AVG_TURB = DAY_AVG_TURB / (float)DAY_AVG_N;
      
      String datastring = String(DAY_AVG_LEVL) + " mm ," + String(DAY_AVG_TURB) + "NTU";
      
      modem.begin();

      modem.sendSBDText(datastring);

      modem.sleep();

      //Reset 'daily_satcom' bool
      daily_satcom == true

      //Reset average vars
      DAY_AVG_LEVL = 0.0;
      DAY_AVG_TURB = 0.0;
      DAY_AVG_N = 0;
    }

    //Provide 12V on the water level sensor via 4N35 optocoupler
    digitalWrite(H2O_LEVL_SWITCH, HIGH);

    //Allow time for water level sensor to stabalize (not sure what best duration is for this, check docs sheet?)
    delay(1000);

    //Get an N average depth reading
    float water_depth_mm = avgWaterLevl(N);

    DAY_AVG_LEVL = DAY_AVG_LEVEL + water_depth_mm;

    //Switch off 12V to water level sensor to save battery
    digitalWrite(H2O_LEVL_SWITCH, LOW);


    //Provide 5V on the turbidity sensor through 4N35 optocoupler
    digitalWrite(TURB_SWITCH, HIGH);

    //Allow turbidty sensor to stabalize, datasheet indicates 500 ms is enough.
    delay(1000);

    float turb_ntu = avgTurb(N);

    DAY_AVG_TURB = DAY_AVG_TURB + turb_ntu;
    DAY_AVG_N =  DAY_AVG_N + 1;

    //Switch off 5V to turbidty probe to save battery
    digitalWrite(TURB_SWITCH, LOW);

    //Assemble data string to write to SD card
    String datastring = String(rtc.getDay()) + "-" + String(rtc.getMonth()) + "-" + String(rtc.getYear()) + " " + String(rtc.getHours()) + ":" + String(rtc.getMinutes()) + ":" + String(rtc.getSeconds()) + "," + String(water_depth_mm) + "," + String(turb_ntu);

    //Write datastring and close logfile on SD card
    dataFile = SD.open(filename, FILE_WRITE);
    if (dataFile)
    {
      dataFile.println(datastring);
      dataFile.close();
    }

    //Set next rtc wakeup alarm
    int alarmMinutes = rtc.getMinutes();
    alarmMinutes += alarmIncMin;
    if (alarmMinutes >= 60) {
      alarmMinutes -= 60;
    }
    
    rtc.setAlarmTime(rtc.getHours(), alarmMinutes, rtc.getSeconds());

    rtc.standbyMode();// Low power sleep until next alarm match
  }
}

//Void function for changing state of 'matched'
void alarmMatch() {
  matched = true;
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

    delay(1000);

  }

  avg_depth = avg_depth / (float)n;

  return avg_depth;

}

//Function for obtaining mean turbidty from n sensor readings
float avgTurb(int n)
{
  float avg_turb = 0.0; //Average turbidty voltage

  for (int i = 0; i < n; i++)
  {

    //Compute average voltage output of turbity probe (need to establish / verify voltage NTU curve)
    float turb_voltage = analogRead(TURB_PIN) * (TURB_VREF / MAX_ANALOG_VAL); // Convert the analog reading (which goes from 0 - 4096) to a voltage (0 - 5V):

    avg_turb = avg_turb + turb_voltage;

    delay(500);

  }

  avg_turb = avg_turb / (float)n; //Calculate average voltage of n readings

  avg_turb = Volt_to_NTU(avg_turb);//Need to convert voltage to NTU here (calibration)

  return avg_turb;

}
