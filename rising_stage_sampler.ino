/***********************************************************
  Author: Hunter Gleason
  Date:June 3, 2021
  Description: This script is a data logging script intended
  for use with rising stage samplers. Using a Adalogger M0
  MCU, a DFRobot Gravity Throw-in type liquid level transmitter
  and a DFRobot Gravity Analog Turbidity Sensor for reading
  and recording stage and turbidity to a SD card. Note, because
  a voltage divider is used to scale the 5V analog output
  from the turbidity to ~0-3.3V, it is assumed that
  the user provide their own NTU calibration. At the end of each
  day the average water level and turbidity computed from the
  daily readings is sent over satellite via the RockBlock modem.

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
   Turbidity sensor analog pin -> A4
   Liquid level sensor switch pin -> D9
   Turbidity sensor switch pin -> D6
   RockBlock RX -> RX
   RockBlock TX -> TX
   RockBlock OnOff -> D10

*/


//Load required libraries
#include <SPI.h>
#include <SD.h>
#include <RTCZero.h>
#include <ArduinoLowPower.h>
#include <IridiumSBD.h>
#include <time.h>


//Define pins
const byte H2O_LEVL_PIN = A3; //Define pin for reading liquid level sensor
const byte TURB_PIN = A4; //Define pin for reading turbidity sensor
const byte H2O_LEVL_SWITCH = 9; //Attach 43N5 optocoupler gate to digital pin 10 for switching power level sensor
const byte TURB_SWITCH = 6; //Attach 43N5 optocoupler gate to digital pin 10 for switching power turbidity sensor
const byte ROCKBLOCK_SWITCH = 10; //Attach RockBlock OnOff pin to D10
const byte RED_LED = 13; //Turn of built in LED to conserve power
const byte chipSelect = 4;  //** CS - pin 4 (for MKRZero SD: SDCARD_SS_PIN)

//Define Global constants, change as required
const float RANGE = 5000.0; // Depth measuring range 5000mm (for water).
const float CURRENT_INIT = 4.19; // Current @ 0mm (unit: mA), may need to adjust ...
const float DENSITY_WATER = 1.00;  // Pure water density.
const float H2O_VREF = 3300.0; //Reference voltage, 3.3V for Adalogger M0.
const float TURB_VREF = 5000.0; //Reference voltage, 5.0 for turbidity sensor (output from usb pin).
const float MAX_ANALOG_VAL = 4096.0; // Maximum analog value at provided ADC resolution.
const String filename = "TST.TXT";//Desired name for logfile.
const int SATCOM_HOURS[] = {0};//24-Hour clock hours for which to send average sensor values over Iridium network.
const int alarmIncMin = 5;//Number of minutes between sleep / read / log cycles.
const int N = 5;//Number of sensor readings to average.


#define IridiumSerial Serial1 // Serial for communicating with RockBlock

//Define Global variables

// Change these values to set the current initial time
//const int hours = 1;
//const int minutes = 30;
//const int seconds = 0;

// Change these values to set the current initial date
//const int day = 13;
//const int month = 8;
//const int year = 21;

volatile bool matched = false;//Boolean variable for indicating alarm match

//Function for converting voltage read from turbidity sensor 'turb_volt' to NTU units (from calibration), change as needed.
float Volt_to_NTU(float turb_volt)
{
  return turb_volt;
}

//Vars for computing daily average statistics for posting over satellite
float AVG_LEVL = 0.0;
float AVG_TURB = 0.0;
int AVG_N = 0;



//Instantiate rtc, dataFile and IridiumSBD
RTCZero rtc;
File dataFile;
IridiumSBD modem(IridiumSerial);


//Runs Once
void setup()
{
  delay(20000);

  pinMode(ROCKBLOCK_SWITCH, OUTPUT);
  digitalWrite(ROCKBLOCK_SWITCH, HIGH);

  int err;

  // Start the serial port connected to the satellite modem
  IridiumSerial.begin(19200);

  // If we're powering the device by USB, tell the library to
  // relax timing constraints waiting for the supercap to recharge.
  modem.setPowerProfile(IridiumSBD::USB_POWER_PROFILE);

  delay(2000);

  // Begin satellite modem operation
  err = modem.begin();

  delay(1000);

  struct tm t;
  err = modem.getSystemTime(t);

  while (!err == ISBD_SUCCESS)
  {
    err = modem.getSystemTime(t);
  }



  rtc.begin();    // Start the RTC in 24hr mode UTC from iridium network time

  //Uncomment if using RockBLOCk to set inital RTC time
  rtc.setTime(t.tm_hour, t.tm_min, t.tm_sec);  // Set the time
  rtc.setDate(t.tm_mday, (t.tm_mon + 1), (t.tm_year - 100)); // Set the date

  //Uncomment if user specifiying intial RTC time
  //  rtc.setTime(hours, minutes, seconds);  // Set the time
  //  rtc.setDate(day, month, year); // Set the date

  rtc.setAlarmTime(rtc.getHours(), rtc.getMinutes() + alarmIncMin , rtc.getSeconds()); //Set the RTC alarm
  rtc.enableAlarm(rtc.MATCH_MMSS);

  //Attach an alarm interrupt routine
  rtc.attachInterrupt(alarmMatch);

  // Kill power to modem

  IridiumSerial.end();
  digitalWrite(ROCKBLOCK_SWITCH, LOW);

  //Set analog resolution to 12 bit
  analogReadResolution(12);

  //Initialize water level input pin as input
  pinMode(H2O_LEVL_PIN, INPUT);

  //Initialize turbidity input pin as input
  pinMode(TURB_PIN, INPUT);

  //Initialize water level switch pin as output and switch off 12V power until next reading
  pinMode(H2O_LEVL_SWITCH, OUTPUT);
  digitalWrite(H2O_LEVL_SWITCH, LOW);

  //Initialize turbidity switch pin as output and turn off 5V from USB pin until next reading
  pinMode(TURB_SWITCH, OUTPUT);
  digitalWrite(TURB_SWITCH, LOW);

  //Turn off RED LED to save power
  pinMode(RED_LED, OUTPUT);
  digitalWrite(RED_LED, LOW);


  //See if the SD card is present and can be initialized, blink LED if not:
  while (!SD.begin(chipSelect)) {
    digitalWrite(RED_LED, HIGH);
    delay(500);
    digitalWrite(RED_LED, LOW);
  }


  //Write header to logfile
  dataFile = SD.open(filename, FILE_WRITE);
  dataFile.println("DateTime,Level_mm,NTU_mV");
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
    //Reset RTC alarm 'matched' bool
    matched == false;

    //If hour matches one of specified sat_com transmit hours, send period average data values
    for (int i = 0; i < (sizeof(SATCOM_HOURS) / sizeof(SATCOM_HOURS[0])); i++)
    {
      if (rtc.getHours() == SATCOM_HOURS[i] && rtc.getMinutes() < (alarmIncMin + 1))
      {
        //Compute period averages
        AVG_LEVL = AVG_LEVL / (float)AVG_N;
        AVG_TURB = AVG_TURB / (float)AVG_N;

        //Assemble datastring for transmission
        String datastring = String(rtc.getYear()) + ":" + String(rtc.getMonth()) + ":" + String(rtc.getDay()) + " " + String(rtc.getHours()) + ":" + String(rtc.getMinutes()) + ":" + String(rtc.getSeconds()) + "," + String(AVG_LEVL) + " mm ," + String(AVG_TURB) + "mV";

        digitalWrite(ROCKBLOCK_SWITCH, HIGH);

        delay(100);

        int err;

        // Start the serial port connected to the satellite modem
        IridiumSerial.begin(19200);

        // If we're powering the device by USB, tell the library to
        // relax timing constraints waiting for the supercap to recharge.
        modem.setPowerProfile(IridiumSBD::USB_POWER_PROFILE);

        //Wake of RockBlock modem
        modem.begin();

        //Send message, light up LED as indicator, will try for 5-min max by default, during which no measurments will be taken
        digitalWrite(RED_LED, HIGH);
        modem.sendSBDText(datastring.c_str());
        digitalWrite(RED_LED, LOW);

        //Put RockBlock modem to sleep

        IridiumSerial.end();
        digitalWrite(ROCKBLOCK_SWITCH, LOW);

        //Reset average variables
        AVG_LEVL = 0.0;
        AVG_TURB = 0.0;
        AVG_N = 0;
      }
    }

    //Provide 12V on the water level sensor via 4N35 optocoupler
    digitalWrite(H2O_LEVL_SWITCH, HIGH);

    //Allow time for water level sensor to stabilize (not sure what best duration is for this, check docs sheet?)
    delay(100);

    //Get an N average depth reading
    float water_depth_mm = avgWaterLevl(N);

    AVG_LEVL = AVG_LEVL + water_depth_mm;

    //Switch off 12V to water level sensor to save battery
    digitalWrite(H2O_LEVL_SWITCH, LOW);


    //Provide 5V on the turbidity sensor through 4N35 optocoupler
    digitalWrite(TURB_SWITCH, HIGH);

    //Allow turbidity sensor to stabilize, datasheet indicates 500 ms is enough.
    delay(50);

    float turb_ntu = avgTurb(N);

    AVG_TURB = AVG_TURB + turb_ntu;
    AVG_N =  AVG_N + 1;

    //Switch off 5V to turbidity probe to save battery
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

    delay(50);

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
    float turb_voltage = analogRead(TURB_PIN) * (TURB_VREF / MAX_ANALOG_VAL); // Convert the analog reading (which goes from 0 - 4096) to a voltage (0 - 5V):

    avg_turb = avg_turb + turb_voltage;

    delay(50);

  }

  avg_turb = avg_turb / (float)n; //Calculate average voltage of n readings

  avg_turb = Volt_to_NTU(avg_turb);//Need to convert voltage to NTU here (calibration)

  return avg_turb;

}
