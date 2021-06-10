/***********************************************************
Author: Hunter Gleason
Date:June 3, 2021
Description: This script is a data logging script intended
for use with rising stage samplers. Using a Adalogger M0
MCU, a DFRobot Gravity Throw-in type liquid level tansmitter 
and a DFRobot Gravity Analog Turbidity Sensor for reading 
and recording stage and turbidty to a SD card. 12V power supply
to liqid level sensor is switched using a transitor with gate
wired to digital pin 10. 
Code based on:
https://wiki.dfrobot.com/Gravity__Analog_Current_to_Voltage_Converter_for_4~20mA_Application__SKU_SEN0262
https://wiki.dfrobot.com/Throw-in_Type_Liquid_Level_Transmitter_SKU_KIT0139
https://www.arduino.cc/en/Tutorial/SleepRTCAlarm
 ****************************************************/

/*PINS
 * Liquid level sensor analog pin -> A1
 * Turbidty sensor analog pin -> A2
 * MOSFET Gate -> D10
 */

 
//Load required libriries 
#include <SPI.h>
#include <SD.h>
#include <RTCZero.h>
#include <ArduinoLowPower.h>

//Define pins 
const byte H2O_LEVL_PIN = A1; //Define pin for reading liquid level sensor
const byte TURB_PIN = A2; //Define pin for reading turbidity sensor 
const byte H2O_LEVL_SWITCH = 10; //Attach transister gate to digtial pin 10 for switching power level sensor 
const byte RED_LED = 13; //Turn of built in LED to conserve power 
const byte chipSelect = 4;  //** CS - pin 4 (for MKRZero SD: SDCARD_SS_PIN)

//Define Global constants 
const int RANGE = 5000; // Depth measuring range 5000mm (for water)
const double CURRENT_INIT = 4.00; // Current @ 0mm (uint: mA)
const double DENSITY_WATER = 1.00;  // Pure water density
const int VREF = 3300; //Refrence voltage, 3.3V for Adalogger M0

const String filename = "data_log.txt"; //Desired name for logfile, change as needed.

/* Change these values to set the current initial time */
const byte hours = 11;
const byte minutes = 12;
const byte seconds = 0;
/* Change these values to set the current initial date */
const byte day = 8;
const byte month = 6;
const byte year = 21;

const int alarmIncMin = 1; //Number of minutes between sleep / read / log cycles, change as needed

//Define Global varibles
bool matched = false; //Boolean variable for indicating alarm match 

int dataVoltage; //Varible for holding analog voltage readings 
float dataCurrent;//Varible for current reading unit:mA
float depth; //Varible for liquid depth reading unit:mm

//Instantiate rtc and dataFile 
RTCZero rtc;
File dataFile;

//Function reads DFRobot Liquid level sensor n times and returns average depth (+-25 mm)
float readLiqdLevel(int n)
{

  float avg_depth = 0.0;

  for (int i = 0; i < n; i++)
  {
    dataVoltage = analogRead(H2O_LEVL_PIN) / 1024.0 * VREF;
    dataCurrent = dataVoltage / 120.0; //Sense Resistor:120ohm
    depth = (dataCurrent - CURRENT_INIT) * (RANGE / DENSITY_WATER / 16.0); //Calculate depth from current readings

    avg_depth = avg_depth + depth;

    delay(10);
  }

  avg_depth = avg_depth / n;
  if (avg_depth < 0.0)
  {
    depth = 0.0;
  }

  return avg_depth;
}

//Runs Once 
void setup()
{

  rtc.begin();    // Start the RTC in 24hr mode
  rtc.setTime(hours, minutes, seconds);   // Set the time
  rtc.setDate(day, month, year);    // Set the date

  rtc.setAlarmTime(hours, minutes + alarmIncMin , seconds); //Set the RTC alarm 
  rtc.enableAlarm(rtc.MATCH_MMSS);

  //Attach an alarm interupt routine 
  rtc.attachInterrupt(alarmMatch);

  //Initlize H2O_LEVL_PIN as input
  pinMode(H2O_LEVL_PIN, INPUT);

  //Initlize H2O_LEVL_SWITCH as output and switch off 12V power until next reading 
  pinMode(H2O_LEVL_SWITCH, OUTPUT);
  digitalWrite(H2O_LEVL_SWITCH, LOW);

  //Turn off RED LED to save power
  pinMode(RED_LED, OUTPUT);
  digitalWrite(RED_LED, LOW);


  // see if the SD card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    // don't do anything more:
    while (1);
  }

  //Write header to logfile
  dataFile = SD.open(filename, FILE_WRITE);
  dataFile.println("DateTime,Level_mm,NTU");
  dataFile.close();

  //Go into standby
  rtc.standbyMode();
}

//Runs repeatedly
void loop()
{
  //If alarm time has been matched 
  if (matched)
  {
    //Reset 'matched' alarm
    matched == false;

    //Provide 12V on the liquid level sensor
    digitalWrite(H2O_LEVL_SWITCH, HIGH);

    //Allow time for liquid level sensor to stabalize (not sure what best duration is for this, check docs sheet?)
    delay(1000);

    //Take a average of 5 consecutive readings
    float cur_depth_mm = readLiqdLevel(5);

    //Kill 12V to liquid level sensor to save battery 
    digitalWrite(H2O_LEVL_SWITCH, LOW);

    //Data string to write to SD card
    String datastring = String(rtc.getDay()) + "-" + String(rtc.getMonth()) + "-" + String(rtc.getYear()) + " " + String(rtc.getHours()) + ":" + String(rtc.getMinutes()) + ":" + String(rtc.getSeconds()) + "," + String(cur_depth_mm);

    //Write and close logfile on SD card
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
    
    rtc.standbyMode();// Sleep until next alarm match
  }
}

//Void function for changing state of 'matched' 
void alarmMatch() {
  matched = true;
}
