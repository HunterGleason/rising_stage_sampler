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
  the user provide their own calibration. 
  
  Code based on:
  https://wiki.dfrobot.com/Gravity__Analog_Current_to_Voltage_Converter_for_4~20mA_Application__SKU_SEN0262
  https://wiki.dfrobot.com/Throw-in_Type_Liquid_Level_Transmitter_SKU_KIT0139
  https://www.arduino.cc/en/Tutorial/SleepRTCAlarm

  See wiring diagram:
  https://github.com/HunterGleason/rising_stage_sampler/blob/with_turb/siphon_sampler.svg 
  
 ****************************************************/

/*PINS
   Liquid level sensor analog pin -> A3
   Turbidty sensor analog pin -> A4
   Liquid level sensor switch pin -> D9
   Turbidty sensor switch pin -> D6
*/


//Load required libriries
#include <RTCZero.h>
#include <ArduinoLowPower.h>
#include <SPI.h>
#include <SD.h>

//Define pins
const byte H2O_LEVL_PIN = A3; //Define pin for reading liquid level sensor
const byte TURB_PIN = A4; //Define pin for reading turbidity sensor
const byte H2O_LEVL_SWITCH = 9; //Attach transister gate to digtial pin 10 for switching power level sensor
const byte TURB_SWITCH = 6; //Attach optocoupler
const byte RED_LED = 13; //Turn of built in LED to conserve power
const int chipSelect = 4;  //** CS - pin 4 (for MKRZero SD: SDCARD_SS_PIN)

//Define Global constants
const float RANGE = 5000.0; // Depth measuring range 5000mm (for water)
const float CURRENT_INIT = 4.00; // Current @ 0mm (uint: mA), may need to adjust ...
const float DENSITY_WATER = 1.00;  // Pure water density
const float H2O_VREF = 3300.0; //Refrence voltage, 3.3V for Adalogger M0
const float TURB_VREF = 5000.0; //Refrence voltage, 5.0 for turbidity sensor (output from usb pin)
const float MAX_ANALOG_VAL = 4096.0; // Maximum analog value at provided ADC resolution  


//Define Global variables
const String filename = "anzac_site2.txt";//Desired name for logfile, change as needed.

// Change these values to set the current initial time
const byte hours = 10;
const byte minutes = 30;
const byte seconds = 00;

// Change these values to set the current initial date
const byte day = 23;
const byte month = 7;
const byte year = 21;

const int alarmIncMin = 1;//Number of minutes between sleep / read / log cycles, change as needed

const int N = 5;//Number of sensor readings to average, change as needed 

bool matched = false;//Boolean variable for indicating alarm match

//Instantiate rtc and dataFile
RTCZero rtc;

//Runs Once
void setup()
{
  delay(20000);

  rtc.begin();    // Start the RTC in 24hr mode
  rtc.setTime(hours, minutes, seconds);   // Set the time
  rtc.setDate(day, month, year);    // Set the date

  rtc.setAlarmTime(hours, minutes + alarmIncMin , seconds); //Set the RTC alarm
  rtc.enableAlarm(rtc.MATCH_MMSS);

  //Attach an alarm interupt routine
  rtc.attachInterrupt(alarmMatch);

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



  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }

  //Write header to logfile
  File dataFile = SD.open(filename, FILE_WRITE);
  dataFile.println("DateTime,Level_mm,NTU");
  dataFile.close();

  //Turn off RED LED to save power
  pinMode(RED_LED, OUTPUT);
  digitalWrite(RED_LED, LOW);

  //Set Adalogger into low power standby mode
  rtc.standbyMode();
}

////Runs repeatedly
//void loop()
//{
//  //If alarm time has been matched
//  if (matched)
//  {
//    //Reset 'matched' alarm
//    matched == false;
//
//    //Provide 12V on the water level sensor via 4N35 optocoupler
//    digitalWrite(H2O_LEVL_SWITCH, HIGH);
//
//    //Allow time for water level sensor to stabalize (not sure what best duration is for this, check docs sheet?)
//    delay(1000);
//
//    //Get an N average depth reading 
//    float water_depth_mm = avgWaterLevl(N);
//
//    //Switch off 12V to water level sensor to save battery
//    digitalWrite(H2O_LEVL_SWITCH, LOW);
//
//
//    //Provide 5V on the turbidity sensor through 4N35 optocoupler
//    digitalWrite(TURB_SWITCH, HIGH);
//
//    //Allow turbidty sensor to stabalize, datasheet indicates 500 ms is enough.
//    delay(1000);
//
//    float turb_ntu = avgTurb(N);
//
//    //Switch off 5V to turbidty probe to save battery
//    digitalWrite(TURB_SWITCH, LOW);
//    
//    //Assemble data string to write to SD card
//    String dataString = String(rtc.getDay()) + "-" + String(rtc.getMonth()) + "-" + String(rtc.getYear()) + " " + String(rtc.getHours()) + ":" + String(rtc.getMinutes()) + ":" + String(rtc.getSeconds()) + "," + String(water_depth_mm) + "," + String(turb_ntu);
//    
//    //Write datastring and close logfile on SD card
//    File dataFile = SD.open(filename, FILE_WRITE);
//
//    dataFile.println("Hmmm");
//    dataFile.close();
//    
//
//    //Set next rtc wakeup alarm
//    int alarmMinutes = rtc.getMinutes();
//    alarmMinutes += alarmIncMin;
//    if (alarmMinutes >= 60) {
//      alarmMinutes -= 60;
//    }
//    rtc.setAlarmTime(rtc.getHours(), alarmMinutes, rtc.getSeconds());
//
//    rtc.standbyMode();// Low power sleep until next alarm match
//  }
//}
//
////Void function for changing state of 'matched'
void alarmMatch() {
  matched = true;
}
//
////Function for converting voltage read from turbidty sensor 'turb_volt' to NTU units (from calibration), change as needed. 
//float Volt_to_NTU(float turb_volt)
//{
//  return turb_volt;
//}
//
////Function for obtaining mean water level from n sensor readings
//float avgWaterLevl(int n)
//{
//  float avg_depth = 0.0;
//
//  for (int i = 0; i < n; i++)
//  {
//    //Read voltage output of H2O level sensor
//    float level_voltage =  analogRead(H2O_LEVL_PIN) * (H2O_VREF / MAX_ANALOG_VAL);
//
//    //Convert to current
//    float level_current = level_voltage / 120.0; //Sense Resistor:120ohm
//
//    //Calculate water depth (mm) from current readings (see datasheet)
//    float depth = (level_current - CURRENT_INIT) * (RANGE / DENSITY_WATER / 16.0);
//
//    avg_depth = avg_depth + depth;
//
//    delay(1000);
//
//  }
//
//  avg_depth = avg_depth / (float) n;
//
//  return avg_depth;
//
//}
//
////Function for obtaining mean turbidty from n sensor readings
//float avgTurb(int n)
//{
//  float avg_turb = 0.0; //Average turbidty voltage
//
//  for (int i = 0; i < n; i++)
//  {
//
//    //Compute average voltage output of turbity probe (need to establish / verify voltage NTU curve)
//    float turb_voltage = analogRead(TURB_PIN) * (TURB_VREF / MAX_ANALOG_VAL); // Convert the analog reading (which goes from 0 - 4096) to a voltage (0 - 5V):
//
//    avg_turb = avg_turb + turb_voltage;
//
//    delay(500);
//
//  }
//
//  avg_turb = avg_turb / (float) n; //Calculate average voltage of n readings
//
//  avg_turb = Volt_to_NTU(avg_turb);//Need to convert voltage to NTU here (calibration) 
//  
//  return avg_turb;
//
//}
