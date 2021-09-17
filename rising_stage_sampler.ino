
/*
  TPL5110_Blink_Demo_example.ino

  Simple Example Code for the TPL5110 Nano Power Timer Hookup Guide. This code
  simply blinks the pin 13 LED and writes pin 4 (donePin pin) high. This shift from
  LOW to HIGH of the donePin pin, signals to the Nano Power Timer to turn off the
  microcontroller.
  SparkFun Electronics
  Date: May, 2019
  Author: Elias Santistevan
*/

#include <IridiumSBD.h> // Click here to get the library: http://librarymanager/All#IridiumSBDI2C
#include <Wire.h> //Needed for I2C communication
#include "RTClib.h"
#include <SPI.h>
#include <SD.h>



RTC_PCF8523 rtc;

File dataFile;

#define IridiumWire Wire
IridiumSBD modem(IridiumWire);

int led = 13; // Pin 13 LED
int donePin = 5; // Done pin - can be any pin.
int modemSet = 9; // Power set pin for Iridium Modem
int modemUnset = 6; // Power unset pin for Iridium Modem
int h2oSet = 10; // Power set pin for H2O level sensor
const int h2oUnset = 11; // Power unset pin for H2O level sensor
const byte H2O_LEVL_PIN = A0; //Define pin for reading liquid level sensor
const byte TURB_PIN = A1;
const int TURB_SWITCH = 12;
const byte chipSelect = 4; 

//Define Global constants, change as required
const float RANGE = 5000.0; // Depth measuring range 5000mm (for water).
const float CURRENT_INIT = 4.10; // Current @ 0mm (unit: mA)
const float DENSITY_WATER = 1.00;  // Pure water density.
const float H2O_VREF = 3300.0; //Reference voltage, 3.3V for Adalogger M0, measure with multimeter for better accuracy
const float TURB_VREF = 3000.0; //Reference voltage for turbidity sensor when NTU = 0, using mulitmeter adjust potentiometer so that analog output is 3.0 V when NTU equals 0, then finish calibration!
const float MAX_ANALOG_VAL = 4096.0; // Maximum analog value at provided ADC resolution.
const String filename = "YUPPY_1.TXT";//Desired name for logfile !!!must be less than 8 char!!!
const int SATCOM_HOURS[] = {0};//24-Hour clock hours for which to send average sensor values over Iridium network.
const int LoggerIncMin = 5; //Number of minutes between sleep / read / log cycles, should match setting on TPL5110, and be >= 5 min if using Iridium modem
const int N = 5; //Number of sensor readings to average.

//Function for converting voltage read from turbidity sensor 'turb_volt' to NTU units (from calibration), change to observed calbration for setup.
float Volt_to_NTU(float turb_volt)
{
  return turb_volt;
}

void setup() {

  while (!SD.begin(chipSelect)) {
    digitalWrite(led, HIGH);
    delay(500);
    digitalWrite(led, LOW);
  }

  pinMode(led, OUTPUT);
  pinMode(donePin, OUTPUT);

  pinMode(modemSet, OUTPUT);
  pinMode(modemUnset, OUTPUT);

  pinMode(h2oSet, OUTPUT);
  pinMode(h2oUnset, OUTPUT);

  pinMode(TURB_SWITCH, OUTPUT);

  // Start the I2C wire port connected to the satellite modem
  Wire.begin(0x68, 100000);
  Wire.begin(0x63, 400000);

  rtc.begin();

  DateTime now = rtc.now();
  int NOW_HOUR = now.hour();
  int NOW_MINT = now.minute();

  //Set analog resolution to 12 bit
  analogReadResolution(12);
  
  for (int i = 0; i < (sizeof(SATCOM_HOURS) / sizeof(SATCOM_HOURS[0])); i++)
  {
    if ( NOW_HOUR == SATCOM_HOURS[i] && NOW_MINT <= LoggerIncMin )
    {
      if (modem.isConnected()) // Check that the Qwiic Iridium is connected
      {
        //Get an N average depth reading
        float level_voltage =  analogRead(H2O_LEVL_PIN) * (H2O_VREF / MAX_ANALOG_VAL);
        float level_current = level_voltage / 120.0; //Sense Resistor:120ohm
        float depth = (level_current - CURRENT_INIT) * (RANGE / DENSITY_WATER / 16.0);

        //Get an N average NTU reading
        //Compute average voltage output of turbidity probe (need to establish / verify voltage NTU curve)
        float turb_voltage = analogRead(TURB_PIN) * (TURB_VREF / MAX_ANALOG_VAL); // Convert the analog reading (which goes from 0 - 4096) to a voltage (0 - 5V):
        float turb_ntu = Volt_to_NTU(turb_voltage);

        //Assemble datastring for transmission
        String datastring = String(now.year()) + ":" + String(now.month()) + ":" + String(now.day()) + " " + String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second()) + "," + String(depth) + " mm ," + String(turb_ntu) + "mV";


        digitalWrite(modemSet, HIGH); // Flip latching relay power to on
        delay(10);
        digitalWrite(modemSet, LOW); // Latched, so set low to save power

        //        modem.enableSuperCapCharger(true); // Enable the super capacitor charger
        //        while (!modem.checkSuperCapCharger()) ; // Wait for the capacitors to charge
        //        modem.enable9603Npower(true); // Enable power for the 9603N
        //        modem.setPowerProfile(IridiumSBD::USB_POWER_PROFILE); // Assume 'USB' power (slow recharge)
        //        modem.begin(); // Wake up the modem
        //        modem.sendSBDText(datastring.c_str()); // Send a message
        //        modem.sleep(); // Put the modem to sleep
        //        modem.enable9603Npower(false); // Disable power for the 9603N
        //        modem.enableSuperCapCharger(false); // Disable the super capacitor charger

        delay(4000);

        digitalWrite(modemUnset, HIGH); // Flip latching relay power to off
        delay(10);
        digitalWrite(modemUnset, LOW); // Latched, so set low to save power
      }
    }
  }



  digitalWrite(h2oSet, HIGH);
  delay(10);
  digitalWrite(h2oSet, LOW);

  delay(1000);

  float level_voltage =  analogRead(H2O_LEVL_PIN) * (H2O_VREF / MAX_ANALOG_VAL);
  float level_current = level_voltage / 120.0; //Sense Resistor:120ohm
  float depth = (level_current - CURRENT_INIT) * (RANGE / DENSITY_WATER / 16.0);

  digitalWrite(h2oUnset, HIGH);
  delay(10);
  digitalWrite(h2oUnset, LOW);


  digitalWrite(TURB_SWITCH, HIGH);

  delay(600);

  //Get an N average NTU reading
  //Compute average voltage output of turbidity probe (need to establish / verify voltage NTU curve)
  float turb_voltage = analogRead(TURB_PIN) * (TURB_VREF / MAX_ANALOG_VAL); // Convert the analog reading (which goes from 0 - 4096) to a voltage (0 - 5V):
  float turb_ntu = Volt_to_NTU(turb_voltage);

  digitalWrite(TURB_SWITCH, LOW);

  String datastring = String(now.year()) + ":" + String(now.month()) + ":" + String(now.day()) + " " + String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second()) + "," + String(depth) + " mm ," + String(turb_ntu) + "mV";

  //Write datastring and close logfile on SD card
  dataFile = SD.open(filename, FILE_WRITE);
  if (dataFile)
  {
    dataFile.println(datastring);
    dataFile.close();
  }

}

void loop() {
  // Blink.
  digitalWrite(led, HIGH);
  delay(1000);
  digitalWrite(led, LOW);
  delay(1000);

  // We're done!
  // It's important that the donePin is written LOW and THEN HIGH. This shift
  // from low to HIGH is how the Nano Power Timer knows to turn off the
  // microcontroller.
  digitalWrite(donePin, LOW);
  digitalWrite(donePin, HIGH);
  delay(10);

}
