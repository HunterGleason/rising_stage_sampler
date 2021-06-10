/***********************************************************
  DFRobot Gravity: Analog Current to Voltage Converter(For 4~20mA Application)
  SKU:SEN0262

  GNU Lesser General Public License.
  See <http://www.gnu.org/licenses/> for details.
  All above must be included in any redistribution
 ****************************************************/

#include <SPI.h>
#include <SD.h>
#include <RTCZero.h>
#include <ArduinoLowPower.h>

const byte H2O_LEVL_PIN = A1; //Define pin for reading DFRobot liquid level sensor
const byte H2O_LEVL_SWITCH = 10;
const byte RED_LED = 13;
const byte chipSelect = 4;  //** CS - pin 4 (for MKRZero SD: SDCARD_SS_PIN)
const int RANGE = 5000; // Depth measuring range 5000mm (for water)
const double CURRENT_INIT = 4.00; // Current @ 0mm (uint: mA)
const double DENSITY_WATER = 1.00;  // Pure water density
const int VREF = 3300; //Refrence voltage, 3.3V for Adalogger M0


const String filename = "data_log.txt";

/* Change these values to set the current initial time */
const byte hours = 11;
const byte minutes = 12;
const byte seconds = 0;
/* Change these values to set the current initial date */
const byte day = 8;
const byte month = 6;
const byte year = 21;

//Number of miliseconds between sleep / read / log cycles
const int log_interval_ms = 15000;
bool matched = false;

int dataVoltage;
float dataCurrent, depth; //unit:mA

RTCZero rtc;
File dataFile;

//Function reads DFRobot Liquid level sensor n times and returns average depth (mm+-25)
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

void setup()
{

  rtc.begin();    // Start the RTC in 24hr mode
  rtc.setTime(hours, minutes, seconds);   // Set the time
  rtc.setDate(day, month, year);    // Set the date

  rtc.setAlarmTime(hours, minutes + 1, seconds);
  rtc.enableAlarm(rtc.MATCH_MMSS);

  rtc.attachInterrupt(alarmMatch);

  //Initlize H2O_LEVL_PIN as input
  pinMode(H2O_LEVL_PIN, INPUT);

  //Initlize H2O_LEVL_SWITCH as output, switch off power
  pinMode(H2O_LEVL_SWITCH, OUTPUT);
  digitalWrite(H2O_LEVL_SWITCH, LOW);

  //Turn of RED LED to save power
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

  rtc.standbyMode();
}

void loop()
{

  if (matched)
  {
    //reset 'matched' alarm
    matched == false;

    //Power on the liquid level sensor
    digitalWrite(H2O_LEVL_SWITCH, HIGH);

    //Allow time for liquid level sensor to stabalize (not sure what best duration is for this, check docs sheet?)
    delay(1000);

    //Take a average of 5 consecutive readings
    float cur_depth_mm = readLiqdLevel(5);

    //Kill power to liquid level sensor
    digitalWrite(H2O_LEVL_SWITCH, LOW);

    //String to write to SD card
    String datastring = String(rtc.getDay()) + "-" + String(rtc.getMonth()) + "-" + String(rtc.getYear()) + " " + String(rtc.getHours()) + ":" + String(rtc.getMinutes()) + ":" + String(rtc.getSeconds()) + "," + String(cur_depth_mm);

    //Write and close logfile
    dataFile = SD.open(filename, FILE_WRITE);

    if (dataFile)
    {
      dataFile.println(datastring);
      dataFile.close();
    }

    //Set next rtc wakeup alarm 
    int alarmMinutes = rtc.getMinutes();

    alarmMinutes += 1;
    if (alarmMinutes >= 60) {
      alarmMinutes -= 60;
    }

    rtc.setAlarmTime(rtc.getHours(), alarmMinutes, rtc.getSeconds());
    rtc.standbyMode();    // Sleep until next alarm match
  }
}

void alarmMatch() {
  matched = true;
}
