# Stage, turbidity & H2O Temperature logger 
Script for measuring stage using the [DFRobot Throw in Pressure transducer](https://www.dfrobot.com/product-1863.html), turbidity using the [DFRobot Turbidity Sensor](https://www.dfrobot.com/product-1394.html) and the [Adafruit Waterproof DS18B20](https://www.adafruit.com/product/381) for measuring water temperature. Script was developed on a Feather [Adalogger M0](https://learn.adafruit.com/adafruit-feather-m0-adalogger/) but may work with other configurations. In addition to data being written to a SD card, daily observations are sent each day at midnight to the internet using a Sparkfun Iridium satellite modem [Sparkfun 9603N](https://www.sparkfun.com/products/16394) (The Iridium modem does require a monthly rental service to exchange information with the Iridium satellite network).

# Installation
Assuming that the Arduino IDE is already installed, and configured for use with the [Adalogger M0](https://learn.adafruit.com/adafruit-feather-m0-adalogger/), this script can be installed using the following commands:

``` bash
cd ~/Arduino
git clone https://github.com/HunterGleason/rising_stage_sampler.git
git checkout with_temp
```
This script relies on the following libraries, which can be installed using the Library Manger in the Arduino IDE:

- <RTClib.h> //Needed for communication with Real Time Clock
- <SPI.h>//Needed for working with SD card
- <SD.h>//Needed for working with SD card
- <IridiumSBD.h>//Needed for communication with the 9603N Iridium modem
- <Wire.h>//Needed for I2C communication
- <CSV_Parser.h>//Needed for parsing CSV data
- <OneWire.h>// Needed for 1-Wire communication
- <DallasTemperature.h>// Needed for communication with DS18B20 temperature sensor

Once all the required libraries are present, I recommend setting the PCF8523 RTC, described in Adafruit [PCF8523](https://learn.adafruit.com/adafruit-pcf8523-real-time-clock/). After the RTC has been set, obtain a micro-SD card, and using a text editor, save the 'params.csv' parameter file described below with desired parameter values. Check that the switches on the [Sparkfun TPL5110](https://www.sparkfun.com/products/15353) match the desired logging interval. Using the IDE upload the rising_stage_sampler.ino to the MCU, unplug the USB, and plug in the USB adapter wired to the TPL5110 [schematic](https://github.com/HunterGleason/MB7369_SnoDpth/blob/wth_iridium/MB7369_SnoDpth.svg) into the MCU. If a battery is connected, logging should begin at ~ the specified interval, be sure to disconnect power before removing or installing the micro-SD card. Data will be output under the specified file name in a CSV.

# Operation 
See [schematic](https://github.com/HunterGleason/MB7369_SnoDpth/blob/wth_iridium/MB7369_SnoDpth.svg) for wiring schematic, and **Installation** section. Time is kept using the PCF8523 real time clock, be sure to set the RTC to the desired time before use [Adafruit PCF8523](https://learn.adafruit.com/adafruit-pcf8523-real-time-clock/). After the RTC has been set, obtain a micro-SD card, and using a text editor, save the 'params.csv' parameter file described below with desired parameters. Power management is done with the TPL5110, **check that the switches on the [Sparkfun TPL5110](https://www.sparkfun.com/products/15353) match the desired logging interval**, which should be no less than 5-minutes if using the Iridium modem. Using the IDE upload the MB7369_SnoDpth.ino to the MCU, unplug the USB, and plug in the USB adapter wired to the TPL5110 [schematic](https://github.com/HunterGleason/rising_stage_sampler/blob/with_temp/siphon_sampler.svg) into the MCU. If a battery is connected, logging should begin at ~ the specified interval, be sure to disconnect power from the MCU before removing or installing the micro-SD card. Data will be output under the specified file name in a CSV format. To convert the voltage measured by the DFRobot turbidity sensor to nephelometric turbidity units (NTU), calibration using standard solutions must be performed before or while deploying the logger. The calibration procedure is outlined below:

## Calibration

1. Leave the analog output from the potentiometer disconnected from the MCU [schematic](https://github.com/HunterGleason/rising_stage_sampler/blob/with_temp/siphon_sampler.svg), with a > 5V power supply connected measure the voltage on the output of the potentiometer, with the turbidity sensor in a solution of 0 NTU adjust the potentiometer so that the voltage is no more than 3.3 V, and ideally around 3.0 V. Once set, this should not have to be repeated. The output from the potentiometer can now be connected to the MCU [schematic](https://github.com/HunterGleason/rising_stage_sampler/blob/with_temp/siphon_sampler.svg).
2. Set the 'slope' and 'intercept' parameters to 1.0 and 0.0, respectively (see Parameter File section), this will return the measured turbidity sensor voltage in millivolts (0-3300). 
3. In the parameter file name the output file something recognizable, e.g., 'calibration.csv'.
4. Recommend setting 'satcom' parameter to '0'. 
5. Set other parameters to deployment values.
6. Recommend setting TPL5110 interval to 30 seconds.
7. Place turbidity probe in NTU standard solution, starting with NTU == 0. 
8. Assuming RTC has been set, launch logger and record time at launch. 
9. Allow for as many points as desired to be collected, recommend at least 10. 
10. Switch turbidity probe to higher NTU solution and record time, again collect as many points as necessary. 
11. Repeat step 10 for each NTU solution standard.   
12. Once all NTU solutions have been sampled, remove the SD card and open the CSV.
13. For values below 400 NTU the relationship is linear, determine slope and intercept parameters from the calibration data.
14. Be sure to update the 'slope' and 'intercept' values in the parameter file before deployment.  


# Parameter File
An example of the 'params.csv' is shown below, this file must be saved to the micro-SD before using this script:

filename,N,slope,intercept,h2o_temp,satcom<br/>
logfile.csv,3,1.0,0.0,1,0

There are six columns that must be present, delimited by commas. The first column named *filename* is the desired name for the data log file, **and must be less than or equal to 8 characters, containing only letters and numbers!** The second column *N* is the number of samples to average when taking sensor readings. The columns *slope* and *intercept* are the beta coefficient and intercept term for converting millivolts to NTU from the calibration (See Calibration section). The *h2o_temp* column in binary either 0:false to not measure temperature, or 1:true measure water temperature, similarly, *satcom* is a binary column indicating weather or not to send daily observations over the Irdium satellite modem.   

# Error Codes

This code utilizes the MCUs built in LED (assumed to be Pin 13) to communicate error codes. These include codes for issues associated with one of either the Iridium modem, RTC or SD card, or missing 'snolog.csv' parameter file. The built in LED will blink at time intervals corresponding to the error, these times are tabulated below:

- LED off -> Normal operation (the LED on the TPL5110 and Iridium modem may be on)
- 5-sec interval -> Iridium modem is not connected
- 10-sec interval -> The RTC failed to initialize
- 1-sec interval -> The SD card was unable to initialize, or is not present 
- 1/4-sec interval -> The snowlog.csv parameter file is not present or cannot be opened
- 2-sec interval -> The SHT30 sensor was unable to initialize
