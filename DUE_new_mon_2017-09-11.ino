// Display
#include <LiquidCrystal.h>
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2; // Connect display pins to DUE 
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);                  // Tell driver which pins are used

// NMEA2000
#include <Arduino.h>
#define N2k_CAN_INT_PIN 11
#include "NMEA2000_CAN.h"  // This will automatically choose right CAN library and create suitable NMEA2000 object
#include "N2kMessages.h"
// List here messages your device will transmit.
const unsigned long TemperatureMonitorTransmitMessages[] PROGMEM={130310L,130311L,130312L,0};
#define TempUpdatePeriod 2000 // Timer to limit temp messages to the N2K bus to every 2 seconds
#define DispUpdatePeriod 1000 // Timer to limit updates to the display to every 1 second

// OneWire temp sensors DS18B20
//Get DallasTemperature Library here:  http://milesburton.com
#include <OneWire.h>
#include <DallasTemperature.h>
#define TEMPERATURE_PRECISION 11
#define ONE_WIRE_BUS_1 52   // BUS (yellow) to DUE digital pin 52 (a 4.7K resistor is necessary between BUS and +5V)                   
OneWire oneWire_cabin(ONE_WIRE_BUS_1);
DallasTemperature sensor_cabin(&oneWire_cabin);  // Pass on oneWire reference to Dallas Temperature. 
#define ONE_WIRE_BUS_2 50   // BUS (yellow) to DUE digital pin 50 (a 4.7K resistor is necessary between BUS and +5V)
OneWire oneWire_water(ONE_WIRE_BUS_2); 
DallasTemperature sensor_water(&oneWire_water); 
#define DEV_TEMP 0

void setup() {
// 2x16 display
lcd.begin(16, 2);
lcd.clear();
lcd.print("Starting");
delay(800);
lcd.clear();

// Dallas
sensor_water.begin();
int numberOfDevices = 0;
int sensresolution = 0;
numberOfDevices = sensor_water.getDeviceCount();  // Grab a count of devices on the wire
DeviceAddress deviceAddress;
if (sensor_water.getAddress(deviceAddress, 0)) sensor_water.setResolution(deviceAddress, TEMPERATURE_PRECISION);
unsigned char Sens_water_addr = sensor_water.getAddress(deviceAddress,0);
sensresolution = sensor_water.getResolution();
lcd.clear();
lcd.print(numberOfDevices);
lcd.print(" watrsns,res:");
lcd.print(sensresolution);
delay(1500);
lcd.clear();
sensor_cabin.begin();
numberOfDevices = sensor_cabin.getDeviceCount();  // Grab a count of devices on the wire
if (sensor_cabin.getAddress(deviceAddress, 0)) sensor_cabin.setResolution(deviceAddress, TEMPERATURE_PRECISION);
sensresolution = sensor_cabin.getResolution();
lcd.print(numberOfDevices);
lcd.print(" cabnsns,res:");
lcd.print(sensresolution);
delay(1500);
lcd.clear();

// NMEA2000
Serial.begin(115200);
// Here we tell, which PGNs we transmit from temperature monitor
  NMEA2000.ExtendTransmitMessages(TemperatureMonitorTransmitMessages,DEV_TEMP);
  
// Set Product information for temperature monitor
  NMEA2000.SetProductInformation("112233", // Manufacturer's Model serial code
                                 100, // Manufacturer's product code
                                 "DUE Temp Monitor",  // Manufacturer's Model ID. Reboot MFD to detect unit if name changed.
                                 "1.0.0.0 (2017-09-19)",         // Manufacturer's Software version code
                                 "1.0.0.0 (2017-09-19)",         // Manufacturer's Model version
                                 0xff,                           // load equivalency - use default
                                 0xffff,                         // NMEA 2000 version - use default
                                 0xff,                           // Certification level - use default
                                 DEV_TEMP
                                 );
 // Set device information for temperature monitor
  NMEA2000.SetDeviceInformation(112233, // Unique number. Use e.g. Serial number
                                130,    // Device function=Temperature. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                75,     // Device class=Sensor Communication Interface. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2040,   // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf                               
                                4,      // Marine
                                DEV_TEMP
                                ); 
  // Uncomment 2 rows below to see, what device will send to bus. Use e.g. OpenSkipper or Actisense NMEA Reader                           
  // NMEA2000.SetForwardStream(&Serial);
  // If you want to use simple ascii monitor like Arduino Serial Monitor, uncomment next line
  NMEA2000.SetForwardType(tNMEA2000::fwdt_Text); // Show in clear text. Leave uncommented for default Actisense format.

  // If you also want to see all traffic on the bus use N2km_ListenAndNode instead of N2km_NodeOnly below
  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode,22);
  // NMEA2000.SetDebugMode(tNMEA2000::dm_Actisense); // Uncomment this, so you can test code without CAN bus chips on Arduino Mega
  // NMEA2000.EnableForward(false); // Disable all msg forwarding to USB (=Serial)
}

void loop() {
//NMEA2000
SendN2kTemperature();
NMEA2000.ParseMessages();

// Display to 2x16-display
static unsigned long DispUpdated=millis();
if ( DispUpdated+DispUpdatePeriod<millis() ) {
  DispUpdated=millis();
  lcd.setCursor(0, 0);
  lcd.print("Cabin Tmp");
  lcd.setCursor(11, 0);
  lcd.print(sensor_cabin.getTempCByIndex(0));
  //
  lcd.setCursor(0, 1);
  lcd.print("Water Tmp");
  lcd.setCursor(11, 1);
  lcd.print(sensor_water.getTempCByIndex(0));
  }
}

double ReadCabinTemp() {
  return CToKelvin(sensor_cabin.getTempCByIndex(0));        // Read temperature from Onewire
  }
double ReadWaterTemp() {
  return CToKelvin(sensor_water.getTempCByIndex(0));        // Read temperature from Onewire
  }


void SendN2kTemperature() {
  static unsigned long Updated=millis();
  tN2kMsg N2kMsg;
  if ( Updated+TempUpdatePeriod<millis() ) {
    Updated=millis();
    sensor_cabin.requestTemperatures();
    sensor_water.requestTemperatures();
    SetN2kTemperature(N2kMsg, 1, 1, N2kts_MainCabinTemperature, ReadCabinTemp());
    NMEA2000.SendMsg(N2kMsg,DEV_TEMP);
    SetN2kEnvironmentalParameters(N2kMsg, 1, N2kts_MainCabinTemperature, ReadCabinTemp());
    NMEA2000.SendMsg(N2kMsg,DEV_TEMP);
    SetN2kOutsideEnvironmentalParameters(N2kMsg, 1, ReadWaterTemp());
    NMEA2000.SendMsg(N2kMsg,DEV_TEMP);
    Serial.print(millis()); Serial.print(" Sent Cabin: "); Serial.print(sensor_cabin.getTempCByIndex(0)); 
    Serial.print("  Water: "); Serial.println(sensor_water.getTempCByIndex(0));
  }
}


