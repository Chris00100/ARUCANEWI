#include <Arduino.h>
// Arduino DUE NMEA2000 boat monitoring system. 4 x temp (OneWire DS18B20), 1 x humidity (DHT11), 1 x air pressure (BMP180)
// 1 x 2x16 LCD display, NMEA2000 transciever (MCP2562). The program reads sensors and writes values to LCD display and NMEA2000 bus
// Based on the libraries by collin80 and ttlappalainen (and others). C Hedlund January 2018
//
//Define global variables to be used in read, send, print and display operations
  double OWireOutsideTmp = 0;          // DS18B20 DUE pin 52
  double OWireHeatingTmp = 0;          // DS18B20 DUE pin 50
  double OWireEngineTmp = 0;           // DS18B20 DUE pin 48
  double OWireFridgeTmp = 0;           // DS18B20 DUE pin 46
  double DHT11MainCabinTmp = 0;        // Mounted in cabin
  double DHT11OutsideHumidity = 0;     // Sensor mounted inside cabin but the only humidity Garmin can receive on NMEA2K is outside
  double BMP180AirPressure = 0;        // Mounted in DUE box mounted in engine room
  double BMP180EngineRoomTmp = 0;      // Mounted in DUE box mounted in engine room
  double AnalogVoltageGenerator = 0;
  int DispBlock = 0;                   // Flips LCD display
  #define TempUpdatePeriod 2000        // Timer in millisconds to limit temp messages to the N2K bus to every 2 seconds
  #define DispUpdatePeriod 3000        // Timer in milliseconds to limit updates to the display to every 3 seconds
  #define TempHumidityReadPeriod 2000  // Timer to read sensor
  #define RPMWritePeriod 500
  
// BMP180 Pressure sensor, lib from https://github.com/adafruit/Adafruit-BMP085-Library
  #include <Wire.h>
  #include <Adafruit_BMP085.h>
  Adafruit_BMP085 bmp;

// 2 line 16 char LCD Display
  #include <LiquidCrystal.h>
  const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2; // Connect display pins to DUE 
  LiquidCrystal lcd(rs, en, d4, d5, d6, d7);                  // Tell driver which pins are used

// Display I2C from https://github.com/marcoschwartz/LiquidCrystal_I2C To Be Develped

// RPM
  volatile byte REV;              //  VOLATILE DATA TYPE TO STORE REVOLUTIONS
  unsigned long int rpm, maxRPM;  //  DEFINE RPM AND MAXIMUM RPM
  unsigned long timeOneRev;       //  DEFINE TIME TAKEN TO COVER ONE REVOLUTION
  int led = 0, RPMlen , prevRPM;  //  INTEGERS TO STORE LED VALUE AND CURRENT RPM AND PREVIOUS RPM 
  long prevtime = 0;              //  STORE IDLE TIME TO TOGGLE MENU

// NMEA2000
// NMEA2K libs from https://github.com/ttlappalainen
// due_can lib from https://github.com/collin80/due_can
// #define N2k_CAN_INT_PIN 11
  #include "NMEA2000_CAN.h"  // This will automatically choose right CAN library and create suitable NMEA2000 object
  #include "N2kMessages.h"
// List here messages your device will transmit.
//PGN127488 Engine params Rapid Udate for Engine RPM
//PGN127489 Engine params Dynamic for Engine Temp
  const unsigned long TemperatureMonitorTransmitMessages[] PROGMEM={130310L,130311L,130312L,130316,127488,127489};

// OneWire temp sensors DS18B20. DallasTemperature Library:  http://milesburton.com
  #include <OneWire.h>
  #include <DallasTemperature.h>
  #define TEMPERATURE_PRECISION 11
  #define ONE_WIRE_BUS_1 52            // BUS (yellow) to DUE digital pin 52 (4.7K resistor required between BUS and +5V)    
  OneWire oneWire_outside(ONE_WIRE_BUS_1);
  DallasTemperature sensor_outside(&oneWire_outside);
  #define ONE_WIRE_BUS_2 50
  OneWire oneWire_heating(ONE_WIRE_BUS_2);
  DallasTemperature sensor_heating(&oneWire_heating);
  #define ONE_WIRE_BUS_3 48
  OneWire oneWire_engine(ONE_WIRE_BUS_3);
  DallasTemperature sensor_engine(&oneWire_engine);
  #define ONE_WIRE_BUS_4 46
  OneWire oneWire_fridge(ONE_WIRE_BUS_4);
  DallasTemperature sensor_fridge(&oneWire_fridge);

// DHT11/21/22 Temp & Humidity sensor on digital pin 8
// Connect pin 1 (on the left) of the sensor to +5V or 3,3 for DUE, pin 2 to any digital and pin4 to gnd
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor
  #include "DHT.h"        // Written by Adafruit & ladyada at https://github.com/adafruit/Adafruit_Sensor
  #define DHTPIN 8        // what digital pin we're connected to
  #define DHTTYPE DHT11   // DHT 21 (AM2301) and 22 (AM2302), AM2321 also possible
  DHT dht(DHTPIN, DHTTYPE);

  #define DEV_TEMP 0   // One (and only) N2K bus device in this sketch at this time

  void setup() {
    Serial.begin(115200);
    Serial.println("Startup time 8 sec pls wait");

//BMP180 init
    Serial.println("BMP180 air pressure sensor init");
    if (!bmp.begin()) {
      Serial.println("Could not find a valid BMP085 sensor, check wiring!");
      //while (1) {}
      }

// DHT11 Init
    Serial.println("DHT11 humidity sensor init");
    dht.begin();
// 2x16 display
    Serial.println("LCD display init");
    lcd.begin(16, 2);
    lcd.clear();
    lcd.print("Boot ready");
    delay(600);
    lcd.clear();

// Dallas Init
    Serial.println("Dallas tmp sensors init");
    lcd.print("Init temp sensors");
    int numberOfDevices = 0;
    int sensresolution = 0;
    DeviceAddress deviceAddress;
    delay(800);
    sensor_heating.begin();
    numberOfDevices = sensor_heating.getDeviceCount();  // Grab a count of devices on the wire
    if (sensor_heating.getAddress(deviceAddress, 0)) sensor_heating.setResolution(deviceAddress, TEMPERATURE_PRECISION);
    sensresolution = sensor_heating.getResolution();
    lcd.clear(); lcd.print(numberOfDevices); lcd.print(" Heating, res:"); lcd.print(sensresolution);
    delay(800); lcd.clear();

    sensor_outside.begin();
    numberOfDevices = sensor_outside.getDeviceCount();  // Grab a count of devices on the wire
    if (sensor_outside.getAddress(deviceAddress, 0)) sensor_outside.setResolution(deviceAddress, TEMPERATURE_PRECISION);
    sensresolution = sensor_outside.getResolution();
    lcd.print(numberOfDevices);lcd.print(" Outside, res:");lcd.print(sensresolution);
    delay(800); lcd.clear();

    sensor_engine.begin();
    numberOfDevices = sensor_engine.getDeviceCount();  // Grab a count of devices on the wire
    if (sensor_engine.getAddress(deviceAddress, 0)) sensor_engine.setResolution(deviceAddress, TEMPERATURE_PRECISION);
    sensresolution = sensor_engine.getResolution();
    lcd.print(numberOfDevices);lcd.print(" Engine, res:");lcd.print(sensresolution);
    delay(800); lcd.clear();

    sensor_fridge.begin();
    numberOfDevices = sensor_fridge.getDeviceCount();  // Grab a count of devices on the wire
    if (sensor_fridge.getAddress(deviceAddress, 0)) sensor_fridge.setResolution(deviceAddress, TEMPERATURE_PRECISION);
    sensresolution = sensor_fridge.getResolution();
    lcd.print(numberOfDevices);lcd.print(" Fridge, res:");lcd.print(sensresolution);
    delay(800); lcd.clear(); lcd.print("Ready, starting"); delay(700); lcd.clear();

// RPM
    Serial.println("RPM Init"); 
    pinMode(53, INPUT);   
    attachInterrupt(digitalPinToInterrupt(53), RPMCount, RISING);     // ADD A HIGH PRIORITY ACTION ( AN INTERRUPT)                                        
    REV = 0;                                  // WHEN THE SENSOR GOES FROM LOW TO HIGH START ALL THE VARIABLES FROM 0
    rpm = 0;

// NMEA2000
    Serial.println("NMEA2000 init");
// Here we tell, which PGNs we transmit
    NMEA2000.ExtendTransmitMessages(TemperatureMonitorTransmitMessages,DEV_TEMP);
  
// Set Product information for temperature monitor
    NMEA2000.SetProductInformation("112233",                  // Manufacturer's Model serial code
               100,                       // Manufacturer's product code
               "DUE Temp Monitor",        // Manufacturer's Model ID. Reboot MFD to detect unit if name changed.
               "1.0.0.0 (2017-09-19)",    // Manufacturer's Software version code
               "1.0.0.0 (2017-09-19)",    // Manufacturer's Model version
               0xff,                      // load equivalency - use default
               0xffff,                    // NMEA 2000 version - use default
               0xff,                      // Certification level - use default
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
    NMEA2000.SetForwardType(tNMEA2000::fwdt_Text);     // Show in clear text. Leave uncommented for default Actisense format.
  // If you also want to see all traffic on the bus use N2km_ListenAndNode instead of N2km_NodeOnly below
    NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode,22);
  // NMEA2000.SetDebugMode(tNMEA2000::dm_Actisense); // Uncomment this, so you can test code without CAN bus chips on Arduino Mega
  // NMEA2000.EnableForward(false);                  // Disable all msg forwarding to USB (=Serial)
    }

void loop() {
  SendN2kTemperature();       //Send Onewire temps (engine, outside, fridge, heating) temp and Alternator Voltage to N2K network and print to Serial
  DisplayOn2x16();            //Print to display
  SendN2kTemperature();       //Send Onewire temps (engine, outside, fridge, heating) temp and Alternator Voltage to N2K network and print to Serial
  SendDHT11BMP180();          //Send info from DHT11 Temp & Humidity AND Outside temp from Onewire sensor AND Air Pressure to N2K
  SendN2kTemperature();       //
  SendRPM();
  NMEA2000.ParseMessages();   //Read N2K/CANbus buffer, not used for anything at this time
  }

void SendN2kTemperature() { 
  static unsigned long Updated=millis();
  if ( Updated+TempUpdatePeriod<millis() ) {
    Updated=millis();
    // Send Engine Temp and Alternator Voltage using PGN 127489, PGN details at end of this file
    tN2kMsg N2kMsg;
    sensor_engine.requestTemperatures();
    OWireEngineTmp = CToKelvin(sensor_engine.getTempCByIndex(0));
    SetN2kEngineDynamicParam(N2kMsg,0,0,0,OWireEngineTmp,analogRead(A0),0,0,0,0,0,0,0); // PGN Field 1=engine#, 2=OilPress, 3=OilTmp, 4=EngTmp, 5=AltVolt
    NMEA2000.SendMsg(N2kMsg); 
    Serial.print("  Engine: "); Serial.print(KelvinToC(OWireEngineTmp));
    //  
    // Send other Temps using PGN 130316 default rate every 2 seconds
    //00 = SeaTemperature, 01 = Outside Temperature, 02 = Inside Temperature, 03 = Engine Room Temperature, 04 = Main Cabin Temperature , 05 = Live Well Temperature
    //06 = Bait Well Temperature, 07 = Refridgeration Temperature , 08 = Heating System Temperature , 09 = Dew Point  Temperature, 10 = Wind Chill Temperature, Apparent
    //11 = Wind Chill Temperature, Theoretical, 12 = Heat Index Temperature, 13 = Freezer Temperature, 14 = Exhaust Gas Temperature                                            
    sensor_fridge.requestTemperatures();
    OWireFridgeTmp = CToKelvin(sensor_fridge.getTempCByIndex(0));
    SetN2kTemperatureExt(N2kMsg, 1, 1, N2kts_RefridgerationTemperature, OWireFridgeTmp);             
    NMEA2000.SendMsg(N2kMsg,DEV_TEMP);
    Serial.print("  Fridge: "); Serial.print(KelvinToC(OWireFridgeTmp));
    //
    sensor_heating.requestTemperatures();
    OWireHeatingTmp = CToKelvin(sensor_heating.getTempCByIndex(0));                                   
    SetN2kTemperatureExt(N2kMsg, 1, 1, N2kts_HeatingSystemTemperature, OWireHeatingTmp);             
    NMEA2000.SendMsg(N2kMsg,DEV_TEMP);
    Serial.print("  Heating: "); Serial.print(KelvinToC(OWireHeatingTmp));
    //
    sensor_outside.requestTemperatures();
    OWireOutsideTmp = CToKelvin(sensor_outside.getTempCByIndex(0));
    SetN2kTemperatureExt(N2kMsg, 1, 1, N2kts_OutsideTemperature, OWireOutsideTmp);             
    NMEA2000.SendMsg(N2kMsg,DEV_TEMP); 
    Serial.print("  Outside: "); Serial.println(KelvinToC(OWireOutsideTmp)); 
    //
    BMP180EngineRoomTmp = bmp.readTemperature();
    SetN2kTemperatureExt(N2kMsg, 1, 1, N2kts_EngineRoomTemperature, CToKelvin(BMP180EngineRoomTmp));             
    NMEA2000.SendMsg(N2kMsg,DEV_TEMP); 
    Serial.print("  Engine Room: "); Serial.println(BMP180EngineRoomTmp); 
    }
  }

void SendDHT11BMP180() {
  static unsigned long Updated=millis();
  if ( Updated+TempHumidityReadPeriod<millis() ) {
    Updated=millis();
    // float hic = dht.computeHeatIndex(DHT11MainCabinTmp, DHT11OutsideHumidity, false);  // Compute heat index in Celsius (isFahreheit = false)
    // Serial.print(hic); // Heat Index
    // Outside Tmp, Humidity and Pressure using N2kPGN130311(N2kMsg,SID,TempSource,Temperature,HumiditySource,Humidity,AtmosphericPressure)
    tN2kMsg N2kMsg;
    DHT11OutsideHumidity = dht.readHumidity();
    BMP180AirPressure = bmp.readPressure();
    SetN2kEnvironmentalParameters(N2kMsg, 1, N2kts_OutsideTemperature, OWireOutsideTmp, N2khs_OutsideHumidity, DHT11OutsideHumidity, BMP180AirPressure);
    NMEA2000.SendMsg(N2kMsg); 
    // Send CabinTemp using PGN 130316, PGN details at end of this file
    DHT11MainCabinTmp = dht.readTemperature();
    SetN2kTemperatureExt(N2kMsg, 1, 1, N2kts_MainCabinTemperature, CToKelvin(DHT11MainCabinTmp));             
    NMEA2000.SendMsg(N2kMsg); 
    Serial.print("DHT11 Humidity: "); Serial.print(DHT11OutsideHumidity); Serial.print(" %\t");
    Serial.print("  DHT11 CabinTemperature: "); Serial.print(DHT11MainCabinTmp);Serial.print(" *C ");
    Serial.print("   AirPressure: "); Serial.print(PascalTomBar(BMP180AirPressure)); Serial.println(" mBar");
  }
}

void SendRPM() {
  if(REV >= 100 ) {                                  // Update after every 100 readings 
    rpm = 30*1000/(millis() - timeOneRev)*REV;       //  Calculate RPM
    timeOneRev = millis();                            
    REV = 0;
    rpm = round(rpm/100);
    rpm = rpm * 100;
    static unsigned long Updated=millis();
    if ( Updated+RPMWritePeriod<millis() ) {
      Updated=millis();
      tN2kMsg N2kMsg;
      SetN2kEngineParamRapid(N2kMsg,0,rpm);          // PGN127488
      NMEA2000.SendMsg(N2kMsg);     
      Serial.print("RPM: "); Serial.println(rpm, DEC);
      }   
   }
}

void RPMCount()        // CNY70 IR Sensor Low to High Interrupt routine 
 {
   REV++;              // INCREASE REVOLUTIONS
 }

void DisplayOn2x16() {
DispBlock = DispBlock + 1;  
static unsigned long DispUpdated=millis();
if ( DispUpdated+DispUpdatePeriod<millis() ) {
  DispUpdated=millis();
  // OWireOutsideTmp, OWireHeatingTmp, OWireEngineTmp, OWireFridgeTmp, DHT11MainCabinTmp, DHT11OutsideHumidity, BMP180AirPressure, AnalogVoltageGenerator
  // Serial.print("DispBlock: "); Serial.println(DispBlock); 
  if ( DispBlock < 10000 ){
    lcd.clear(); lcd.setCursor(0, 0); lcd.print("Heater Tmp");
    lcd.setCursor(11, 0); lcd.print(KelvinToC(OWireHeatingTmp));
    lcd.setCursor(0, 1); lcd.print("Engine Tmp");
    lcd.setCursor(11, 1); lcd.print(KelvinToC(OWireEngineTmp));
    } 
  if ( DispBlock > 10000 and DispBlock < 20000 ){
    lcd.clear(); lcd.setCursor(0, 0); lcd.print("AirPress");
    lcd.setCursor(12, 0); lcd.print(PascalTomBar(BMP180AirPressure));
    lcd.setCursor(0, 1); lcd.print("Humidity");
    lcd.setCursor(11, 1); lcd.print(DHT11OutsideHumidity);
    }
  if ( DispBlock > 20000 and DispBlock < 30000 ){
    lcd.clear(); lcd.setCursor(0, 0); lcd.print("Cabin Tmp");
    lcd.setCursor(11, 0); lcd.print(DHT11MainCabinTmp);
    lcd.setCursor(0, 1); lcd.print("Fridge Tmp");
    lcd.setCursor(11, 1); lcd.print(KelvinToC(OWireFridgeTmp));
    DispBlock = DispBlock + 1;
    }
  }
  if ( DispBlock > 30000 ){
    DispBlock = 0; 
    } 
}
//PGN 127488 provides data with a high update rate--ten times per second--about a limited number of engine parameters which are liable to rapidly change.
//Field 1: Engine Instance (8-bit unsigned integer) This field indicates the particular engine for which this data applies. A single engine will have an instance of 0. Engines in multi-engine boats will be numbered starting at 0 at the bow of the boat incrementing to n going towards the stern of the boat. For engines at the same distance from the bow and stern, the engines are numbered starting from the port side and proceeding towards the starboard side.
//Field 2: Engine Speed (16-bit unsigned integer) This field indicates the rotational speed of the engine in units of 1/4 RPM.
//Field 3: Engine Boost Pressure (16-bit unsigned integer) This field indicates the turbocharger boost pressure in units of 100 Pa.
//Field 4: Engine tilt/trim (8-bit signed integer) This field indicates the tilt or trim (positive or negative) of the engine in units of 1 percent.
//Field 5: Reserved (16 bits) This field is reserved by NMEA; typically all 1's are sent (65535).
//
//PGN 127489 carries data about a wide range of engine parameters with an update rate of once per second.
//Field 1: Engine Instance (8-bit unsigned integer) This field indicates the particular engine for which this data applies. A single engine will have an instance of 0. Engines in multi-engine boats will be numbered starting at 0 at the bow of the boat incrementing to n going towards the stern of the boat. For engines at the same distance from the bow and stern, the engines are numbered starting from the port side and proceeding towards the starboard side.
//Field 2: Engine Oil Pressure (16-bit unsigned integer) This field indicates the oil pressure of the engine in units of 100 Pa.
//Field 3: Engine Oil Temperature (16-bit unsigned integer) This field indicates the oil temperature of the engine in units of 0.1°K.
//Field 4: Engine Temperature (16-bit unsigned integer) This field indicates the temperature of the engine coolant in units of 0.1°K.
//Field 5: Alternator Potential (16-bit signed integer) This field indicates the alternator voltage in units of 0.01V.
//Field 6: Fuel Rate (16-bit signed integer) This field indicates the fuel consumption rate of the engine in units of 0.0001 cubic-meters/hour.
//Field 7: Total Engine Hours (32-bit unsigned integer) This field indicates the cumulative runtime of the engine in units of 1 second.
//Field 8: Engine Coolant Pressure (16-bit unsigned integer) This field indicates the pressure of the engine coolant in units of 100 Pa.
//Field 9: Fuel Pressure (16-bit unsigned integer) This field indicates the pressure of the engine fuel in units of 1000 Pa.
//Field 10: Reserved (8 bits) This field is reserved by NMEA; typically all bits sent as a logic 1.
//Field 11: Engine Discrete Status 1 (16 bits) This field indicates warning conditions of the engine with the following bit assignments (value of 1 indicates warning present):
  //Bit 0: Check Engine
  //Bit 1: Over Temperature
  //Bit 2: Low Oil Pressure
  //Bit 3: Low Oil Level
  //Bit 4: Low Fuel Pressure
  //Bit 5: Low System Voltage
  //Bit 6: Low Coolant Level
  //Bit 7: Water Flow
  //Bit 8: Water in Fuel
  //Bit 9: Charge Indicator
  //Bit 10: Preheat Indicator
  //Bit 11: High Boost Pressure
  //Bit 12: Rev Limit Exceeded
  //Bit 13: EGR System
  //Bit 14: Throttle Position Sensor
  //Bit 15: Emergency Stop Mode
//Field 12: Engine Discrete Status 2 - (16 bits) This field indicates warning conditions of the engine with the following bit assignments (value of 1 indicates warning present):
  //Bit 0: Warning Level 1
  //Bit 1: Warning Level 2
  //Bit 2: Power Reduction
  //Bit 3: Maintenance Needed
  //Bit 4: Engine Comm Error
  //Bit 5: Sub or Secondary Throttle
  //Bit 6: Neutral Start Protect
  //Bit 7: Engine Shutting Down
  //Bit 8-15: These bits are reserved and should be masked when read
//Field 13: Percent Engine Load (8-bit signed integer) This field indicates the percent load of the engine in units of 1 percent.
//Field 14: Percent Engine Torque (8-bit signed integer) This field indicates the percent torque of the engine in units of 1 percent.
//
//PGN 127508L
//BatteryVoltage,0.01);
//BatteryCurrent,0.1);
//BatteryTemperature,0.01);
//
//PGN 130311
//tN2kHumiditySource HumiditySource, double Humidity, double AtmosphericPressure
//
//PGN 130316 default rate 0.5 times/second
//tN2kMsg &N2kMsg, unsigned char SID, unsigned char TempInstance, tN2kTempSource TempSource,double ActualTemperature, double SetTemperature)
//00 = SeaTemperature, 01 = Outside Temperature, 02 = Inside Temperature, 03 = Engine Room Temperature, 04 = Main Cabin Temperature , 05 = Live Well Temperature
//06 = Bait Well Temperature, 07 = Refridgeration Temperature , 08 = Heating System Temperature , 09 = Dew Point  Temperature, 10 = Wind Chill Temperature, Apparent
//11 = Wind Chill Temperature, Theoretical, 12 = Heat Index Temperature, 13 = Freezer Temperature, 14 = Exhaust Gas Temperature   
