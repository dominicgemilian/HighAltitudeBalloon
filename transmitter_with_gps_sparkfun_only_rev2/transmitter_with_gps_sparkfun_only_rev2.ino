// Feather9x_TX
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messaging client (transmitter)
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example Feather9x_RX

#include <SPI.h>
#include <RH_RF95.h>
#include <Wire.h>
#include <iostream>
#include <string>
#include <TinyGPS++.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //Click here to get the library:  http://librarymanager/All#SparkFun_u-blox_GNSS

#define DATAOUT 11      //MOSI
#define DATAIN 12       //MISO
#define SPICLOCK 6      //sck
#define SPICS 10  //ss

SFE_UBLOX_GNSS myGNSS;

// First 3 here are boards w/radio BUILT-IN. Boards using FeatherWing follow.
#if defined (__AVR_ATmega32U4__)  // Feather 32u4 w/Radio
#define RFM95_CS    8
#define RFM95_INT   7
#define RFM95_RST   4

#elif defined(ADAFRUIT_FEATHER_M0) || defined(ADAFRUIT_FEATHER_M0_EXPRESS) || defined(ARDUINO_SAMD_FEATHER_M0)  // Feather M0 w/Radio
#define RFM95_CS    8
#define RFM95_INT   3
#define RFM95_RST   4

#elif defined(ARDUINO_ADAFRUIT_FEATHER_RP2040_RFM)  // Feather RP2040 w/Radio
#define RFM95_CS   16
#define RFM95_INT  21
#define RFM95_RST  17

#elif defined (__AVR_ATmega328P__)  // Feather 328P w/wing
#define RFM95_CS    4  //
#define RFM95_INT   3  //
#define RFM95_RST   2  // "A"

#elif defined(ESP8266)  // ESP8266 feather w/wing
#define RFM95_CS    2  // "E"
#define RFM95_INT  15  // "B"
#define RFM95_RST  16  // "D"

#elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2) || defined(ARDUINO_NRF52840_FEATHER) || defined(ARDUINO_NRF52840_FEATHER_SENSE)
#define RFM95_CS   10  // "B"
#define RFM95_INT   9  // "A"
#define RFM95_RST  11  // "C"

#elif defined(ESP32)  // ESP32 feather w/wing
#define RFM95_CS   33  // "B"
#define RFM95_INT  27  // "A"
#define RFM95_RST  13

#elif defined(ARDUINO_NRF52832_FEATHER)  // nRF52832 feather w/wing
#define RFM95_CS   11  // "B"
#define RFM95_INT  31  // "C"
#define RFM95_RST   7  // "A"

#endif

/* Some other possible setups include:

  // Feather 32u4:
  #define RFM95_CS   8
  #define RFM95_RST  4
  #define RFM95_INT  7

  // Feather M0:
  #define RFM95_CS   8
  #define RFM95_RST  4
  #define RFM95_INT  3

  // Arduino shield:
  #define RFM95_CS  10
  #define RFM95_RST  9
  #define RFM95_INT  7

  // Feather 32u4 w/wing:
  #define RFM95_RST 11  // "A"
  #define RFM95_CS  10  // "B"
  #define RFM95_INT  2  // "SDA" (only SDA/SCL/RX/TX have IRQ!)

  // Feather m0 w/wing:
  #define RFM95_RST 11  // "A"
  #define RFM95_CS  10  // "B"
  #define RFM95_INT  6  // "D"
*/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

TinyGPSPlus gps;

// Flag to set to payload or ground station mode
bool groundStation = 0;

// Variable to store NMEA string from GPS board
String aprsMessageArray;

void setup() {

  SPI.begin();
  
  // Set SPI pin modes
  pinMode(SPICLOCK, OUTPUT);
  pinMode(DATAOUT, OUTPUT);
  pinMode(DATAIN, INPUT);
  pinMode(SPICS,OUTPUT);

  digitalWrite(SPICS, HIGH); //disable device


  // Set the reset pin
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  // Both Serial and Serial1 used
  Serial.begin(38400);

  Serial1.begin(38400);
  Serial1.println("SparkFun u-blox Example");

  Wire.begin();

  if ((myGNSS.begin() == false) && (groundStation == false))
  {
    Serial1.println(F("u-blox GNSS module not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  //This will pipe all NMEA sentences to the serial port so we can see them
  myGNSS.setNMEAOutputPort(Serial1);

  // while (!Serial) delay(1);
  delay(100);

  Serial.println("Feather LoRa TX Test!");

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
}

int16_t packetnum = 0;  // packet counter, we increment per xmission

void loop() {

  // Print the amount of free RAM at startup
  //Serial.print("Free RAM: ");
  //Serial.println(freeRam());

  // Run the HAB payload code if you're not a ground station
  if (groundStation == 0){

    // Runs function that gets NMEA messages from GPS board, sends NMEA messages to Trackuino
    // and then sends APRS packets to ground station
    runHAB();

    delay(10);
  
  }

  else{

    // Runs the ground station receiver code
    loraReceive();
    delay(10);
  }
 

}

// Calls function to get NMEA messages from the I2C board
// Calls function to send NMEA mmessage to Trackuino
// Calls function to send APRS message
void runHAB() {

   delay(5000);
   
   // Get NMEA messages from the I2C board to obtain position
   // getPosition();

   myGNSS.checkUblox(); //See if new data is available. Process bytes as they come in.

   // If the NMEA messages resulted in a valid position print the debug statements below
   if (myGNSS.getPVT()) {

      Serial.println("");
      Serial.println("=================Run HAB================");

      // Print time in UTC (Coordinated Universal Time)
      Serial.print("UTC Time: ");
      Serial.print(myGNSS.getHour());   // Hour (0-23)
      Serial.print(":");
      Serial.print(myGNSS.getMinute()); // Minute (0-59)
      Serial.print(":");
      Serial.print(myGNSS.getSecond()); // Second (0-59)
      Serial.print(".");
      Serial.println(myGNSS.getMillisecond()/10); // Centisecond (0-99)

      delay(10);

      Serial.print("Latitude: ");
      Serial.println(myGNSS.getLatitude()/10000000.0, 6);
      double latitude = myGNSS.getLatitude()/10000000.0;
      String lat_str = String(latitude, 6);
      String lat = "Latitude: " + lat_str;
      const uint8_t* lat_data = (const uint8_t*)lat.c_str();
      rf95.send(lat_data, lat.length());
      rf95.waitPacketSent();
      Serial.print("Longitude: ");
      Serial.println(myGNSS.getLongitude()/10000000.0, 6);
      double longitude = myGNSS.getLongitude()/10000000.0;
      String lon_str = String(longitude, 6);
      String lon = "Longitude: " + lon_str;
      const uint8_t* lon_data = (const uint8_t*)lon.c_str();
      rf95.waitPacketSent();
      Serial.print("Altitude (meters): ");
      Serial.println(myGNSS.getAltitude()*0.00328084, 6);
      //int32_t altitude = myGNSS.getAltitude(1100);
      double altitude = myGNSS.getAltitude()*0.00328084;
      String alt_str = String(altitude, 6);
      String alt = "Altitude (meters): " + alt_str;
      const uint8_t* alt_data = (const uint8_t*)alt.c_str();
      rf95.send(alt_data, alt.length());
      rf95.waitPacketSent();
      Serial.println("=================Run HAB================");
      Serial.println("");

      delay(10);

       // Initialize SPI bus settings
      SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE3));
         
   }


}


// Function that requests NMEA message packets from the GPS board
void getPosition(){

  String nmeaString; 

  Wire.beginTransmission(0x42);
  Wire.write(0x00); // Send a command byte to request position data (this is a placeholder, actual command depends on ZOE-M8Q module configuration)
  Wire.endTransmission();
  Wire.requestFrom(0x42, 32); // Request position data (adjust the number of bytes as needed)

  int count = 0;

  // Used later on for nmea parsing
  int nmeaIndex = 0;

  while (Wire.available()) {

    char zoeM8QRead = Wire.read();

    //myGNSS.encode(zoeM8QRead);
    nmeaString += zoeM8QRead; 


    if (zoeM8QRead == '\n' || zoeM8QRead == '\r') {
      // End of NMEA sentence, process it
      nmeaString[nmeaIndex] = '\0';  // Null-terminate the string

      if (nmeaIndex > 0){


        Serial.println("");
        Serial.println("======getPos()=====");
        Serial.print("Raw NMEA: ");
        Serial.println(nmeaString);
        Serial.println("======getPos()=====");
        Serial.println("");
  
  
        
        // If the NMEA messages were of the type GNRMC or GNGGA then we're ready to go
        //if ((nmeaString.substring(0, 6) == "$GNRMC" || nmeaString.substring(0, 6) == "$GNGGA") && nmeaString.length() >= 31) {
  
        delay(10);
    
        Serial.println("");
        Serial.println("=====================sending NMEA to Trackuino=====================");
        Serial.println(nmeaString);
        Serial.println("=====================sending NMEA to Trackuino=====================");
        Serial.println("");
        //Serial1.println(nmeaString);
        
      }

  
      delay(10);
    
      //}

      // Reset buffer index for next sentence
      nmeaIndex = 0;
      nmeaString = "";
      
    } else {
      
      // Add character to buffer
      if (nmeaIndex < sizeof(nmeaString) - 1) {
        nmeaString[nmeaIndex++] = zoeM8QRead;

      }
    }
    
  }

}


void loraReceive(){

  // Read data from serial
  while (rf95.available() > 0) {

    // Should be a message for us now
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len)) {
      digitalWrite(LED_BUILTIN, HIGH);
      RH_RF95::printBuffer("Received: ", buf, len);
      //Serial.print("Got: ");
      Serial.println((char*) buf);

      //Serial.print("RSSI: ");
      //Serial.println(rf95.lastRssi(), DEC);
    }
       
  }

}
