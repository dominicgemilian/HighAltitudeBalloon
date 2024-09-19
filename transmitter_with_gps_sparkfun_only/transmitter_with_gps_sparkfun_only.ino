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

// TinyGPSPlus gps;



// Flag to set to payload or ground station mode
bool groundStation = 1;

// Variable to store NMEA string from GPS board
String aprsMessageArray;

void setup() {

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

   delay(250);
   
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

      // Create an APRS message
      createAPRSMessage();

      // Send the APRS message over LoRA
      //sendAPRSMessage(aprsMessageArray);

      delay(10);

      Serial.print("Latitude: ");
      Serial.println(myGNSS.getLatitude()/10000000.0, 6);
      Serial.print("Longitude: ");
      Serial.println(myGNSS.getLongitude()/10000000.0, 6);
      Serial.print("Altitude: ");
      Serial.println(myGNSS.getAltitude()*0.00328084, 6);
      Serial.println("=================Run HAB================");
      Serial.println("");

      delay(10);


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


// Function that creates an APRS formatted messaage
void createAPRSMessage(){

  // Create APRS packet
  String aprsMessage = "KC1ADC-11";
  aprsMessage += ">APRS,WIDE2-1:";
  aprsMessage += "/";

  if (myGNSS.getHour() < 10){
    aprsMessage += 0; // Hour
    aprsMessage += myGNSS.getHour(); // Hour
  }
  else{
    aprsMessage += myGNSS.getHour(); // Hour
  }
  
  if (myGNSS.getMinute() < 10){
    aprsMessage += 0; // Minute
    aprsMessage += myGNSS.getMinute(); // Minute
  }
  else{
    aprsMessage += myGNSS.getMinute(); // Minute
  }
  
  aprsMessage += "h";

  aprsMessage += String(myGNSS.getLatitude()/10000000.0,6);
  aprsMessage += "/";
  aprsMessage += String(myGNSS.getLongitude()/10000000.0,6);
  aprsMessage += "O";
  aprsMessage += String(myGNSS.getHeading()/100000.0,3);
  aprsMessage += "/";
  aprsMessage += String((.001*myGNSS.getGroundSpeed())/.514444,3);
  aprsMessage += "/";
  aprsMessage += "A=";
  aprsMessage += String(myGNSS.getAltitude()*0.00328084,6);

  aprsMessageArray = aprsMessage.c_str();
  
  Serial.println();
  Serial.println("======createAPRSMessage()=====");
  Serial.print("APRS Message: ");
  Serial.println(aprsMessageArray);
  Serial.println("======createAPRSMessage()=====");
  Serial.println();

  sendAPRSMessage(aprsMessageArray);
}


// Function that sends APRS formatted message over LoRa
void sendAPRSMessage(String aprsMessageArray) {

  // Create a uint8_t array to hold the ASCII values of the string characters
  int length = aprsMessageArray.length();
  uint8_t aprsMessageRF95[length + 1]; // +1 for the null terminator

  // Convert the string to uint8_t array
  for (int i = 0; i < length; i++) {
    aprsMessageRF95[i] = (uint8_t) aprsMessageArray[i];
  }

  
  rf95.send(aprsMessageRF95, sizeof(aprsMessageRF95));
  rf95.waitPacketSent();
}

String inputData = "";  // Variable to store incoming APRS packet
bool newData = false;

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
    
    inputData = (char*) buf;

    newData = true;
   
  }


  // Parse APRS packet
  if (newData) {
  

    parseAPRS(inputData);

    // Reset input data
    inputData = "";
    newData = false;
  }


}

// Function to parse APRS packet
void parseAPRS(String message) {
 
  int timeStartPos = message.indexOf(":") + 2;
  int timeEndPos = timeStartPos + 4;
  
  String time = message.substring(timeStartPos, timeEndPos);

  // Check for the position of ':' (start of position report) and 'z' (timestamp)
  int latStartPos = message.indexOf("h") + 1;
  int latEndPos = latStartPos + 9;
  
  String latitude = message.substring(latStartPos, latEndPos);

  // Check for the position of ':' (start of position report) and 'z' (timestamp)
  int lonStartPos = latEndPos + 2;
  int lonEndPos = lonStartPos + 9;
  
  String longitude = message.substring(lonStartPos, lonEndPos);

  // Check for the position of ':' (start of position report) and 'z' (timestamp)
  int headingStartPos = lonEndPos + 1;
  int headingEndPos = headingStartPos + 6;
  
  String heading = message.substring(headingStartPos, headingEndPos);

  // Check for the position of ':' (start of position report) and 'z' (timestamp)
  int speedStartPos = headingEndPos + 2;
  int speedEndPos = speedStartPos + 5;
  
  String speed = message.substring(speedStartPos, speedEndPos);

  // Check for the position of ':' (start of position report) and 'z' (timestamp)
  int altStartPos = speedEndPos + 3;
  int altEndPos = altStartPos + 11;
  
  String altitude = message.substring(altStartPos, altEndPos);
  
  // Print parsed data
  Serial.print("time: ");
  Serial.println(time);
  Serial.print("lat: ");
  Serial.println(latitude);
  Serial.print("lon: ");
  Serial.println(longitude);
  Serial.print("alt: ");
  Serial.println(altitude);
  Serial.print("heading: ");
  Serial.println(heading);
  Serial.print("speed: ");
  Serial.println(speed);

  delay(10);
}
