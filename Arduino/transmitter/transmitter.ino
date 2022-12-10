// Code to collect and transmit data from vehicle to receiver
//
// Radio: Digi XBee Pro S1
// GPS: Adafruit GPS modules using MTK3329/MTK3339 driver
// Accelerometer: 
//
// This code is used on the vehicle to collect data from a GPS, 
// accelerometer, <OTHER SENSORS>, and transmits it via an XBee
// radio. GPS data is transmitted in its raw form, and should be 
// parsed on the receiving end (unless we decide to have this
// info displayed to the driver). The same model XBee radio is 
// used as a receiver
//
// This code will be modified to transmit raw data packets over 
// the radios which will be parsed on the receiving machine. We
// can adopt the necesary functions from the file linked below.
//
// Adafruit parse .cpp file:
// https://github.com/adafruit/Adafruit_GPS/blob/master/src/NMEA_parse.cpp
// 
////////////////////////////////////////////////////////////////
//
// What still needs to be done here:
// - Get the proper baud rate set for the GPS and radio
// - Ensure nothing else needs to be done to read GPS data to
//    digital pins
// - Find how to connect XBee with either shield or direct
//    wiring, and add those pin initializations below
// - Add code in the loop() section to transmit data over Xbee
// - Add functionality for other hardware (accelerometer, etc.)
// 
// NOTE: Global variable currently use 62% of dynamic memory
//


#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

// GPS CONNECTIONS
//
// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// Connect the GPS TX (transmit) pin to Digital 8
// Connect the GPS RX (receive) pin to Digital 7

// XBEE RADIO CONNECTIONS
//
// Connect the Xbee Power pin to 5V
// Connect the Xbee Ground pin to ground
// Connect the Xbee DOut pin to Digital 6

// ACCELEROMETER CONNECTIONS
//
//
//
//
//


// Initialize pin numbers
SoftwareSerial mySerial(8, 7); // GPS pins (see above)
Adafruit_GPS GPS(&mySerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO  true


// The remained of this code will need modification once the baud rates of the radios
// are confirmed to be at their maximum
void setup()
{

  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  delay(5000);

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's
  GPS.begin(9600);

  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time

  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);
  // Ask for firmware version -- don't need this lol
  //mySerial.println(PMTK_Q_RELEASE);
}

uint32_t timer = millis();
void loop()                     // run over and over again
{
  // Read data from GPS
  char c = GPS.read();

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

    // Keep the parsing here for now, but the end goal is to find a way to do this on
    // either the receiving Arduino or the laptop to which it is connected
    //
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // Most of what's below will be done on the receiving end...unless we pass this info
  // to the driver.
  //
  // We will need to somehow include that if(GPS.fix) statement though...our code to 
  // transmit GPS data will probably fall inside of this if statement!
  /*
  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer

    Serial.print("<"); // Use < to indicate start of the message
    Serial.print("\nTime: ");
    if (GPS.hour < 10) { Serial.print('0'); }
    Serial.print(GPS.hour, DEC); Serial.print(':');
    if (GPS.minute < 10) { Serial.print('0'); }
    Serial.print(GPS.minute, DEC); Serial.print(':');
    if (GPS.seconds < 10) { Serial.print('0'); }
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    if (GPS.milliseconds < 10) {
      Serial.print("00");
    } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
      Serial.print("0");
    }
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);

      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
      Serial.print("Antenna status: "); Serial.println((int)GPS.antenna);
    }
  Serial.print(">"); // Use > to indicate end of message
  }
  */
}
