/*
Transmitter code, goes in microcontroller on car. Receives GPS data from GPS module, 
sends data to receiver via transmitter.
*/

//Configure radio module
  #include <SPI.h>
  #include <nRF24L01.h>
  #include <RF24.h>

  #define ce 4
  #define csn 5

  RF24 radio(ce, csn);

  const byte address[6] = "00001";

//Configure GPS module
  #include <Adafruit_GPS.h>

  #define gpsIn 6
  #define gpsOut 7

  SoftwareSerial mySerial(6, 7);
  Adafruit_GPS GPS(&mySerial);

//Configure accelerometer
  #include <Wire.h>
  #include <Adafruit_MMA8451.h>
  #include <Adafruit_Sensor.h>

  Adafruit_MMA8451 mma = Adafruit_MMA8451();

//Configure LCD


//Configure stepper motor
  #include <Stepper.h>

  #define STEPS 200

  #define AIN2 8
  #define AIN1 9
  #define BIN1 10
  #define BIN2 11
  Stepper stepper (STEPS, AIN2, AIN1, BIN1, BIN2);

//Configuring data package as a struct with longitude and latitude
typedef struct data {
  float lon;
  float lat;
  int flipped; //1 if car flipped over, 0 otherwise
};

void setup() {
  //Setup serial connection for LCD


  //Setup for radio module
    radio.begin();

    //set the address
    radio.openWritingPipe(address);
    
    //Set module as transmitter
    radio.stopListening();

  //Setup for GPS module ->> currently set to RMC and GGA
    // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    // uncomment this line to turn on only the "minimum recommended" data for high update rates!
    //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
    // uncomment this line to turn on all the available data - for 9600 baud you'll want 1 Hz rate
    //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_ALLDATA);
  
    // Set the update rate ->> currently set to 1Hz
    // 1 Hz update rate
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    // 5 Hz update rate- for 9600 baud you'll have to set the output to RMC or RMCGGA only (see above)
    //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
    // 10 Hz update rate - for 9600 baud you'll have to set the output to RMC only (see above)
    //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);

  //Setup for accelerometer
    //MMA8451 status
      if (!mma.begin()) {
        Serial.println("Adaruit MMA8451 Couldn't start");
        while (1);
      }
      Serial.println("MMA8451 found!");

    //Set range (currently set to 4Gs)
      //mma.setRange(MMA8451_RANGE_2_G);
      mma.setRange(MMA8451_RANGE_4_G);
      //mma.setRange(MMA8451_RANGE_8_G);
    
    //Output set range
      int range = mma.getRange();
      Serial.println("MMA8451 range set to " + (String)pow(2, range) + " G");

  //Setup for stepper motor
    stepper.setSpeed(60); //Speed set to 30 RPM
}

void loop() {
  //Initialize data package
  data package;

  //Receive GPS data
  char c = GPS.read();
  int package_received = GPS.newNMEAreceived();
  if (package_received) {
    if (GPS.parse(GPS.lastNMEA())) { //Parse GPS data
      //Get data
      float latitude = GPS.latitude;
      float longitude = GPS.longitude;
      package.lat = latitude;
      package.lon = longitude;

      //Send data to receiver (for printing to serial monitor for now)
      radio.write(&package, sizeof(package));
    }
  }

  //Receive accelerometer data
    //Get accelerations
      /* Get a new sensor event */ 
        sensors_event_t event; 
        mma.getEvent(&event);

      /* Display the results (acceleration is measured in m/s^2) */
      float xdir = event.acceleration.x;
      float ydir = event.acceleration.y;
      float zdir = event.acceleration.z;
    
    //Get orientation
      /*The return value ranges from 0 to 7
      0: Portrait Up Front
      1: Portrait Up Back
      2: Portrait Down Front
      3: Portrait Down Back
      4: Landscape Right Front
      5: Landscape Right Back
      6: Landscape Left Front
      7: Landscape Left Back */

      int orientation = mma.getOrientation();
      //Need to figure out what the orientations are, will do this for now:
        if (zdir < 0)
          package.flipped = 1; //1 for flipped, 0 for anything else
        else
          package.flipped = 0;

  //LCD stuff


  //Stepper motor


}