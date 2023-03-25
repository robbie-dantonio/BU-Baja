/*
Transmitter code, goes in microcontroller on car. Receives GPS data from GPS module, 
sends data to receiver via transmitter.
*/

//Configure flow meter (measures tank level)
  // Define the pins for the flow meter sensor
  /*const int flowMeterPin = 3; //Changed since pin 7 is used by gpsOut

  // Define the variables
  volatile float flowRate;
  volatile unsigned int pulseCount;

  float totalFuel = 0.0;

  unsigned long previousMillis = 0;
  const long interval = 1000;*/

//Configure radio module
  #include <SPI.h>
  #include <NRFLite.h>

  #define radio_ce 49
  #define radio_csn 48
  #define car_radio_id 0 //Mostly transmits
  #define pit_radio_id 1 //Mostly receives

  //Configuring data package as a struct with necessary data
  typedef struct Data {
    float lon;
    float lat;
    int orientation; //What each value means in accelerometer program section

    //speed
    float speed; //Speed determined by integrating acceleration

    //gps info
    int gpsDataAvailable;
  };

  //Initialize data package
  Data package;

  //Initialize radio
  NRFLite radio;

//Configure GPS module
  #include <Adafruit_GPS.h>

  #define gpsIn 6
  #define gpsOut 7

  SoftwareSerial mySerial(gpsIn, gpsOut);
  Adafruit_GPS GPS(&mySerial);

  void gpsSetup(int); //Will setup GPS module

  //GPS operations
  void gpsOps();

//Configure accelerometer
  #include <Wire.h>
  #include <Adafruit_MMA8451.h>
  #include <Adafruit_Sensor.h>

  Adafruit_MMA8451 mma = Adafruit_MMA8451();

  #define accRange 4 //Range of accelerometer (in Gs)
  void accSetup(); //Setup for accelerometer
  void accOps(); //Acceleration operations

  //Speed calculation (deprecated because of inaccurate results)
  //void calcSpeed (Data *, float, float, float, float);

//Configure LCD
  typedef struct Status {
    int radioOk;  //1 if radio on, 0 otherwise
    int radioSending; //1 if radio transmitting, 0 otherwise

    int accOk; //1 if accelerometer initialized, 0 otherwise
    float accX;
    float accY;
    float accZ;

    int gpsDataAvailable; //1 if gps has data, 0 otherwise
    int gpsFix; //1 if gps has fix, 0 otherwise
  };

  //Initialize status struct; will store information regarding the operations of the individual parts
  Status status;

//Configure stepper motor
  #include <Stepper.h>

  #define STEPS 200

  #define AIN2 4
  #define AIN1 5
  #define BIN1 6
  #define BIN2 7
  Stepper stepper (STEPS, AIN2, AIN1, BIN1, BIN2);

  #define speedRange 35 //Range is from 0 to 35 mph
  int stepPos = 0; //Number of steps the pointer is offset from 0 position
  int calcSteps (int, int, float); //steps, range, speed, outputs how many steps the pointer needs to move

void setup() {
  //Setup for flow meter
    // Initialize the flow meter sensor
    //pinMode(flowMeterPin, INPUT_PULLUP);
    //attachInterrupt(digitalPinToInterrupt(flowMeterPin), pulseCounter, FALLING);

  //Setup serial connection for LCD
  Serial.begin(115200);

  //Setup for LCD display


  //Setup for radio module, set radio status
    if (!radio.init(car_radio_id, radio_ce, radio_csn)) {
      status.radioOk = 0;
    }
    else {
      status.radioOk = 1;
    }

  //Define variables in 'Data' package

  //Setup gps
  gpsSetup();

  //Setup Accelerometer
  accSetup();

  //Setup for stepper motor
    stepper.setSpeed(60); //Speed set to 30 RPM
    //Test run stepper, used for visual diagnostics and to set pointer at 0 point
    stepper.step(-STEPS);
    stepper.step(STEPS);
    stepper.step(-STEPS);
}

void loop() {
  //GPS operations
    gpsOps();

  // Calculate the flow rate and the amount of gasoline used (flow meter)
    //unsigned long currentMillis = millis();
    /*if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      
      noInterrupts();
      flowRate = pulseCount / 7.5;
      pulseCount = 0;
      interrupts();
      
      totalFuel += flowRate * (interval / 1000.0);
    }*/

  //Acceleration ops
    accOps();

  //Send data to receiver (for printing to serial monitor for now)
    if (!radio.send(pit_radio_id, &package, sizeof(package))) {
      status.radioSending = 0;
    }
    else {
      status.radioSending = 1;
    }


  //LCD stuff
    //Output telemetry data (speed, accelerations)
    //Output status

  //Stepper motor
  if (status.gpsDataAvailable) {
    int stepsNeeded = calcSteps(stepPos, speedRange, package.speed);
    stepper.step(stepsNeeded);
    stepPos += stepsNeeded;
  }
}

void accOps () {
  //Receive accelerometer data
    //Get accelerations
    if (!status.accOk) {
      /* Get a new sensor event, measure time elapsed */ 
        //float timeStart = millis();
        sensors_event_t event; 
        mma.getEvent(&event);
        //float timeEnd = millis();
        //float timeElapsed = timeEnd-timeStart;

      /* Display the results (acceleration is measured in m/s^2) */
      status.accX = event.acceleration.x;
      status.accY = event.acceleration.y;
      status.accZ = event.acceleration.z;

      //Calculate speed (deprecated because of inaccurate results)
        //calcSpeed (&package, timeElapsed, accX, accY, accZ);
 
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

      package.orientation = mma.getOrientation();
      Serial.print("Orientation: " + (String)package.orientation + "\n");
      //Need to figure out what the orientations are, will do this for now:
        /*if (zdir < 0)
          package.flipped = 1; //1 for flipped, 0 for anything else
        else
          package.flipped = 0;*/
    }
}

void accSetup () {
  //Setup for accelerometer
    //MMA8451 status
      if (!mma.begin()) {
        Serial.println("Adafruit MMA8451 Couldn't start");
        status.accOk = 0; //Set marker indicating accelerator is disabled
      }
      else {
        //Set accelerometer status
        status.accOk = 1;
      
        //Set range
        switch (accRange) {
          case 2:
            mma.setRange(MMA8451_RANGE_2_G);
            break;
          case 4:
            mma.setRange(MMA8451_RANGE_4_G);
            break;
          case 8:
            mma.setRange(MMA8451_RANGE_8_G);
            break;          
        }
    
        //Output set range
          int range = mma.getRange();
          Serial.println("MMA8451 range set to " + (String)(pow(2, range+1)) + " G");
      }
}

void gpsOps () {
  //Receive GPS data
    char c = GPS.read();
    if (GPS.fix) {
      //Set status for gps
      status.gpsFix = 1;

      if (GPS.newNMEAreceived()) {
        package.gpsDataAvailable = 1; //Tell ground station that gps package was received
        status.gpsDataAvailable = 1;
        if (GPS.parse(GPS.lastNMEA())) { //Parse GPS data
          //Get data
          package.speed = GPS.speed;

          float latitude = GPS.latitude;
          float longitude = GPS.longitude;
          package.lat = latitude;
          package.lon = longitude;

          Serial.print("Latitude: " + (String)latitude + " Longitude: " + (String)longitude + "\n");
        }
      }
      else {
        package.gpsDataAvailable = 0;
      }
    }
    else {
      status.gpsFix = 0;
    }
}

void gpsSetup() {
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
}

int calcSteps (int steps, int range, float speed) {
  int targetPosition = ceil(STEPS*(speed/range));
  return targetPosition - steps;
}

/*void calcSpeed (Data *package, float timeElapsed, float xAcc, float yAcc, float zAcc) {
  //"Integrate"
  float speedChangeX = package->speedX + xAcc*timeElapsed;
  float speedChangeY = package->speedY + yAcc*timeElapsed;
  float speedChangeZ = package->speedZ + zAcc*timeElapsed;

  //Update speed info
  package->speedX += speedChangeX;
  package->speedY += speedChangeY;
  package->speedZ += speedChangeZ;
  package->speed += pow(pow(speedChangeX, 2) + pow(speedChangeY, 2) + pow(speedChangeZ, 2), 0.5); 
  package->speed = abs(package->speed - 9.7); //IDK why speed is around 9.7, just did an offset and will see if this is accurate -Xiang
  //Serial.print("Speed X: " + (String)package->speed + "\n"); //-> Work on why speeds tend towards 9.8?
}*/