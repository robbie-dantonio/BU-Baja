/*Receiver code, connected to ground station in the pits*/

//Configure radio
  #include <SPI.h>
  #include <NRFLite.h>

  #define radio_ce 9
  #define radio_csn 10
  #define car_radio_id 0 //Mostly transmits
  #define pit_radio_id 1 //Mostly receives

  //Configuring data package as a struct with necessary data
  typedef struct Data {
    float lon;
    float lat;
    int orientation; //What each value means in accelerometer program section

    //speed
    float speed; //Speed determined by GPS module

    //gps info
    int gpsDataAvailable;

    //Call back to pit
    int recall; //1 if car wants to recall to pit, 0 otherwise
  };

  //Initialize Data package
  Data package;

  //Initialize radio object
  NRFLite radio;

void setup() {
  //Setup serial monitor (for now)
  Serial.begin(115200); //Set to this baud rate to accomodate the load of GPS data that will be received

  //Setup radio
    if (!radio.init(pit_radio_id, radio_ce, radio_csn)) {
      Serial.print("Radio failed to start...\n");
      while (1) {} //Hold in infinite loop
    }
    else {
      Serial.print("Radio Intialized!\n");
    }
}

int test_counter = 0;

void loop() {
  //Receive package
  if (radio.hasData()) {
    radio.readData(&package);

    //Test connection
    test_counter++;
    Serial.print("Packet received " + (String)test_counter + "\n");

    //Accelerometer output
    //Serial.print("Orientation: " + (String)package.orientation + "\n");
    Serial.print("Speed: " + (String)package.speedX + "\n");
  }

  //GPS Output
/*  Serial.print("Longitude: " + (String)package.lon + "\n");
  Serial.print("Latitude: " + (String)package.lat + "\n");
  Serial.print("Speed: " + (String)package.speed + "\n\n");*/
}