/****************************************************
 * BU BAJA SAE dashboard code
 * 
 * Screen (TFT) Pins:
 *  D/C   9
 *  CS    10
 *  MOSI  11
 *  SCK   13
 *  
 * GPS Pins:
 *  TX    7
 *  RX    8
 *  
 * Speedometer Stepper Motor Pins
 *  Ain2  3
 *  Ain1  4
 *  Bin1  5
 *  Bin2  6
 *  
 * Accelerometer Pins
 *  Connect to SCL and SDA  
 ****************************************************/

#include <SoftwareSerial.h>
#include <Stepper.h>
#include <Adafruit_GPS.h>
#include <SPI.h>
#include <Wire.h>
#include "Adafruit_MMA8451.h"
#include "Adafruit_Sensor.h"
#include "NRFLite.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#include "Fonts/FreeSansBold24pt7b.h"



/* Define pins******************************************************************************/
#define FLOW_PIN  2   // Flow meter

#define A_IN2     3   // Speedometer stepper motor
#define A_IN1     4
#define B_IN1     5
#define B_IN2     6

#define GPS_TX    7   // GPS
#define GPS_RX    8

#define TFT_DC    9   // Screen (TFT)
#define TFT_CS    10
#define TFT_MOSI  11
#define TFT_SCK   13

#define RADIO_CSN 48  // Radio
#define RADIO_CE  49 



/* Structs *******************************************************************************/
// Radio: Packet struct, gets transmitted by the radio
typedef struct packet {
  float lon;
  float lat;
  int orientation; //What each value means in accelerometer program section

  //speed
  float speed; //Speed determined by GPS module

  //gps info
  int gpsDataAvailable;
};

// Radio: Status struct, used for keeping data together
typedef struct status {
    int radioOk;  //1 if radio on, 0 otherwise
    int radioSending; //1 if radio transmitting, 0 otherwise

    //GPS Data
    float lon;
    float lat;
    float speed;

    //Accelerometer Data
    int accOk; //1 if accelerometer initialized, 0 otherwise
    float accX;
    float accY;
    float accZ;
    int orientation;

    int gpsDataAvailable; //1 if gps has data, 0 otherwise
    int gpsFix; //1 if gps has fix, 0 otherwise
  };



/* Globals/constants ******************************************************************************/
#define STEPS_PER_REVOLUTION  600 // Speedometer: hardware maximum number of steps
#define MAX_STEPS 500             // Speedometer: our defined step limit

#define CAR_RADIO_ID 0            // Radio: Mostly transmits
#define PIT_RADIO_ID 1            // Radio: Mostly receives

#define ACC_RANGE 4               // Accelerometer: Range of accelerometer (in Gs)

static int stepCount;             // Speedometer: used in step calculations

uint32_t timer;                   // GPS: timer used in demo code, might not be necesary

uint32_t startTime;               // Flow Meter: Used to keep track of time when calculating fuel consumption
const float pulseFactor = 0.00066;// Flow Meter: volume of each pulse in gallons. THIS NEEDS TO BE TESTED AND CHANGED
float flowRate;                   // Flow Meter: flow rate in gallons per minute
float totalVolume;                // Flow Meter: total volume of liquid passed through the sensor in gallons
unsigned int pulseCount;          // Flow meter

NRFLite radio;                    // Radio: NRFLite object
packet Package;                   // Radio: Instance of packet struct, to be transmitted
status Status;                    // Radio: Instance of status struct, stores info regarding operations of individual parts


/* Methods ************************************************************************************/
unsigned long dashboardInit();                /* Initializes dashboard electronics            */
void gpsInit();                               /* Initializes GPS module                       */
void accInit();                               /* Initializes accelerometer                    */
void flowMeterInit();                         /* Initializes flow meter                       */
void radioInit();                             /* Initializes NRF24L01 radio module            */
unsigned long showSpeed(int current_speed);   /* Sets screen and speedometer to current_speed */
void gpsOps();                                /* Handles GPS operations                       */
void accOps();                                /* Handles accelerometer operations             */
void flowMeterOps ();                         /* Handles flow meter operations                */


/* TEMP Variables for testing ******************************************************************/
int current_speed;



/* Initialize class constructors ***************************************************************/
SoftwareSerial mySerial(GPS_TX, GPS_RX); // GPS
Adafruit_GPS GPS(&mySerial);  //GPS
Stepper speedo(STEPS_PER_REVOLUTION, A_IN2, A_IN1, B_IN1, B_IN2); // Speedometer 
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);  // Screen
Adafruit_MMA8451 mma = Adafruit_MMA8451();  // Accelerometer



/* Setup/Loop ************************************************************************************/

void setup() {
  startTime = millis();
  timer = millis();
  
  gpsInit();
  flowMeterInit();
  radioInit();
  accInit();
  dashboardInit();
}


void loop(void) {
  flowMeterOps();
  gpsOps();
  accOps();
  showSpeed(Status.speed);

  // Radio stuff
  Package.gpsDataAvailable = Status.gpsDataAvailable;
  Package.lat = Status.lat;
  Package.lon = Status.lon;
  Package.orientation = Status.orientation;
  Package.speed = Status.speed;

  if (!radio.send(PIT_RADIO_ID, &Package, sizeof(Package))) {
    Status.radioSending = 0;
  }
  else {
    Status.radioSending = 1;
  }
}


/* Methods ************************************************************************************/

/* Flow Meter Operations */
void flowMeterOps(){
  if (digitalRead(FLOW_PIN) == LOW) {
    pulseCount++;  // increment the pulse count when a pulse is detected
  }
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - startTime;
  if (elapsedTime >= 1000) {  // calculate flow rate every second
    flowRate = pulseCount * pulseFactor * 60.0;  // calculate flow rate in gallons per minute
    totalVolume += flowRate / 60.0;  // add the volume passed during this interval to the total
    Serial.print("Flow rate: ");
    Serial.print(flowRate, 2);
    Serial.print(" gal/min\t");
    Serial.print("Total volume: ");
    Serial.print(totalVolume, 2);
    Serial.println(" gallons");
    pulseCount = 0;  // reset the pulse count
    startTime = currentTime;  // reset the start time
  }
}
/* GPS Operations */
void gpsOps(){
  //Receive GPS data
    char c = GPS.read();
    if (GPS.fix) {
      //Set status for gps
      Status.gpsFix = 1;

      if (GPS.newNMEAreceived()) { 
        Status.gpsDataAvailable = 1; //Record whether gps data is available

        if (GPS.parse(GPS.lastNMEA())) { //Parse GPS data
          //Get data
          Status.speed = GPS.speed;

          float latitude = GPS.latitude;
          float longitude = GPS.longitude;
          Status.lat = latitude;
          Status.lon = longitude;

          Serial.print("Latitude: " + (String)latitude + " Longitude: " + (String)longitude + "\n");
        }
      }
      else {
        Status.gpsDataAvailable = 0;
      }
    }
    else {
      Status.gpsFix = 0;
    }
}

/* Accelerometer Operations */
void accOps(){
  //Receive accelerometer data
    //Get accelerations
    if (Status.accOk) {
      /* Get a new sensor event, measure time elapsed */ 
        //float timeStart = millis();
        sensors_event_t event; 
        mma.getEvent(&event);
        //float timeEnd = millis();
        //float timeElapsed = timeEnd-timeStart;

      /* Display the results (acceleration is measured in m/s^2) */
      Status.accX = event.acceleration.x;
      Status.accY = event.acceleration.y;
      Status.accZ = event.acceleration.z;

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

      Status.orientation = mma.getOrientation();
      //Serial.print("Orientation: " + (String)status.orientation + "\n");
    }
}

/* Initializes accelerometer module */
void accInit(){
  if (!mma.begin()) {
    Serial.println("Adafruit MMA8451 Couldn't start");
    Status.accOk = 0; //Set marker indicating accelerator is disabled
  }
  else {
    //Set accelerometer status
    Status.accOk = 1;
  
    //Set range
    switch (ACC_RANGE) {
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


/* Initializes radio module */
void radioInit(){
  if (!radio.init(CAR_RADIO_ID, RADIO_CE, RADIO_CSN)) {
    Status.radioOk = 0;
  }
  else {
    Status.radioOk = 1;
  }
}


/* Initializes flow meter */
void flowMeterInit(){
  pinMode(FLOW_PIN, INPUT_PULLUP);
  pulseCount = 0;
}


/* Initiliaze GPS module */
void gpsInit(){
  GPS.begin(9600);  // Set GPS to 9600 NMEA - default baud rate for GPS
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // Turn on recommended minimum data
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);  // 1 Hz update rate
  //GPS.sendCommand(PGCMD_ANTENNA); // Request updates on antenna - maybe can comment this out
}


/* Initialize dashboard electronics */
unsigned long dashboardInit(){
  // Initialize speedometer needle
  speedo.setSpeed(60);
  speedo.step(-STEPS_PER_REVOLUTION);
  speedo.step(STEPS_PER_REVOLUTION);
  speedo.step(-STEPS_PER_REVOLUTION);
  stepCount = 0;

  // Initialize speed on screen
  unsigned long start = micros();
  tft.begin();
  tft.fillScreen(ILI9341_BLACK);
  tft.setRotation(1);
  tft.setFont(&FreeSansBold24pt7b);
  tft.setTextColor(ILI9341_WHITE);    
  tft.setTextSize(1);
  tft.setCursor(0, 38);
  tft.print("0");
  tft.setCursor(43, 38);
  tft.print(" mph");

  // Initialize fuel gauge on screen
  tft.setCursor(0, 215);
  tft.setFont();
  tft.setTextColor(ILI9341_BLUE);    
  tft.setTextSize(3);
  tft.print("Fuel");
  delay(70);
  tft.fillRoundRect(80, 195, 25, 40, 4, ILI9341_RED);     delay(70);
  tft.fillRoundRect(110, 195, 25, 40, 4, ILI9341_RED);    delay(70);
  tft.fillRoundRect(140, 195, 25, 40, 4, ILI9341_ORANGE); delay(70);
  tft.fillRoundRect(170, 195, 25, 40, 4, ILI9341_ORANGE); delay(70);
  tft.fillRoundRect(200, 195, 25, 40, 4, ILI9341_GREEN);  delay(70);
  tft.fillRoundRect(230, 195, 25, 40, 4, ILI9341_GREEN);  delay(70);
  tft.fillRoundRect(260, 195, 25, 40, 4, ILI9341_GREEN);  delay(70);
  tft.fillRoundRect(290, 195, 25, 40, 4, ILI9341_GREEN);  
  delay(1000);
  
  return micros() - start;
}



/* Display speed on dashboard */
unsigned long showSpeed(int current_speed){
  // Speedometer needle
  int gpsSteps = map(current_speed, 0, 20, 0, MAX_STEPS);
  speedo.step(gpsSteps - stepCount);
  stepCount = stepCount + (gpsSteps - stepCount); 
  
  // Screen - Speed
  unsigned long start = micros();
  tft.fillRect(0, 0, 55, 60, ILI9341_BLACK);
  tft.setFont(&FreeSansBold24pt7b);
  tft.setTextColor(ILI9341_WHITE);    
  tft.setTextSize(1);
  tft.setCursor(0, 38);
  tft.print(current_speed);
  //tft.print(" mph");

  // Screen - Fuel gauge
  tft.setCursor(0, 215);
  tft.setFont();
  tft.setTextColor(ILI9341_BLUE);    
  tft.setTextSize(3);
  tft.print("Fuel");
  tft.fillRoundRect(80, 195, 25, 40, 4, ILI9341_RED);
  tft.fillRoundRect(110, 195, 25, 40, 4, ILI9341_RED);
  tft.fillRoundRect(140, 195, 25, 40, 4, ILI9341_ORANGE);
  tft.fillRoundRect(170, 195, 25, 40, 4, ILI9341_ORANGE);
  tft.fillRoundRect(200, 195, 25, 40, 4, ILI9341_GREEN);
  tft.fillRoundRect(230, 195, 25, 40, 4, ILI9341_GREEN);
  tft.fillRoundRect(260, 195, 25, 40, 4, ILI9341_GREEN);
  tft.fillRoundRect(290, 195, 25, 40, 4, ILI9341_GREEN);
  return micros() - start;
}
