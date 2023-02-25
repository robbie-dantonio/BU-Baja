//Configure radio
  #include <SPI.h>
  #include <nRF24L01.h>
  #include <RF24.h>

  #define ce 4
  #define csn 5

  RF24 radio(ce, csn);

  const byte address[6] = "00001";

//Configuring data package as a struct with necessary data
typedef struct data {
  float lon;
  float lat;
  int flipped; //1 if car flipped over, 0 otherwise

  //speed
  float speedX;
  float speedY;
  float speedZ;
  float speed; //Speed determined by integrating acceleration
};

void setup() {
  //Setup serial monitor (for now)
  Serial.begin(115200); //Set to this baud rate to accomodate the load of GPS data that will be received

  //Setup radio
    radio.begin();

    //set the address
    radio.openReadingPipe(0, address);
    
    //Set module as transmitter
    radio.stopListening();
}

void loop() {
  data package;

  if (radio.available()) {
    radio.read(&package, sizeof(data));
  }

  Serial.print("Longitude: " + (String)package.lon + "\n");
  Serial.print("Latitude: " + (String)package.lat + "\n");
  Serial.print("Speed: " + (String)package.speed + "\n\n");
}