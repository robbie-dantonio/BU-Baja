#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define ce 4
#define csn 5

RF24 radio(ce, csn);

const byte address[6] = "00001";

typedef struct location {
  float lat;
  float lon;
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
  location data;

  if (radio.available()) {
    radio.read(&data, sizeof(location));
  }

  Serial.print("Longitude: " + (String)data.lon + "\n");
  Serial.print("Latitude: " + (String)data.lat + "\n\n");
}