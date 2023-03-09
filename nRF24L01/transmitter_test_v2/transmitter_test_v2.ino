//Include Libraries
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

//create an RF24 object
RF24 radio(9, 10);  // CE, CSN

//address through which two modules communicate.
const byte address[6] = "xiang";

void setup()
{
  while (!Serial);
    Serial.begin(9600);
  
  SPI.begin(); //Start SPI interface
  /*radio.begin();
  if(!radio.begin()) {
    Serial.print("Radio module not responding!\n");
    while (1) {}
  }
  else {
    Serial.print("Radio Module Initialized!\n");
    delay(5000);
  }*/
  
  //set the address
  radio.openWritingPipe(address);
  
  //Set module as receiver
  radio.stopListening();
}

void loop()
{
  //Write data to buffer
  char text[32] = "Hello World!";
  radio.write(&text, sizeof(text));
  Serial.print("Message sent.\n");
}