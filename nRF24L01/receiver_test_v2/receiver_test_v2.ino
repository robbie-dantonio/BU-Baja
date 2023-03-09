//Include Libraries
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

//create an RF24 object
RF24 radio(49, 48);  // CE, CSN

//address through which two modules communicate.
const byte address[6] = "xiang";

void setup()
{
  while (!Serial);
    Serial.begin(9600);
  
  SPI.begin(); //Start SPI interface
  if(!radio.begin()) {
    Serial.print("Radio module not responding!\n");
    while (1) {}
  }
  else {
    Serial.print("Radio Module Initialized!\n");
    delay(5000);
  }
  
  //set the address
  radio.openReadingPipe(0, address);
  
  //Set module as receiver
  radio.startListening();
}

void loop()
{
  //Read the data if available in buffer
  if (radio.available())
  {
    char text[32] = {0};
    radio.read(&text, sizeof(text));
    Serial.println(text);
  }
}