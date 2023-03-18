#include <SPI.h>
  #include <NRFLite.h>

  #define radio_ce 9
  #define radio_csn 10
  #define car_radio_id 0 //Mostly transmits
  #define pit_radio_id 1 //Mostly receives

  NRFLite radio;

 typedef struct Data {
  char text;
};

Data package;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

if (!radio.init(pit_radio_id, radio_ce, radio_csn)) {
      Serial.print("Radio failed to start...\n");
      while (1) {} //Hold in infinite loop
    }
    else {
      Serial.print("Radio Intialized!\n");
    }
}

void loop() {
  // put your main code here, to run repeatedly:
if (radio.hasData()) {
    radio.readData(&package);

    //Accelerometer output
    Serial.print(package.text);
  }
}
