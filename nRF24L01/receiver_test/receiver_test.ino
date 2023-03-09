#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define CE_PIN   9
#define CSN_PIN 10

const byte thisSlaveAddress[5] = {'R','x','A','A','A'};

RF24 radio(CE_PIN, CSN_PIN);

char dataReceived[10]; // this must match dataToSend in the TX
bool newData = false;

void setup() {
    Serial.begin(9600);

    // Reset the radio module
    /*pinMode(CE_PIN, OUTPUT);
    digitalWrite(CE_PIN, LOW);
    delayMicroseconds(10);
    digitalWrite(CE_PIN, HIGH);*/

    radio.begin();
    radio.setDataRate(RF24_250KBPS);
    radio.openReadingPipe(1, thisSlaveAddress);
    radio.startListening();
    radio.setPALevel(RF24_PA_MIN);
}

void loop() {
    getData();
    showData();
    delay(1000);
}

void getData() {
    if (radio.available()) {
        radio.read( &dataReceived, sizeof(dataReceived) );

        Serial.print("Output: " + (String)dataReceived + "\n");
        Serial.print("Size of Output: " + (String)sizeof(dataReceived) + "\n");
        Serial.print("First character: " + (String)dataReceived[0] + "\n");

        memset(&dataReceived, 0, sizeof(dataReceived)); // clear the dataReceived array
        newData = true;
    }
}

void showData() {
    if (newData == true) {
        Serial.print("Data received: ");
        Serial.println(dataReceived);
        newData = false;
    }
}
