int flowPin = 2;  // input pin for the flow sensor
float flowRate;   // flow rate in gal/min
float totalVolume; // total volume of liquid passed through the sensor in gallons
unsigned long previousMillis = 0;
unsigned long currentMillis;
unsigned long elapsedTime;
float calibrationFactor = 7.5;  // adjust this value according to the sensor's datasheet

void setup() {
  Serial.begin(9600);
  pinMode(flowPin, INPUT);
}

void loop() {
  currentMillis = millis();
  elapsedTime = currentMillis - previousMillis;
  if (elapsedTime > 1000) { // calculate flow rate every second
    float sensorReading = pulseIn(flowPin, HIGH);  // read the sensor's output
    flowRate = (sensorReading / calibrationFactor) * 60.0;  // convert to gal/min
    totalVolume += (flowRate / 60.0) * elapsedTime / 3785.41;  // add the volume passed during this interval to the total (1 gallon = 3785.41 mL)
    Serial.print("Flow rate: ");
    Serial.print(flowRate, 2);
    Serial.print(" gal/min\t");
    Serial.print("Total volume: ");
    Serial.print(totalVolume, 2);
    Serial.println(" gallons");
    previousMillis = currentMillis;
  }
}
