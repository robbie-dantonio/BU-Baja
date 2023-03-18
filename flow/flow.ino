const int flowPin = 2;  // input pin for the flow sensor
const float pulseFactor = 0.00066;  // volume of each pulse in gallons. THIS NEEDS TO BE TESTED AND CHANGED
float flowRate;  // flow rate in gallons per minute
float totalVolume; // total volume of liquid passed through the sensor in gallons
unsigned long startTime;
unsigned int pulseCount;

void setup() {
  Serial.begin(9600);
  pinMode(flowPin, INPUT_PULLUP);
  pulseCount = 0;
  startTime = millis();
}

void loop() {
  if (digitalRead(flowPin) == LOW) {
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
