// Define the pins for the flow meter sensor
const int flowMeterPin = 7;

// Define the variables
volatile float flowRate;
volatile unsigned int pulseCount;

float totalFuel = 0.0;

unsigned long previousMillis = 0;
const long interval = 1000;

void setup() {
  // Initialize the flow meter sensor
  pinMode(flowMeterPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(flowMeterPin), pulseCounter, FALLING);

  // Initialize the Serial communication
  Serial.begin(9600);
}

void loop() {
  // Calculate the flow rate and the amount of gasoline used
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    
    noInterrupts();
    flowRate = pulseCount / 7.5;
    pulseCount = 0;
    interrupts();
    
    totalFuel += flowRate * (interval / 1000.0);
  }
  
  // Print the data to the Serial line
  Serial.print("Gasoline Used: ");
  Serial.print(totalFuel, 2);
  Serial.println(" liters");
}

void pulseCounter() {
  pulseCount++;
}
