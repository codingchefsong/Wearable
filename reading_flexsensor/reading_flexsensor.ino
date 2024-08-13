#include <Adafruit_CircuitPlayground.h>

// Define the pins connected to the flex sensors
const int FLEX_PIN1 = A1;
const int FLEX_PIN2 = A2;
const int FLEX_PIN3 = A3;

// Variables to store the analog values read from the flex sensors
int flexVal1 = 0;
int flexVal2 = 0;
int flexVal3 = 0;

void setup() {
    // Initialize serial communication at a baud rate of 115200
    Serial.begin(115200);
    
    // Set the flex sensor pins as inputs
    pinMode(FLEX_PIN1, INPUT);
    pinMode(FLEX_PIN2, INPUT);
    pinMode(FLEX_PIN3, INPUT);
}

void loop() {
    // Read the analog value from the first flex sensor (connected to A1)
    flexVal1 = analogRead(FLEX_PIN1);
    // Print the value to the serial monitor
    Serial.print("A1:");
    Serial.print(flexVal1);

    // Read the analog value from the second flex sensor (connected to A2)
    flexVal2 = analogRead(FLEX_PIN2);
    // Print the value to the serial monitor
    Serial.print(",A2:");
    Serial.print(flexVal2);

    // Read the analog value from the third flex sensor (connected to A3)
    flexVal3 = analogRead(FLEX_PIN3);
    // Print the value to the serial monitor
    Serial.print(",A3:");
    Serial.println(flexVal3);
  
    // Wait for 500 milliseconds before the next reading
    delay(500);
}
