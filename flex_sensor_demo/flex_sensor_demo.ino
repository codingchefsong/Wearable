#include <Adafruit_CircuitPlayground.h>
#include <limits.h>

// Define the pins connected to the flex sensors
const int FLEX_PIN1 = A1;

// Variables to store the analog values read from the flex sensors
int flexVal1 = 0;
int pixeln = 0;
int maxV = INT_MIN;
int minV = INT_MAX;
double diff = 0;
int color = 0x220000;

void setup() {
    // Initialize serial communication at a baud rate of 115200
    Serial.begin(115200);
    
    // Set the flex sensor pins as inputs
    pinMode(FLEX_PIN1, INPUT);

    CircuitPlayground.begin();
}

void setLED(int num, uint32_t c){
  for(int i=0; i<num; i++){
    CircuitPlayground.setPixelColor(i, c);
  }

}

void loop() {
    CircuitPlayground.clearPixels();
    // Read the analog value from the first flex sensor (connected to A1)
    flexVal1 = analogRead(FLEX_PIN1);
    // Print the value to the serial monitor
    Serial.print("A1:");
    Serial.println(flexVal1);
    if(maxV==INT_MIN){
      maxV=flexVal1;
    }
    if(flexVal1 < minV){
      minV = flexVal1 ;
    }else if(minV < 1){
      minV = INT_MAX ;
    }else{
      diff = (maxV-flexVal1) * 100.0/(maxV - minV);
      Serial.print(maxV);
      Serial.print(",");
      Serial.print(minV);
      Serial.print(",");
      Serial.println(diff);
      if (diff > 0){
        setLED(diff/10+1, color);
      }
    }
    // Wait for 5 milliseconds before the next reading
    delay(5);
}
