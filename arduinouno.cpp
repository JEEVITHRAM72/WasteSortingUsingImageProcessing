#include <AccelStepper.h>
#include <NewPing.h>

// Define motor pins
#define motorPin1 3
#define motorPin2 4
#define motorPin3 5
#define motorPin4 6

// Define sensor pins
#define trigPin 9
#define echoPin 10
#define proximityPin 8

// Stepper motor setup
AccelStepper stepper(AccelStepper::DRIVER, motorPin1, motorPin2);

// Ultrasonic sensor setup
NewPing sonar(trigPin, echoPin, 200);  // Max distance 200 cm

// Initial position of the stepper motor
int currentAngle = 0;

void setup() {
  Serial.begin(9600);
  
  // Stepper motor settings
  stepper.setMaxSpeed(500);
  stepper.setAcceleration(100);
  
  pinMode(proximityPin, INPUT);
}

void loop() {
  // Check if waste is detected by the ultrasonic sensor (within 10 cm)
  int distance = sonar.ping_cm();
  bool wasteDetected = distance > 0 && distance < 10;  // Adjust distance as needed

  // Check proximity sensor (e.g., a simple digital sensor)
  bool proximityDetected = digitalRead(proximityPin) == HIGH;

  if (wasteDetected || proximityDetected) {
    // Read Python serial data
    if (Serial.available() > 0) {
      String input = Serial.readStringUntil('\n');
      
      // Output action based on Python code
      if (input == "biodegradable") {
        moveStepperLeft(90); // Move left 90 degrees (counterclockwise)
      } else if (input == "non biodegradable") {
        moveStepperRight(90); // Move right 90 degrees (clockwise)
      } else {
        Serial.println("No valid output");
      }
    }
  }
  
  delay(100); // Delay to avoid flooding the serial monitor
}

// Move the stepper motor to the left (counterclockwise) by 90 degrees
void moveStepperLeft(int angle) {
  int steps = (angle * 200) / 360;  // Assuming 200 steps per revolution (1.8° per step)
  stepper.moveTo(steps);
  
  // Move the stepper motor to the desired position
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }
  
  currentAngle -= angle;
  if (currentAngle < 0) {
    currentAngle += 360;  // Keep angle within 0-360 range
  }
}

// Move the stepper motor to the right (clockwise) by 90 degrees
void moveStepperRight(int angle) {
  int steps = (angle * 200) / 360;  // Assuming 200 steps per revolution (1.8° per step)
  stepper.moveTo(-steps);
  
  // Move the stepper motor to the desired position
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }
  
  currentAngle += angle;
  if (currentAngle >= 360) {
    currentAngle -= 360;  // Keep angle within 0-360 range
  }
}

