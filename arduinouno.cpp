#include <Servo.h>

Servo myServo;  // Servo object
int servoPin = 9;  // Pin where the servo is connected
int ultrasonicTrigPin = 10;  // Trigger pin for ultrasonic
int ultrasonicEchoPin = 11;  // Echo pin for ultrasonic
int proximityPin = 8;  // Proximity sensor pin
long duration, distance;

int pos = 0;  // Initial servo position

void setup() {
  Serial.begin(9600);  // Initialize serial communication
  myServo.attach(servoPin);  // Attach the servo to the pin
  myServo.write(pos);  // Set servo to initial position (0 degrees)

  // Initialize sensor pins
  pinMode(ultrasonicTrigPin, OUTPUT);
  pinMode(ultrasonicEchoPin, INPUT);
  pinMode(proximityPin, INPUT);
}

void loop() {
  if (Serial.available()) {
    char command = Serial.read();  // Read the incoming byte

    if (command == 'c') {  // Check for waste detection
      // Measure distance using ultrasonic sensor
      digitalWrite(ultrasonicTrigPin, LOW);
      delayMicroseconds(2);
      digitalWrite(ultrasonicTrigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(ultrasonicTrigPin, LOW);

      duration = pulseIn(ultrasonicEchoPin, HIGH);
      distance = (duration / 2) * 0.0344;  // Calculate distance in cm

      // Check proximity sensor
      int proximityValue = digitalRead(proximityPin);  // Read proximity sensor state

      // If either ultrasonic or proximity sensor detects waste
      if (distance < 50 || proximityValue == HIGH) {
        Serial.println("Waste detected!");  // Print waste detection
        delay(500);  // Prevent repeated triggering

        // Move servo based on previous command (from Python)
        if (Serial.available()) {
          char moveCommand = Serial.read();
          if (moveCommand == 'l') {
            pos = 90;  // Move servo to 90 degrees left (biodegradable)
            myServo.write(pos);
            delay(500);  // Wait for the servo to reach position
          }
          else if (moveCommand == 'r') {
            pos = 90;  // Move servo to 90 degrees right (non-biodegradable)
            myServo.write(pos);
            delay(500);  // Wait for the servo to reach position
          }
        }
      }
    }
  }
}
