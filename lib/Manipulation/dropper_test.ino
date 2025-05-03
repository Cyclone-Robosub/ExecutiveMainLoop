#include <Servo.h>

Servo myServo;
const int servoPin = 9;

void setup() {
    Serial.begin(9600); // Start serial communication
    myServo.attach(servoPin); // Attach the servo to pin 9
    myServo.write(90); // Set initial position to 90 degrees
    Serial.println("Servo initialized at 90 degrees");
}

void loop() {
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n'); // Read input from serial monitor
        command.trim(); // Remove any whitespace

        if (command == "1") {
            myServo.write(80);
            Serial.println("Servo moved to 80 degrees");
        } 
        else if (command == "2") {
            myServo.write(100);
            Serial.println("Servo moved to 100 degrees");
        } 
        else if (command == "r") {
            myServo.write(90);
            Serial.println("Servo moved to 90 degrees");
        } 
        else {
            Serial.println("Invalid command. Use open1, open2, or rest.");
        }
    }
}
