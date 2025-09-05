#include <Servo.h>

#define S0 4
#define S1 5
#define S2 6
#define S3 7
#define sensorOut 8

Servo servo1;  // First servo
Servo servo2;  // Second servo

int R = 0, G = 0, B = 0;

int currentServo1 = 160; // Track current position of servo1
int currentServo2 = 90;  // Track current position of servo2
const int stepDelay = 20; // Delay per degree step (adjust for speed)

void moveServoSlowly(Servo &servo, int &currentAngle, int targetAngle) {
  if (currentAngle < targetAngle) {
    for (; currentAngle <= targetAngle; currentAngle += 1) {
      servo.write(currentAngle);
      delay(stepDelay);
    }
  } else {
    for (; currentAngle >= targetAngle; currentAngle -= 1) {
      servo.write(currentAngle);
      delay(stepDelay);
    }
  }
}

void setup() {
  Serial.begin(9600);
  servo1.attach(10);   // Attach first servo to pin 10
  servo2.attach(9);    // Attach second servo to pin 9
  servo1.write(currentServo1);
  servo2.write(currentServo2);
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);

  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
}

void loop() {
  if (Serial.available()) {
    int command = Serial.parseInt();

    if (command == 1) {
      // Moving to default position
      moveServoSlowly(servo1, currentServo1, 160);
      moveServoSlowly(servo2, currentServo2, 90);
    }

    if (command == 2) {
      // Picking and placing sequence
      delay(1000); // Initial pause
      moveServoSlowly(servo1, currentServo1, 150);
      moveServoSlowly(servo2, currentServo2, 180);
      moveServoSlowly(servo1, currentServo1, 180);
      moveServoSlowly(servo2, currentServo2, 20);
      moveServoSlowly(servo1, currentServo1, 150);
      moveServoSlowly(servo2, currentServo2, 90);
      moveServoSlowly(servo1, currentServo1, 160);
      delay(5000); // Final pause
    }

    if (command == 3) {
      // Picking and moving sequence
      delay(500); // Initial pause
      moveServoSlowly(servo1, currentServo1, 150);
      moveServoSlowly(servo2, currentServo2, 180);
      moveServoSlowly(servo1, currentServo1, 170);
      moveServoSlowly(servo2, currentServo2, 90);
    }

    if (command == 4) {
      // Picking and moving sequence
      delay(500); // Initial pause
      moveServoSlowly(servo2, currentServo2, 180);
      moveServoSlowly(servo1, currentServo1, 150);
      moveServoSlowly(servo2, currentServo2, 90);
      moveServoSlowly(servo1, currentServo1, 160);
    }

    if (command == 5) {
      // Picking down and placing sequence
      delay(1000); // Initial pause
      moveServoSlowly(servo1, currentServo1, 150);
      moveServoSlowly(servo2, currentServo2, 20);
      moveServoSlowly(servo1, currentServo1, 180);
      moveServoSlowly(servo2, currentServo2, 180);
      moveServoSlowly(servo1, currentServo1, 150);
      moveServoSlowly(servo2, currentServo2, 90);
      moveServoSlowly(servo1, currentServo1, 160);
      delay(5000); // Final pause
    }
    if (command == 6) {
      digitalWrite(S2, LOW); digitalWrite(S3, LOW);
      R = pulseIn(sensorOut, LOW);
      delay(100);

      // Read Green
      digitalWrite(S2, HIGH); digitalWrite(S3, HIGH);
      G = pulseIn(sensorOut, LOW);
      delay(100);

      // Read Blue
      digitalWrite(S2, LOW); digitalWrite(S3, HIGH);
      B = pulseIn(sensorOut, LOW);
      delay(100);

      // Send result
      if (R < G && R < B) {
        Serial.println("R");
      } else if (G < R && G < B) {
        Serial.println("G");
      } else if(B < R && B < G) {
        Serial.println("B");
      }
    }

  }
}
