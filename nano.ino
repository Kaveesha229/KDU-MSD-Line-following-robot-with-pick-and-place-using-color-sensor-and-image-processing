#include <Servo.h>

#define S0 4
#define S1 5
#define S2 6
#define S3 7
#define sensorOut 8

Servo servo1;  // First servo
Servo servo2;  // Second servo

int R = 0, G = 0, B = 0;

int currentServo1 = 160;
int currentServo2 = 40;
const int stepDelay = 20;

void moveServoSlowly(Servo &servo, int &currentAngle, int targetAngle) {
  if (currentAngle < targetAngle) {
    for (; currentAngle <= targetAngle; currentAngle++) {
      servo.write(currentAngle);
      delay(stepDelay);
    }
  } else {
    for (; currentAngle >= targetAngle; currentAngle--) {
      servo.write(currentAngle);
      delay(stepDelay);
    }
  }
}

void setup() {
  Serial.begin(9600);

  servo1.attach(10);
  servo2.attach(9);
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
    char command = Serial.read();

    if (command == '1') {
      moveServoSlowly(servo1, currentServo1, 160);
      moveServoSlowly(servo2, currentServo2, 40);
    }

    if (command == '2') {
      delay(1000);
      moveServoSlowly(servo1, currentServo1, 150);
      moveServoSlowly(servo2, currentServo2, 180);
      moveServoSlowly(servo1, currentServo1, 180);
      moveServoSlowly(servo2, currentServo2, 15);
      moveServoSlowly(servo1, currentServo1, 150);
      moveServoSlowly(servo2, currentServo2, 40);
      moveServoSlowly(servo1, currentServo1, 160);
      delay(5000);
    }

    if (command == '3') {
      delay(500);
      moveServoSlowly(servo1, currentServo1, 150);
      moveServoSlowly(servo2, currentServo2, 180);
      moveServoSlowly(servo1, currentServo1, 170);
      moveServoSlowly(servo2, currentServo2, 40);
    }

    if (command == '4') {
      delay(500);
      moveServoSlowly(servo2, currentServo2, 180);
      moveServoSlowly(servo1, currentServo1, 150);
      moveServoSlowly(servo2, currentServo2, 40);
      moveServoSlowly(servo1, currentServo1, 160);
    }

    if (command == '5') {
      delay(1000);
      moveServoSlowly(servo1, currentServo1, 150);
      moveServoSlowly(servo2, currentServo2, 15);
      moveServoSlowly(servo1, currentServo1, 180);
      moveServoSlowly(servo2, currentServo2, 180);
      moveServoSlowly(servo1, currentServo1, 150);
      moveServoSlowly(servo2, currentServo2, 40);
      moveServoSlowly(servo1, currentServo1, 160);
      delay(5000);
    }

    if (command == '6') {
       Serial.flush();
      digitalWrite(S2, LOW); digitalWrite(S3, LOW);
      R = pulseIn(sensorOut, LOW);
      delay(100);

      digitalWrite(S2, HIGH); digitalWrite(S3, HIGH);
      G = pulseIn(sensorOut, LOW);
      delay(100);

      digitalWrite(S2, LOW); digitalWrite(S3, HIGH);
      B = pulseIn(sensorOut, LOW);
      delay(100);

      // Send color result to ESP32
      // Send color CODE to ESP32
      if (R < G && R < B) {
        Serial.print('1');  // Red code
      } else if (G < R && G < B) {
        Serial.print('2');  // Green code
      } else if (B < R && B < G) {
        Serial.print('3');  // Blue code
      }
    }
  }
}
