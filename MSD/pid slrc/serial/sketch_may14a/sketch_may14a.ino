#include <Wire.h>
#include <Adafruit_VL53L0X.h>

Adafruit_VL53L0X lox;

#define THRESHOLD 3500

#define IR1 14
#define IR2 27
#define IR3 26
#define IR4 25
#define IR5 33
#define IR6 32
#define IR7 35
#define IR8 34
#define IRL 4
#define IRR 2

#define LMotorA 19
#define LMotorB 21
#define LMotorPWM 22

#define RMotorA 5
#define RMotorB 18
#define RMotorPWM 23

#define MAX_SPEED 150
#define turn_speed 150
int MotorBasespeed = 110;

int IR_VAL[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int IR_weights[8] = {-40, -20, -10, -5, 5, 10, 20, 40};
int IR_PINS[8] = {IR1, IR2, IR3, IR4, IR5, IR6, IR7, IR8};

int RMOTORSpeed = 0;
int LMOTORSpeed = 0;
int speedAdjust = 0;

float P, I, D;
float error = 0;
float previousError = 0;
float Kp = 0.8;
float Kd = 0.2;
float Ki = 0;

const int OFFSET_MM = 30;
#define SDA_PIN 15
#define SCL_PIN 13

void setup() {
  Serial.begin(115200);
  
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  pinMode(IR4, INPUT);
  pinMode(IR5, INPUT);
  pinMode(IR6, INPUT);
  pinMode(IR7, INPUT);
  pinMode(IR8, INPUT);

  pinMode(RMotorA, OUTPUT);
  pinMode(RMotorB, OUTPUT);
  pinMode(RMotorPWM, OUTPUT);

  pinMode(LMotorA, OUTPUT);
  pinMode(LMotorB, OUTPUT);
  pinMode(LMotorPWM, OUTPUT);

  pinMode(IRR, INPUT);
  pinMode(IRL, INPUT);

  Wire.begin(SDA_PIN, SCL_PIN);
  if (!lox.begin()) {
    Serial.println("Failed to boot VL53L0X");
    while(1);
  }

  Serial.println("System Ready");
  set_forward();
  delay(2000);
}

void loop() {
  linefollowing();
  objectdetect();
  RL();
}

void PID_control() {
  error = 0;
  for (int i = 0; i < 8; i++) {
    error += IR_weights[i] * IR_VAL[i];
  }
  
  P = error;
  I = I + error;
  D = error - previousError;
  previousError = error;

  speedAdjust = (Kp * P + Ki * I + Kd * D);

  RMOTORSpeed = MotorBasespeed - speedAdjust;
  LMOTORSpeed = MotorBasespeed + speedAdjust;

  RMOTORSpeed = constrain(RMOTORSpeed, 0, MAX_SPEED);
  LMOTORSpeed = constrain(LMOTORSpeed, 0, MAX_SPEED);

  // Debug output (non-blocking)
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 500) {
    lastDebug = millis();
    Serial.print("PID| Error:"); Serial.print(error);
    Serial.print(" P:"); Serial.print(P);
    Serial.print(" I:"); Serial.print(I);
    Serial.print(" D:"); Serial.print(D);
    Serial.print(" L:"); Serial.print(LMOTORSpeed);
    Serial.print(" R:"); Serial.println(RMOTORSpeed);
  }
}

void read_IR() {
  for (int i = 0; i < 8; i++) {
    int analogValue = analogRead(IR_PINS[i]);
    IR_VAL[i] = (analogValue > THRESHOLD) ? 1 : 0;
  }

  // Debug output (non-blocking)
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 300) {
    lastDebug = millis();
    Serial.print("IR| ");
    for (int i = 0; i < 8; i++) {
      Serial.print(IR_VAL[i]); Serial.print(" ");
    }
    Serial.println();
  }
}

void RL() {
  int IRR_value = digitalRead(IRR);
  int IRL_value = digitalRead(IRL);

  if (IRL_value == 1) {
    Serial.println("Left turn detected!");
    break_s();
    delay(1000);
    if (IRR_value == 1 && IRL_value == 1) {
      Serial.println("T-intersection detected");
      unsigned long startTime = millis();  
      while (millis() - startTime < 530) { 
        linefollowing();
      }
      turnright();
    } else if (IRR_value == 0 && IRL_value == 1) {
      unsigned long startTime = millis();
      while (millis() - startTime < 530) { 
        linefollowing();
      }
      turnleft();
    }
  }
  else if (IRR_value == 1) {
    Serial.println("Right turn detected!");
    break_s();
    delay(1000);
    if (IRR_value == 1 && IRL_value == 1) {
      Serial.println("T-intersection detected");
      unsigned long startTime = millis();  
      while (millis() - startTime < 530) { 
        linefollowing();
      }
      turnright();
    } else if (IRR_value == 1 && IRL_value == 0) {
      unsigned long startTime = millis();
      while (millis() - startTime < 530) { 
        linefollowing();
      }
      turnright();
    }
  }
}

void objectdetect() {
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);

  if (measure.RangeStatus != 4) {
    int distance_obj = measure.RangeMilliMeter - OFFSET_MM;
    
    // Debug output (non-blocking)
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 500) {
      lastDebug = millis();
      Serial.print("TOF| Dist:"); Serial.print(distance_obj); Serial.println("mm");
    }
    
    if (distance_obj <= 110) {         
      Serial.println("Obstacle detected! Performing avoidance");
      break_s();
      delay(2000);
      turn180();
    }
  }
}

void set_speed() {
  analogWrite(LMotorPWM, LMOTORSpeed);
  analogWrite(RMotorPWM, RMOTORSpeed);
}

void linefollowing() {
  set_forward(); 
  read_IR();
  PID_control();
  set_speed();
}

void turnright() {
  Serial.println("Turning right");
  digitalWrite(RMotorA, LOW);
  digitalWrite(LMotorA, turn_speed);
  digitalWrite(RMotorB, turn_speed);
  digitalWrite(LMotorB, LOW);
  delay(820);
  stop();
  delay(200);
}

void turnleft() {
  Serial.println("Turning left");
  digitalWrite(RMotorA, turn_speed);
  digitalWrite(LMotorA, LOW);
  digitalWrite(RMotorB, LOW);
  digitalWrite(LMotorB, turn_speed);
  delay(780);
  stop();
  delay(200);
}

void stop() {
  digitalWrite(RMotorA, LOW);
  digitalWrite(LMotorA, LOW);
  digitalWrite(RMotorB, LOW);
  digitalWrite(LMotorB, LOW);
}

void break_s() {
  Serial.println("Emergency stop");
  digitalWrite(RMotorA, LOW);
  digitalWrite(LMotorA, LOW);
  digitalWrite(RMotorB, LOW);
  digitalWrite(LMotorB, LOW);
}

void set_forward() {
  digitalWrite(RMotorB, LOW);
  digitalWrite(LMotorB, LOW);
  digitalWrite(RMotorA, turn_speed);
  digitalWrite(LMotorA, turn_speed);
}

void turn180() { 
  Serial.println("Turning 180 degrees");
  digitalWrite(RMotorB, turn_speed);
  digitalWrite(LMotorB, LOW);
  digitalWrite(RMotorA, LOW);
  digitalWrite(LMotorA, turn_speed);
  delay(1750);
  set_forward();
  delay(50);
}
