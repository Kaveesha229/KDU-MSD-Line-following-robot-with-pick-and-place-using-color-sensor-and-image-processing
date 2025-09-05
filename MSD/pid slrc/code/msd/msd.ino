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

void PID_control();
void read_IR();
void set_speed();
void set_forward();
void stop();
void break_s();
void RL();
void turnright();
void turnleft();
void turn180();
void objectdetect();
void linefollowing();

const int OFFSET_MM = 30;
#define SDA_PIN 15
#define SCL_PIN 13

void setup() {
  Serial.begin(9600);
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

 /* Wire.begin(SDA_PIN, SCL_PIN);
  if (!lox.begin()) while(1); */

  set_forward();
  delay(2000);
}
//my loop;
void loop() {
  linefollowing();
  
  //objectdetect();
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
}

void read_IR() {
  for (int i = 0; i < 8; i++) {
    int analogValue = analogRead(IR_PINS[i]);
    IR_VAL[i] = (analogValue > THRESHOLD) ? 1 : 0;
  }
}

void RL() {
  int IRR_value = digitalRead(IRR);
  int IRL_value = digitalRead(IRL);

  if (IRL_value == 1) {
    break_s();
    delay(1000);
    if (IRR_value == 1 && IRL_value == 1) {  // Both sensors triggered
      unsigned long startTime = millis();  
      while (millis() - startTime < 350) { 
        linefollowing();
      }
      turnright();
    } else if (IRR_value == 0 && IRL_value == 1) {  // Only left sensor
      unsigned long startTime = millis();
      while (millis() - startTime < 350) { 
        linefollowing();
      }
      turnleft();
    }
  }
  
  // Right turn logic (NEW FIX)
  else if (IRR_value == 1) {  // Check right sensor separately
    break_s();
    delay(1000);
    if (IRR_value == 1 && IRL_value == 1) {  // Both sensors
      unsigned long startTime = millis();  
      while (millis() - startTime < 350) { 
        linefollowing();
      }
      turnright();
    } else if (IRR_value == 1 && IRL_value == 0) {  // Only right sensor
      unsigned long startTime = millis();
      while (millis() - startTime < 350) { 
        linefollowing();
      }
      turnright();
    }
  }
}
/*void objectdetect() {
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);

  if (measure.RangeStatus != 4) {
    int distance_obj = measure.RangeMilliMeter - OFFSET_MM;
    distance_obj = max(distance_obj, 0);
    
    if (distance_obj <= 110 && distance_obj > 105) {
      break_s();
      delay(2000);
      turn180();
    }
  }
}*/

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
  digitalWrite(RMotorA, LOW);
  digitalWrite(LMotorA, turn_speed);
  digitalWrite(RMotorB, turn_speed);
  digitalWrite(LMotorB, LOW);
  delay(470);
  stop();
  delay(200);
}

void turnleft() {
  digitalWrite(RMotorA, turn_speed);
  digitalWrite(LMotorA, LOW);
  digitalWrite(RMotorB, LOW);
  digitalWrite(LMotorB, turn_speed);
  delay(450);
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
  digitalWrite(RMotorA, LOW);
  digitalWrite(LMotorA, LOW);
  digitalWrite(RMotorB, LOW);
  digitalWrite(LMotorB, LOW);
}

void set_forward() {
  digitalWrite(RMotorB, LOW);
  digitalWrite(LMotorB, LOW);
  digitalWrite(RMotorA, turn_speed);
  digitalWrite(LMotorA, turn_speed
  );
}

void turn180() { 
  digitalWrite(RMotorB, turn_speed);
  digitalWrite(LMotorB, LOW);
  digitalWrite(RMotorA, LOW);
  digitalWrite(LMotorA, turn_speed);
  delay(1000);
  set_forward();
  delay(50);
}
