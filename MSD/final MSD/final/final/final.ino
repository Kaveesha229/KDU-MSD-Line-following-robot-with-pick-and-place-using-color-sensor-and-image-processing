#include <Wire.h>
//#include <Adafruit_VL53L0X.h>

//Adafruit_VL53L0X lox;

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
#define IRF 13

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
float Kp = 0.5;
float Kd = 0.1;
float Ki = 0;

int IRR_value;
int IRL_value;
int distance_obj;

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
void linefollowing(int baseSpeed);
void IR_value();

const int OFFSET_MM = 30;
//#define SDA_PIN 15
//#define SCL_PIN 13

int b = 1;

void setup() {
  Serial.begin(9600);
  Serial2.begin(9600, SERIAL_8N1, 16, 17);

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

  pinMode(IRF, INPUT);

  set_forward();
  delay(2000); 

  //Wire.begin(SDA_PIN, SCL_PIN);
  //if (!lox.begin()) while(1); 
}

void loop() {
  if (b == 1) {
    Serial2.println(1);
    Serial.println("Current section: b = 1");
    linefollowing(110);
    int IRR_value = digitalRead(IRR);
    if (IRR_value == 1)
      b = 2;
  }

  if (b == 2) {
    Serial.println("Current section: b = 2");
    unsigned long startTime = millis();  
    while (millis() - startTime < 350) { 
      linefollowing(110);
    }
    turnright();
    delay(2000);
    b = 3;
  }
  
  if (b == 3) {
    linefollowing(100);
    int IRL_value = digitalRead(IRL);
    if (IRL_value == 1) {
      unsigned long startTime = millis();  
      while (millis() - startTime < 350) { 
        linefollowing(110);
      }
      turnleft();
      break_s();
      delay(200);
      b = 4;
    }
  }

   if (b == 4) {
    Serial.println("Current section: b = 4");
    linefollowing(70);
    objectdetect();
    Serial.print("Corrected Distance (mm): ");
    Serial.println(distance_obj);
    if (distance_obj == 1) {
      break_s();
      delay(2000);
      
      Serial2.println(2);     

      delay(10000);
      linefollowing(110);
      turn180();
      b = 5;
    }    
  }
  if (b == 5) {
    Serial.println("Current section: b = 5");
    linefollowing(110);
    int IRR_value = digitalRead(IRR);
    if (IRR_value == 1) {
      unsigned long startTime = millis();  
      while (millis() - startTime < 350) { 
        linefollowing(110);
      }
      turnright();
      b = 6;
    }
  }

  if (b == 6) {
    Serial.println("Current section: b = 6");
    linefollowing(110);
    int IRR_value = digitalRead(IRR);
    if (IRR_value == 1) {
      unsigned long startTime = millis();  
      while (millis() - startTime < 350) { 
        linefollowing(110);
      }
      turnright();
      b = 7;
    }
  }

  if (b == 7) {
    Serial.println("Current section: b = 7");
    linefollowing(110);
    objectdetect();
     if (distance_obj == 1) {
      break_s();
      delay(2000);
      
      Serial2.println(3);     

      delay(10000);
      linefollowing(110);
      turn180();
      b = 8;
    }
  }
//according to the data 
  if (b == 8) {
    Serial.println("Current section: b = 8");
    linefollowing(110);
    int IRL_value = digitalRead(IRL);
    if (IRL_value == 1) {
      unsigned long startTime = millis();  
      while (millis() - startTime < 350) { 
        linefollowing(110);
      }
      turnleft();
      b = 9;
    }
  }

  if (b == 9) {
    Serial.println("Current section: b = 9");
    linefollowing(110);
    int IRR_value = digitalRead(IRR);
    if (IRR_value == 1) {
      unsigned long startTime = millis();  
      while (millis() - startTime < 350) { 
        linefollowing(110);
      }
      turnright();
      b = 10; 
    }
  }
//upto here

  if (b == 10) {
    Serial.println("Current section: b = 10");
    linefollowing(110);
    RL();
  }
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
    delay(75);
    if (IRR_value == 1 && IRL_value == 1) {
      unsigned long startTime = millis();  
      while (millis() - startTime < 350) { 
        linefollowing(110);
      }
      b = 2;
    }
    else if (IRR_value == 0 && IRL_value == 1) {
      unsigned long startTime = millis();
      while (millis() - startTime < 350) { 
        linefollowing(110);
      }
      break_s();
      delay(2000);
      turnleft();
    }
  }
  else if (IRR_value == 1) {
    break_s();
    delay(75);
    if (IRR_value == 1 && IRL_value == 1) {
      unsigned long startTime = millis();  
      while (millis() - startTime < 350) { 
        linefollowing(110);
      }
      b = 2;
    } else if (IRR_value == 1 && IRL_value == 0) {
      unsigned long startTime = millis();
      while (millis() - startTime < 350) { 
        linefollowing(110);
      }
      break_s();
      delay(2000);
      turnright();
    }
  }
}

void IR_value() {
  int IRR_value = digitalRead(IRR);
  Serial.println(IRR_value);
  int IRL_value = digitalRead(IRL);
  Serial.println(IRL_value);
}

void objectdetect() {
  int irf_val = digitalRead(IRF);
  if (irf_val == HIGH) {
    distance_obj = 0;
  } else {
    distance_obj = 1;
  }

  /*VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  
  if (measure.RangeStatus != 4) {
    distance_obj = measure.RangeMilliMeter - OFFSET_MM;
    if (distance_obj < 0) distance_obj = 0;
  }*/
}


void set_speed() {
  analogWrite(LMotorPWM, LMOTORSpeed);
  analogWrite(RMotorPWM, RMOTORSpeed);
}

void linefollowing(int baseSpeed) {
  MotorBasespeed = baseSpeed;
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
  delay(500);
  stop();
  delay(200);
}

void turnleft() {
  digitalWrite(RMotorA, turn_speed);
  digitalWrite(LMotorA, LOW);
  digitalWrite(RMotorB, LOW);
  digitalWrite(LMotorB, turn_speed);
  delay(430);
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
  digitalWrite(LMotorA, turn_speed);
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
