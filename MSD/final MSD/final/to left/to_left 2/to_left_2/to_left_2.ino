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
int IR_weights[8] = { -30, -20, -10, -5, 5, 10, 20, 30};
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
void linefollowing(int baseSpeed);
void IR_value();
void toright();
void toleft();

const int OFFSET_MM = 30;

int b = 99;
int k;

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

  set_forward();
  delay(2000);
}

void loop() {

  if (b == 99) {
    Serial.println("Current section: b = 99");
    linefollowing(90);
    IRL_value = digitalRead(IRL);
    if (IRL_value == 1) {
      b = 9999;
      break_s();
      delay(750);
    }
  }

  if (b == 9999) {
    Serial.println("Current section: b = 10000");
    IRL_value = digitalRead(IRL);
    while (IRL_value == 1) {
      set_forward();
      IRL_value = digitalRead(IRL);
    }
    b = 10000;
  }

  if (b == 10000) {
    Serial.println("Current section: b = 10001");
    unsigned long startTime = millis();
    while (millis() - startTime < 1000) {
      linefollowing(90);
      b = 10002;
    }
  }

  if (b == 10002) {
    Serial.println("Current section: b = 10002");
    toleft();
    b = 101;
  }

  if (b == 101) {
Serial.println("Current section: b = 101");
    toleft();
    b = 102;
  }

  if (b == 102) {
    Serial.println("Current section: b = 102");
    toleft();
    b = 103;
  }

  if (b == 103) {
    toright();
    b = 104;
  }

  if (b == 104) {
    toleft();
    b = 105;
  }

  if (b == 105) {
    toleft();
    b = 106;
  }

  if (b == 106) {
    toleft();
    b = 200;
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
  LMOTORSpeed = MotorBasespeed + speedAdjust - 15;

  RMOTORSpeed = constrain(RMOTORSpeed, 0, MAX_SPEED);
  LMOTORSpeed = constrain(LMOTORSpeed, 0, MAX_SPEED);
}

void read_IR() {
  for (int i = 0; i < 8; i++) {
    int analogValue = analogRead(IR_PINS[i]);
    IR_VAL[i] = (analogValue > THRESHOLD) ? 1 : 0;
  }
}
/*
void RL() {
  IRR_value = digitalRead(IRR);
  IRL_value = digitalRead(IRL);

  if (IRL_value == 1) {
    break_s();
    delay(75);
    if (IRR_value == 1 && IRL_value == 1) {
      unsigned long startTime = millis();
      while (millis() - startTime < 350) {
        linefollowing(110);
      }
      b = 2;
    } else if (IRR_value == 0 && IRL_value == 1) {
      unsigned long startTime = millis();
      while (millis() - startTime < 350) {
        linefollowing(110);
      }
      break_s();
      delay(2000);
      turnleft();
    }
  } else if (IRR_value == 1) {
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
*/
void IR_value() {
  IRR_value = digitalRead(IRR);
  Serial.println(IRR_value);
  IRL_value = digitalRead(IRL);
  Serial.println(IRL_value);
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

void toleft() {
  int y = 1;
  while (y == 1) {
    Serial.println("Current section: to left");
    linefollowing(110);
    IRL_value = digitalRead(IRL);
    if (IRL_value == 1) {
      unsigned long startTime = millis();
      while (millis() - startTime < 350) {
        linefollowing(110);
      }
      turnleft();
      y = 0;
      Serial.println("Current section: toleft turn left");
    }
  }
}

void toright() {
  int y = 1;
  while (y == 1) {
    Serial.println("Current section: to right");
    linefollowing(110);
    IRR_value = digitalRead(IRR);
    if (IRR_value == 1) {
      unsigned long startTime = millis();
      while (millis() - startTime < 350) {
        linefollowing(110);
      }
      turnright();
      y = 0;
      Serial.println("Current section: toright, turn right");
    }
  }
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
  stop();  // Same as stop
}

void set_forward() {
  digitalWrite(RMotorB, LOW);
  digitalWrite(LMotorB, LOW);
  digitalWrite(RMotorA, turn_speed);
  digitalWrite(LMotorA, 10 + turn_speed);
}

void turn180() {
  digitalWrite(RMotorB, turn_speed);
  digitalWrite(LMotorB, LOW);
  digitalWrite(RMotorA, LOW);
  digitalWrite(LMotorA, turn_speed);
  delay(1100);
  set_forward();
  delay(50);
}
