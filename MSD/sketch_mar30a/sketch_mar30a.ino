#define THRESHOLD 3500
#define THRESHOLDR 850
#define THRESHOLDL 850

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

int MotorBasespeed = 100;

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

  Serial.println("Robot Initialized. Starting PID Control...");
  set_forward();
  delay(2000);
}

void loop() {
  read_IR();
  if (IR_VAL[0] == 0 && IR_VAL[1] == 0 && IR_VAL[2] == 0 && IR_VAL[3] == 0 && IR_VAL[4] == 0 && IR_VAL[5] == 0 && IR_VAL[6] == 0 && IR_VAL[7] == 0) {
    Serial.println("All IR sensors off-line. Stopping robot.");
    stop();
    while (1) {}
  }
  PID_control();
  set_speed();
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

  // Serial Monitor: Print PID outputs
  Serial.print("PID Adjust: ");
  Serial.print(speedAdjust);
  Serial.print(" | L Speed: ");
  Serial.print(LMOTORSpeed);
  Serial.print(" | R Speed: ");
  Serial.println(RMOTORSpeed);
}

void read_IR() {
  for (int i = 0; i < 8; i++) {
    int analogValue = analogRead(IR_PINS[i]);
    IR_VAL[i] = (analogValue > THRESHOLD) ? 1 : 0;
  }
}

void RL() {
   int IRR_ANA = analogRead(IRR);
   int IRL_ANA = analogRead(IRL);
   int IRR_value = (IRR_ANA > THRESHOLDR) ? 1 : 0;
   int IRL_value = (IRL_ANA > THRESHOLDR) ? 1 : 0;

   if (IRR_value == 1) {
       Serial.println("Right turn triggered!");
       turnright();
   }
   if (IRL_value == 1) {
       Serial.println("Left turn triggered!");
       turnleft();
   }
}

void set_speed() {
  analogWrite(LMotorPWM, LMOTORSpeed);
  analogWrite(RMotorPWM, RMOTORSpeed);
}

void turnright() {
  digitalWrite(RMotorA, LOW);
  digitalWrite(LMotorA, HIGH);
  digitalWrite(RMotorB, HIGH);
  digitalWrite(LMotorB, LOW);
  delay(500);
}

void turnleft() {
  set_forward();
  delay(630);
  digitalWrite(RMotorA, HIGH);
  digitalWrite(LMotorA, LOW);
  digitalWrite(RMotorB, LOW);
  digitalWrite(LMotorB, HIGH);
  delay(730);
  break_s();
  delay(200);
}

void stop() {
  digitalWrite(RMotorA, LOW);
  digitalWrite(LMotorA, LOW);
  digitalWrite(RMotorB, LOW);
  digitalWrite(LMotorB, LOW);
  Serial.println("Motors stopped.");
}

void break_s() {
  digitalWrite(RMotorA, LOW);
  digitalWrite(LMotorA, LOW);
  digitalWrite(RMotorB, LOW);
  digitalWrite(LMotorB, LOW);
  Serial.println("Emergency brake applied.");
}

void set_forward() {
  digitalWrite(RMotorB, LOW);
  digitalWrite(LMotorB, LOW);
  digitalWrite(RMotorA, HIGH);
  digitalWrite(LMotorA, HIGH);
  Serial.println("Moving forward.");
}
