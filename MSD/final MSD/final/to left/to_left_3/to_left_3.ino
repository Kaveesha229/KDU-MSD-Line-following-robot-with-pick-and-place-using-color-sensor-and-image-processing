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

int color;
int colorsp[3];
int blynk[2] = {1, 3}; // RED = 1 , GREEN = 2 , BLUE = 3

int RMOTORSpeed = 0;
int LMOTORSpeed = 0;
int speedAdjust = 0;

float P, I, D;
float error = 0;
float previousError = 0;
float Kp = 0.8;
float Kd = 0.2;
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
void toright();
void toleft();
void detect_color();
void do_colorsen();
void undo_colorsen();
void turn180left();
void nil_patap();
void kola_patap();

const int OFFSET_MM = 30;

int b = 199;
int k;

void setup() {
  Serial.begin(115200);
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
}

void loop() {

  if (b == 200) {
    toright();
    b = 201;
  }

  if (b == 201) {
    unsigned long startTime = millis();
    while (millis() - startTime < 850) {
      linefollowing(110);
      b = 202;
    }
    break_s();
    delay(1000);
  }

  if (b == 202) {
    detect_color();
    colorsp[0] = color;
    b = 203;
  }

  if (b == 203) {
    if (colorsp[0] == blynk[0]) {
      b = 205;
    }
    else {
      b = 219;
    }
  }

  if (b == 205) {
    delay(1000);
    Serial2.println('4');
    delay(7000);
    b = 206;
  }

  if (b == 206) {
    turn180();
    b = 207;
  }

  if (b == 207) {
    toleft();
    b = 208;
  }

  if (b == 208) {
    toleft();
    b = 209;
  }

  if (b == 209) {
    unsigned long startTime = millis();
    while (millis() - startTime < 850) {
      linefollowing(110);
      b = 210;
    }
    break_s();
    delay(1000);
  }

  if (b == 210) {
    detect_color();
    colorsp[1] = color;
    b = 211;
  }

  if (b == 211) {
    if (colorsp[1] == blynk[1]) {
      b = 212;
    }
    else {
      b = 217;
    }
  }

  if (b == 212) {
    delay(1000);
    Serial2.println('4');
    delay(17000);
    b = 213;
  }

  if (b == 213) {
    turn180();
    b = 214;
  }

  if (b == 214) {
    linefollowing(110);
    IRL_value = digitalRead(IRL);
    if (IRL_value == 1) {
      b = 215;
      break_s();
      delay(500);
    }
  }

  if (b == 215) {
    unsigned long startTime = millis();
    while (millis() - startTime < 350) {
      linefollowing(110);
      b = 216;
    }
  }

  if (b == 216) {
    b = 300;
  }

  if (b == 217) {
    turn180left();
    b = 218;
  }

  if (b == 218) {
    nil_patap();
  }

  if (b == 219) {
    turn180();
    b = 220;
  }

  if (b == 220) {
    toleft();
    b = 221;
  }

  if (b == 221) {
    toleft();
    b = 222;
  }

  if (b == 222) {
    unsigned long startTime = millis();
    while (millis() - startTime < 850) {
      linefollowing(110);
      b = 223;
    }
    break_s();
    delay(1000);
  }

  if (b == 223) {
    detect_color();
    colorsp[1] = color;
    b = 224;
  }

  if (b == 224) {
    if (colorsp[1] == blynk[0]) {
      b = 225;
    } else {
      b = 235;
    }
  }

  if (b == 225) {
    delay(1000);
    Serial2.println('4');
    delay(17000);
    b = 226;
  }

  if (b == 226) {
    if (colorsp[0] == blynk[1]) {
      b = 227;
    } else {
      b = 232;
    }
  }

  if (b == 227) {
    turn180();
    b = 228;
  }

  if (b == 228) {
    toright();
    b = 229;
  }

  if (b == 229) {
    toright();
    b = 230;
  }

  if (b == 230) {
    unsigned long startTime = millis();
    while (millis() - startTime < 850) {
      linefollowing(110);
      b = 231;
    }
    break_s();
    delay(1000);
  }

  if (b == 231) {
    kola_patap();
  }

  if (b == 232) {
    turn180left();
    b = 233;
  }

  if (b == 233) {
    delay(1000);
    Serial2.println('5');
    delay(17000);
    b = 234;
  }

  if (b == 234) {
    nil_patap();
  }

  if (b == 235 ) {
    turn180left();
    b = 236;
  }

  if (b == 236) {
    delay(1000);
    Serial2.println('4');
    delay(10000);
    b = 237;
  }

  if (b == 237) {
    turn180();
    b = 238;
  }

  if (b == 238) {
    toright();
    b = 239;
  }

  if (b == 239) {
    if (colorsp[1] == blynk[1]) {
      b = 240;
    } else {
      b = 248;
    }
  }

  if (b == 240) {
    toright();
    b = 241;
  }

  if (b == 241) {
    unsigned long startTime = millis();
    while (millis() - startTime < 850) {
      linefollowing(110);
      b = 242;
    }
    break_s();
    delay(1000);
  }

  if (b == 242) {
    delay(1000);
    Serial2.println('5');
    delay(17000);
    b = 243;
  }

  if (b == 243) {
    turn180();
    b = 244;
  }

  if (b == 244) {
    linefollowing(110);
    IRL_value = digitalRead(IRL);
    if (IRL_value == 1) {
      b = 245;
    }
  }

  if (b == 245) {
    unsigned long startTime = millis();
    while (millis() - startTime < 350) {
      linefollowing(110);
    }
    b = 246;
  }

  if (b == 246) {
    b = 300;
  }

  if (b == 248) {
    linefollowing(110);
    IRL_value = digitalRead(IRL);
    if (IRL_value == 1) {
      break_s();
      delay(300);
      b = 249;
    }
  }

  if (b == 249) {
    unsigned long startTime = millis();
    while (millis() - startTime < 350) {
      linefollowing(110);
    }
    b = 250;
  }

  if (b == 250) {
    toright();
    b = 251;
  }

  if (b == 251) {
    unsigned long startTime = millis();
    while (millis() - startTime < 850) {
      linefollowing(110);
      b = 252;
    }
    break_s();
    delay(1000);
  }


  if (b == 252) {
    kola_patap();
    b = 254;
  }

  if (b == 300){
    break_s();
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

void detect_color() {
  do_colorsen();
  delay (1000);
  Serial2.write('6');

  // Wait for response with timeout
  unsigned long start = millis();
  while (!Serial2.available() && (millis() - start < 1000)) {
    delay(10);
  }

  if (Serial2.available()) {
    color = 0;
    Serial.println(color);
    color = Serial2.read() - '0';  // Use parseInt to read the number
    Serial.print("Detected Color: ");
    Serial.println(color);
  }
  delay(500);
  undo_colorsen();
  delay(1000);
}

void do_colorsen() {
  digitalWrite(RMotorA, turn_speed);
  digitalWrite(LMotorA, LOW);
  digitalWrite(RMotorB, LOW);
  digitalWrite(LMotorB, LOW);
  delay(170);
  stop();
  delay(200);
}

void undo_colorsen() {
  digitalWrite(RMotorA, LOW);
  digitalWrite(LMotorA, LOW);
  digitalWrite(RMotorB, turn_speed);
  digitalWrite(LMotorB, LOW);
  delay(170);
  stop();
  delay(200);
}

void IR_value() {
  IRR_value = digitalRead(IRR);
  Serial.println(IRR_value);
  IRL_value = digitalRead(IRL);
  Serial.println(IRL_value);
}

void objectdetect() {
  int irf_val = digitalRead(IRF);
  if (irf_val == HIGH) {
    distance_obj = 0;
  } else {
    distance_obj = 1;
  }
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

void turn180left() {
  Serial.println("turn180right");
  k = 1;
  if (k == 1) {
    turn180();
    k = 2;
  }

  if (k == 2 ) {
    toleft();
    k = 3;
  }
  if (k == 3) {
    toleft();
    k = 4;
  }

  if (k == 4) {
    unsigned long startTime = millis();
    while (millis() - startTime < 850) {
      linefollowing(110);
    }
    break_s();
    delay(1000);
  }
}

void nil_patap() {
  k = 1;
  if (k == 1) {
    delay(1000);
    Serial2.println('5');
    delay(17000);
    k = 2;
  }

  if (k == 2) {
    turn180();
    k = 3;
  }

  if (k == 3) {
    toright();
    k = 4;
  }

  if (k == 4) {
    toleft();
    k = 5;
  }

  if (k == 5) {
    b = 300;
  }
}

void kola_patap() {
  k = 1;
  if (k == 1) {
    delay(1000);
    Serial2.println('5');
    delay(17000);
    k = 2;
  }

  if (k == 2) {
    turn180();
    k = 3;
  }

  if (k == 3) {
    toleft();
    k = 4;
  }

  if (k == 4) {
    toright();
    k = 5;
  }

  if (k == 5) {
    b = 300;
  }
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
  digitalWrite(LMotorA, turn_speed);
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
