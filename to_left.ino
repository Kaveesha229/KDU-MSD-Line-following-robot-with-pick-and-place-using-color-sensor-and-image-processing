#define THRESHOLD 3800

#define BLYNK_TEMPLATE_ID "TMPL6u1INiQJK"
#define BLYNK_TEMPLATE_NAME "testing"
#define BLYNK_AUTH_TOKEN  "A9WRT8HhAkMjEPaqgRhP1J1uALbEtV6G"

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

char auth[] = BLYNK_AUTH_TOKEN;
const char* ssid = "iPhone 15";
const char* password = "1234567890";

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
int colors[3];
int colorsp[3];
int blynk[2] = {0, 0}; // RED = 1 , GREEN = 2 , BLUE = 3
int colorCount = 0;

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

int line = 90;
int whi = 450;

void PID_control();
void read_IR();
void set_speed();
void set_forward();
void set_backward();
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
void turn180right();
void kola_pata();
void nil_pata();
void do_colorsen();
void undo_colorsen();
void turn180left();
void nil_patap();
void kola_patap();
void place_2();
void place_3();
void place_4();
void place_5();
void sleft();
void sright();

const int OFFSET_MM = 30;

int b = 578;
int k;

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 16, 17);

  WiFi.begin(ssid, password);
  Blynk.begin(auth, ssid, password);

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

  if (b == 578) {
    Blynk.run();
    Serial.println(blynk[0]);
    Serial.println(blynk[1]);
    if (colorCount == 2) {
      b = 579;
    }
  }

  if (b == 579) {
    WiFi.mode(WIFI_OFF);
    break_s();
    delay(1000);
    b = 0;
  }

  if (b == 0) {
    unsigned long startTime = millis();
    while (millis() - startTime < 400) {
      linefollowing(line);
    }
    b = 1;
  }

  if (b == 1) {
    Serial2.println('1');
    delay(1000);
    Serial.println("Current section: b = 1");
    b = 2;
  }

  if (b == 2) {
    Serial.println("Current section: b = 2");
    toright();
    b = 3;
  }

  if (b == 3) {
    Serial.println("Current section: b = 3");
    toleft();
    b = 4;
  }

  if (b == 4) {
    Serial.println("Current section: b = 4");
    linefollowing(70);
    objectdetect();
    if (distance_obj == 1) {
      break_s();
      delay(1000);
      linefollowing(line);
      b = 5;
    }
  }
  if (b == 5) {
    Serial.println("Current section: b = 5");
    Serial.print(colors[0]);
    detect_color();
    colors[0] = color;
    Serial.print("Color is");
    Serial.println(colors[0]);
    Serial.print("blynk");
    Serial.println(blynk[1]);

    b = 6;
  }

  if (b == 6) {
    Serial.println("Current section: b = 6");

    if (colors[0] == blynk[1]) {

      place_2();

      b = 7;
    } else {
      b = 18;
    }
  }

  if (b == 7) {
    Serial.println("Current section: b = 7");
    turn180right();
    b = 7000;
  }

  if (b == 7000) {
    Serial.println("Current section: b = 7000");
    linefollowing(line);

    detect_color();
    colors[1] = color;
    Serial.print("Color is ");
    Serial.println(colors[1]);
    Serial.print("blynk ");
    Serial.println(blynk[0]);
    b = 8;
  }

  if (b == 8) {
    Serial.println("Current section: b = 8");
    if (colors[1] == blynk[0]) {
      b = 9;
    } else {
      b = 12;
    }
  }

  if (b == 9) {
    Serial.println("Current section: b = 9");

    place_3();

    b = 10;
  }

  if (b == 10) {
    Serial.println("Current section: b = 10");
    turn180();
    b = 1100;
  }

  if (b == 1100) {
    linefollowing(line);
    IRL_value = digitalRead(IRL);
    if (IRL_value == 1) {
      b = 1101;
    }
  }

  if ( b == 1101) {
    unsigned long startTime = millis();
    while (millis() - startTime < whi) {
      linefollowing(line);
    }
    b = 11;
  }

  if (b == 11) {
    b = 100;
  }

  if (b == 12) {
    turn180right();
    b = 1300;
  }

  if (b == 1300) {
    kola_pata();
  }

  if (b == 18) {
    Serial.println("Current section: b = 18");
    turn180right();
    b = 1800;
  }

  if (b == 1800) {
    detect_color();
    colors[1] = color;
    b = 19;
  }

  if (b == 19) {
    if (colors[1] == blynk[1]) {
      b = 20;
    } else {
      b = 28;
    }
  }

  if (b == 20) {

    place_2();

    b = 21;
  }

  if (b == 21) {
    if (colors[0] == blynk[0]) {
      b = 25;
    } else {
      b = 22;
    }
  }

  if (b == 22) {
    linefollowing(line);
    turn180right();
    b = 23;
  }

  if (b == 23) {
    kola_pata();
  }

  if (b == 25) {
    linefollowing(line);
    turn180();
    b = 2501;
  }

  if (b == 2501) {
    toleft();
    b = 2502;
  }

  if (b == 2502) {
    toleft();
    b = 2503;
  }

  if (b == 2503) {
    linefollowing(70);
    objectdetect();
    if (distance_obj == 1) {
      break_s();
      delay(1000);
      b = 26;
    }
  }

  if (b == 26) {
    nil_pata();
  }

  if (b == 28) {
    linefollowing(line);
    turn180right();
    b = 29;
  }

  if (b == 29) {

    place_2();

    b = 30;
  }

  if (b == 30) {
    linefollowing(line);
    turn180();
    b = 31;
  }

  if (b == 31) {
    toleft();
    b = 32;
  }

  if (b == 32) {
    if (colors[1] == blynk[0]) {
      b = 33;
    } else {
      b = 39;
    }
  }

  if (b == 33) {
    toleft();
    b = 34;
  }

  if (b == 34) {
    linefollowing(70);
    objectdetect();
    if (distance_obj == 1) {
      break_s();
      delay(1000);
      b = 35;
    }
  }
  if (b == 35) {

    place_3();

    b = 36;
  }

  if (b == 36) {
    linefollowing(line);
    turn180();
    b = 37;
  }

  if (b == 37) {
    linefollowing(line);
    IRL_value = digitalRead(IRL);
    if (IRL_value == 1) {
      b = 3700;
    }
  }

  if ( b == 3700) {
    unsigned long startTime = millis();
    while (millis() - startTime < whi) {
      linefollowing(line);
    }
    b = 38;
  }

  if (b == 38) {
    b = 100;
  }

  if (b == 39) {
    linefollowing(line);
    IRL_value = digitalRead(IRL);
    if (IRL_value == 1) {
      break_s();
      delay(300);
      b = 40;
    }
  }

  if (b == 40) {
    unsigned long startTime = millis();
    while (millis() - startTime < whi) {
      linefollowing(line);
    }
    b = 41;
  }

  if (b == 41) {
    toleft();
    b = 42;
  }

  if (b == 42) {
    linefollowing(70);
    objectdetect();
    if (distance_obj == 1) {
      break_s();
      delay(1000);
      b = 43;
    }
  }

  if (b == 43) {
    nil_pata();
  }

  if (b == 100) {
    break_s();
    delay (1000);
    b = 99;
  }

  if (b == 99) {
    Serial.println("Current section: b = 99");
    linefollowing(line);
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
    while (IRL_value == 0) {
      set_forward();
      IRL_value = digitalRead(IRL);
    }
    b = 10000;
  }

  if (b == 10000) {
    Serial.println("Current section: b = 10001");
    unsigned long startTime = millis();
    while (millis() - startTime < 2000) {
      linefollowing(line);
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
    break_s();
    delay(1000);
    b = 200;
  }

  if (b == 200) {
    toright();
    b = 201;
  }

  if (b == 201) {
    unsigned long startTime = millis();
    while (millis() - startTime < 700) {
      linefollowing(line);
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

    place_4();

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
    while (millis() - startTime < 700) {
      linefollowing(line);
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

    place_5();

    b = 213;
  }

  if (b == 213) {
    turn180();
    b = 214;
  }

  if (b == 214) {
    linefollowing(line);
    IRL_value = digitalRead(IRL);
    if (IRL_value == 1) {
      b = 215;
      break_s();
      delay(500);
    }
  }

  if (b == 215) {
    unsigned long startTime = millis();
    while (millis() - startTime < whi) {
      linefollowing(line);
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
    while (millis() - startTime < 700) {
      linefollowing(line);
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

    place_4();

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
    while (millis() - startTime < 700) {
      linefollowing(line);
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

    place_4();

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
    while (millis() - startTime < 700) {
      linefollowing(line);
      b = 242;
    }
    break_s();
    delay(1000);
  }

  if (b == 242) {

    place_5();

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
    while (millis() - startTime < whi) {
      linefollowing(line);
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
    while (millis() - startTime < whi) {
      linefollowing(line);
    }
    b = 250;
  }

  if (b == 250) {
    toright();
    b = 251;
  }

  if (b == 251) {
    unsigned long startTime = millis();
    while (millis() - startTime < 700) {
      linefollowing(line);
      b = 252;
    }
    break_s();
    delay(1000);
  }


  if (b == 252) {
    kola_patap();
  }

  if (b == 300) {
    break_s();
    delay(3000);
    b = 301;
  }

  if (b == 301) {
    toleft();
    b = 302;
  }

  if (b == 302) {
    toleft();
    b = 303;
  }

  if (b == 303) {
    linefollowing(line);
    IRL_value = digitalRead(IRL);
    if (IRL_value == 1) {
      b = 304;
    }
  }

  if (b == 304) {
    unsigned long startTime = millis();
    while (millis() - startTime < 550) {
      linefollowing(110);
    }
    b = 305;

  }

  if (b == 305) {
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
  LMOTORSpeed = MotorBasespeed + speedAdjust - 6.5;

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
      while (millis() - startTime < whi) {
        linefollowing(line);
      }
      b = 2;
    } else if (IRR_value == 0 && IRL_value == 1) {
      unsigned long startTime = millis();
      while (millis() - startTime < whi) {
        linefollowing(line);
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
      while (millis() - startTime < whi) {
        linefollowing(line);
      }
      b = 2;
    } else if (IRR_value == 1 && IRL_value == 0) {
      unsigned long startTime = millis();
      while (millis() - startTime < whi) {
        linefollowing(line);
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
  delay(230);
  stop();
  delay(200);
}

void undo_colorsen() {
  digitalWrite(RMotorA, LOW);
  digitalWrite(LMotorA, LOW);
  digitalWrite(RMotorB, turn_speed);
  digitalWrite(LMotorB, LOW);
  delay(230);
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

void turn180right() {
  Serial.println("turn180right");
  k = 1;
  if (k == 1) {
    linefollowing(line);
    turn180();
    k = 2;
  }
  if (k == 2 ) {
    toright();
    k = 3;
  }
  if (k == 3) {
    toright();
    k = 4;
  }

  if (k == 4) {
    int y = 1;
    while (y == 1) {
      Serial.println("vjjbhjbjh");
      linefollowing(70);
      objectdetect();
      if (distance_obj == 1) {
        break_s();
        delay(1000);
        y = 0;
      }
    }
  }
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
    while (millis() - startTime < 700) {
      linefollowing(line);
    }
    break_s();
    delay(1000);
  }
}

void kola_pata() {
  k = 1;
  if (k == 1) {

    place_3();

    k = 2;
  }

  if (k == 2) {
    linefollowing(line);
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
    b = 100;
  }
}

void nil_pata() {
  k = 1;
  if (k == 1) {

    place_3();

    k = 2;
  }

  if (k == 2) {
    linefollowing(line);
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
    b = 100;
  }
}

void nil_patap() {
  k = 1;
  if (k == 1) {

    place_5();

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

    place_5();

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

void place_2() {
  set_backward();
  delay(350);
  break_s();

  delay(1000);
  Serial2.println('2');
  delay(15000);

  set_forward();
  delay(550);
}

void place_3() {
  set_backward();
  delay(350);
  break_s();

  delay(1000);
  Serial2.println('3');
  delay(11000);

  set_forward();
  delay(550);

}

void place_4() {
  set_backward();
  delay(350);
  break_s();

  delay(1000);
  Serial2.println('4');
  delay(10000);

  set_forward();
  delay(350);
}

void place_5() {
  set_backward();
  delay(350);
  break_s();

  delay(1000);
  Serial2.println('5');
  delay(15000);

  set_forward();
  delay(350);
}

void toleft() {
  int y = 1;
  while (y == 1) {
    Serial.println("Current section: to left");
    linefollowing(line);
    IRL_value = digitalRead(IRL);
    if (IRL_value == 1) {
      unsigned long startTime = millis();
      while (millis() - startTime < 500) {
        linefollowing(line);
      }
      turnleft();
      y = 0;
      Serial.println("Current section: toright, turn left");
    }
  }
}

void toright() {
  int y = 1;
  while (y == 1) {
    Serial.println("Current section: to right");
    linefollowing(line);
    IRR_value = digitalRead(IRR);
    if (IRR_value == 1) {
      unsigned long startTime = millis();
      while (millis() - startTime < 500) {
        linefollowing(line);
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
  delay(710);
  stop();
  delay(200);
}

void turnleft() {
  digitalWrite(RMotorA, turn_speed);
  digitalWrite(LMotorA, LOW);
  digitalWrite(RMotorB, LOW);
  digitalWrite(LMotorB, turn_speed);
  delay(690);
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
  digitalWrite(RMotorA, 90);
  digitalWrite(LMotorA, 100);
}

void set_backward() {
  digitalWrite(RMotorB, 90);
  digitalWrite(LMotorB, 100);
  digitalWrite(RMotorA, LOW);
  digitalWrite(LMotorA, LOW);
}

void turn180() {
  digitalWrite(RMotorB, turn_speed);
  digitalWrite(LMotorB, LOW);
  digitalWrite(RMotorA, LOW);
  digitalWrite(LMotorA, turn_speed);
  delay(1500); // 1500
  set_forward();
  delay(50);
}

void sleft() {
  digitalWrite(RMotorA, LOW);
  digitalWrite(LMotorA, turn_speed);
  digitalWrite(RMotorB, LOW);
  digitalWrite(LMotorB, LOW);
  delay(200);
  stop();
}



bool isColorAlreadyStored(int code) {
  return (blynk[0] == code || blynk[1] == code);
}


void handleColorPress(int colorCode) {
  if (colorCount < 2 && !isColorAlreadyStored(colorCode)) {
    blynk[colorCount] = colorCode;
    colorCount++;
  }
}

// Red V1
BLYNK_WRITE(V1) {
  int value = param.asInt();
  if (value == 1) {
    handleColorPress(1);
  }
}

// Green V2
BLYNK_WRITE(V2) {
  int value = param.asInt();
  if (value == 1) {
    handleColorPress(2);
  }
}

// Blue  V0
BLYNK_WRITE(V0) {
  int value = param.asInt();
  if (value == 1) {
    handleColorPress(3);
  }
}
