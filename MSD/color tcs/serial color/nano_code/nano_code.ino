#define S0 4
#define S1 5
#define S2 6
#define S3 7
#define sensorOut 8

int R = 0, G = 0, B = 0;

void setup() {
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);

  Serial.begin(9600); // Communication with ESP32
}

void loop() {
  if (Serial.available()) {
    char command = Serial.read();
    if (command == 'C') {  // Command to detect color

      // Read Red
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
        Serial.println(1);
      } else if (G < R && G < B) {
        Serial.println(2);
      } else if (B < R && B < G) {
        Serial.println(3);
      } else {
        Serial.println("Unknown");
      }
    }
  }
}
