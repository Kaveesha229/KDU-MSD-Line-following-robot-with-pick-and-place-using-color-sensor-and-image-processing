#define S0 4
#define S1 5
#define S2 6
#define S3 7
#define sensorOut 8

int R = 0;
int G = 0;
int B = 0;

void setup() {
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);

  Serial.begin(9600);
}

void loop() {
  // Read Red
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  R = pulseIn(sensorOut, LOW);
  delay(100);

  // Read Green
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  G = pulseIn(sensorOut, LOW);
  delay(100);

  // Read Blue
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  B = pulseIn(sensorOut, LOW);
  delay(100);

  // Determine the color
  if (R < G && R < B) {
    Serial.print("R");
  } else if (G < R && G < B) {
    Serial.print("G");
  } else if (B < R && B < G) {
    Serial.print("B");
  }

  delay(500);  
}
