int lastIRState = LOW;

void setup() {
  Serial.begin(9600);       
  Serial2.begin(9600, SERIAL_8N1, 16, 17);
  pinMode(4, INPUT);        
}

void loop() {
  int currentIRState = digitalRead(4);

  if (currentIRState != lastIRState) {
    lastIRState = currentIRState;

    if (currentIRState == HIGH) {
      Serial2.println(1);     
      Serial.println("IR HIGH - Sent: 1");
    } else {
      Serial2.println(2);     
      Serial.println("IR LOW - Sent: 2");
    }

    delay(100);  // Small debounce delay
  }
}
