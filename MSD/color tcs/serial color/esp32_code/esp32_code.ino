#define RXD2 16  // ESP32 RX pin connected to Nano TX
#define TXD2 17  // ESP32 TX pin connected to Nano RX

void setup() {
  Serial.begin(115200);      // Monitor
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2); // Serial2 to talk to Nano
}

void loop() {
  // Send command to detect color
  Serial2.write('C');

  // Wait for response
  while (!Serial2.available()) {
    delay(10);
  }

  // Read response
  String color = Serial2.readStringUntil('\n');
  Serial.print("Detected Color: ");
  Serial.println(color);

  delay(2000); // Wait before next detection
}
