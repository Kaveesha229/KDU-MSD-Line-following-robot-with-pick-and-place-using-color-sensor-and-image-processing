#include <HardwareSerial.h>

HardwareSerial SerialPort(2); // UART2 on GPIO16 (RX2), GPIO17 (TX2)

void setup() {
  Serial.begin(115200); // USB serial for debugging
  Serial2.begin(115200, SERIAL_8N1, 16, 17); // Initialize UART2 with RX=16, TX=17
  
  Serial.println("ESP32-CAM Sender Ready");
}

void loop() {
  // Send data to ESP32 board over UART2
  Serial2.println("Hello from ESP32-CAM!");
  delay(1000);
}
