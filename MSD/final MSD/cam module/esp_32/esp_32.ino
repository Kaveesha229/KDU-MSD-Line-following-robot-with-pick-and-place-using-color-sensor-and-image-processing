#include <WiFi.h>
#include <WebServer.h>

// Replace with your network credentials
const char* ssid = "iPhone 15";         // Replace with your WiFi name
const char* password = "1234567890";    // Replace with your WiFi password

WebServer server(80);

void handleReceive() {
  if (server.hasArg("data")) {
    String receivedData = server.arg("data");
    Serial.println("Data received: " + receivedData);
    server.send(200, "text/plain", "Data received");
  } else {
    server.send(400, "text/plain", "No data received");
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("Connected to WiFi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  server.on("/sendData", HTTP_POST, handleReceive);
  server.begin();
  Serial.println("Server started");
}

void loop() {
  server.handleClient();
}
