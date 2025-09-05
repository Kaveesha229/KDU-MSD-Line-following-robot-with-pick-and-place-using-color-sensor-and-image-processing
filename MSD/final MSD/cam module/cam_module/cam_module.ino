#include <WiFi.h>
#include <HTTPClient.h>

// Replace with your network credentials
const char* ssid = "iPhone 15";         // Replace with your WiFi name
const char* password = "1234567890";    // Replace with your WiFi password

// IP address of the ESP32 receiver (check Serial output of ESP32 receiver)
const char* serverName = "http://172.20.10.3/sendData";

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
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(serverName);
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");

    String dataToSend = "Hello from ESP32-CAM";
    int httpResponseCode = http.POST("data=" + dataToSend);

    if (httpResponseCode > 0) {
      String response = http.getString();
      Serial.println("Server response: " + response);
    } else {
      Serial.print("Error sending POST: ");
      Serial.println(httpResponseCode);
    }

    http.end();
  } else {
    Serial.println("WiFi disconnected");
  }

  delay(5000); // Send every 5 seconds
}
