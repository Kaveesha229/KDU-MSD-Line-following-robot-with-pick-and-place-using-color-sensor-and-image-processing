#define BLYNK_TEMPLATE_ID "TMPL6u1INiQJK"
#define BLYNK_TEMPLATE_NAME "testing"
#define BLYNK_AUTH_TOKEN  "A9WRT8HhAkMjEPaqgRhP1J1uALbEtV6G"

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

char auth[] = BLYNK_AUTH_TOKEN;
const char* ssid = "iPhone 15";
const char* password = "1234567890";

int start[3] = {0};   // Store start values
int finish[3] = {0};  // Store finish values

int startIndex = 0;
int finishIndex = 0;

int blynk_count = 0;  // Counter for total values received (max = 6)

// Append to start array in order
void addToStart(int value) {
  if (startIndex < 3) {
    start[startIndex++] = value;
    blynk_count++;
  }
}

// Append to finish array in order
void addToFinish(int value) {
  if (finishIndex < 3) {
    finish[finishIndex++] = value;
    blynk_count++;
  }
}

// Handle start buttons
BLYNK_WRITE(V10) {
  if (param.asInt() == 1) addToStart(11);
}

BLYNK_WRITE(V11) {
  if (param.asInt() == 1) addToStart(12);
}

BLYNK_WRITE(V12) {
  if (param.asInt() == 1) addToStart(13);
}

// Handle finish buttons
BLYNK_WRITE(V13) {
  if (param.asInt() == 1) addToFinish(21);
}

BLYNK_WRITE(V14) {
  if (param.asInt() == 1) addToFinish(22);
}

BLYNK_WRITE(V15) {
  if (param.asInt() == 1) addToFinish(23);
}

void setup() {
  Serial.begin(115200);
  Blynk.begin(auth, ssid, password);
}

void loop() {
  Blynk.run();

  Serial.print("Start: ");
  for (int i = 0; i < 3; i++) Serial.print(start[i]), Serial.print(" ");
  Serial.print(" | Finish: ");
  for (int i = 0; i < 3; i++) Serial.print(finish[i]), Serial.print(" ");
  Serial.print(" | blynk_count: ");
  Serial.println(blynk_count);

  // Check if all values are received
  if (blynk_count == 6) {
    Serial.println("✅ All 6 values received. Ready to proceed!");
    // You can now call your next function here
    // proceedToNextTask();  <-- Example
  }

  delay(1000);  // Debug only
}
