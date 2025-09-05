#include <Wire.h>
#include <Adafruit_VL53L0X.h>

Adafruit_VL53L0X lox;
const int OFFSET_MM = 30; // Calibration offset

// Custom I2C pins (change if needed)
#define SDA_PIN 15
#define SCL_PIN 13

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN); // Explicit pin assignment
  if (!lox.begin()) while(1);   // Stop if sensor fails
}

void loop() {
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  
  if (measure.RangeStatus != 4) {
    int distance_obj = measure.RangeMilliMeter - OFFSET_MM;
    if (distance_obj < 0) distance_obj = 0;

    if (distance_obj > 100) {
      Serial.print("Corrected Distance (mm): ");
      Serial.println(distance_obj);

      // You can place your further logic here
      // e.g., if (distance_obj > 150) { ... }
    }
  }

  delay(100);
}
