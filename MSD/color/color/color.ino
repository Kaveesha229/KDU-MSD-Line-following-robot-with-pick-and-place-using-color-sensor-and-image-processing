#include "esp_camera.h"
#include "Arduino.h"

// ESP32-CAM (AI Thinker) PIN configuration
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

void setupCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_RGB565;     // For pixel access
  config.frame_size = FRAMESIZE_QQVGA;        // 160x120 - fits in memory
  config.jpeg_quality = 12;
  config.fb_count = 1;

  // Initialize the camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    while (true); // Stop execution
  } else {
    Serial.println("Camera init success");
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  setupCamera();
}

void loop() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    delay(1000);
    return;
  }

  int redCount = 0;
  int greenCount = 0;
  int blueCount = 0;

  // Loop through pixels in RGB565 format
  for (int i = 0; i < fb->len; i += 2) {
    uint8_t byte1 = fb->buf[i];
    uint8_t byte2 = fb->buf[i + 1];

    // Convert from RGB565 to RGB888
    uint8_t r = ((byte2 & 0xF8) >> 3) << 3;                  // 5-bit red
    uint8_t g = (((byte2 & 0x07) << 3) | ((byte1 & 0xE0) >> 5)) << 2;  // 6-bit green
    uint8_t b = (byte1 & 0x1F) << 3;                          // 5-bit blue

    // Simple thresholds
    if (r > 150 && g < 100 && b < 100) redCount++;
    else if (g > 150 && r < 100 && b < 100) greenCount++;
    else if (b > 150 && r < 100 && g < 100) blueCount++;
  }

  Serial.printf("Pixels: R=%d, G=%d, B=%d\n", redCount, greenCount, blueCount);

  if (redCount > greenCount && redCount > blueCount) {
    Serial.println("Detected Color: RED");
  } else if (greenCount > redCount && greenCount > blueCount) {
    Serial.println("Detected Color: GREEN");
  } else if (blueCount > redCount && blueCount > greenCount) {
    Serial.println("Detected Color: BLUE");
  } else {
    Serial.println("No dominant color detected");
  }

  esp_camera_fb_return(fb);
  delay(2000); // Delay before next capture
}
