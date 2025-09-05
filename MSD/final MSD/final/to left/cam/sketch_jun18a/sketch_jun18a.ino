#include "esp_camera.h"
#include <WiFi.h>
#include "model_data.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"

const char* ssid = "iPhone 15";
const char* password = "1234567890";
// Output to main ESP32
const char* host = "192.168.1.50"; // IP of main ESP32 board
const int port = 80;

#define BUTTON_PIN 7  // GPIO pin signal from main ESP32

// TensorFlow setup
constexpr int tensor_arena_size = 60 * 1024;
uint8_t tensor_arena[tensor_arena_size];

tflite::MicroInterpreter* interpreter;
TfLiteTensor* input = nullptr;
TfLiteTensor* output = nullptr;

const tflite::Model* model = nullptr;
tflite::MicroErrorReporter error_reporter;
tflite::AllOpsResolver resolver;

// Camera config
camera_config_t camera_config = {
  .pin_pwdn       = 32,
  .pin_reset      = -1,
  .pin_xclk       = 0,
  .pin_sscb_sda   = 26,
  .pin_sscb_scl   = 27,
  .pin_d7         = 35,
  .pin_d6         = 34,
  .pin_d5         = 39,
  .pin_d4         = 36,
  .pin_d3         = 21,
  .pin_d2         = 19,
  .pin_d1         = 18,
  .pin_d0         = 5,
  .pin_vsync      = 25,
  .pin_href       = 23,
  .pin_pclk       = 22,
  .xclk_freq_hz   = 20000000,
  .ledc_timer     = LEDC_TIMER_0,
  .ledc_channel   = LEDC_CHANNEL_0,
  .pixel_format   = PIXFORMAT_GRAYSCALE,
  .frame_size     = FRAMESIZE_96X96,
  .jpeg_quality   = 12,
  .fb_count       = 1
};

void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.println("WiFi connected");

  // Init camera
  esp_err_t err = esp_camera_init(&camera_config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: %s", esp_err_to_name(err));
    return;
  }

  // Load TFLite model
  model = tflite::GetModel(model_data);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    Serial.println("Model schema mismatch");
    return;
  }

  static tflite::MicroInterpreter static_interpreter(model, resolver, tensor_arena, tensor_arena_size, &error_reporter);
  interpreter = &static_interpreter;

  interpreter->AllocateTensors();
  input = interpreter->input(0);
  output = interpreter->output(0);

  Serial.println("System ready");
}

void loop() {
  if (digitalRead(BUTTON_PIN) == HIGH) {
    Serial.println("Trigger received");
    classify_and_send();
    delay(2000); // debounce delay
  }
}

void classify_and_send() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }

  // Resize and copy image to input tensor
  for (int i = 0; i < fb->len && i < input->bytes; i++) {
    input->data.uint8[i] = fb->buf[i];
  }

  esp_camera_fb_return(fb);

  interpreter->Invoke();

  // Get classification result
  int predicted = -1;
  float max_score = 0.0;

  for (int i = 0; i < output->dims->data[1]; i++) {
    float score = output->data.f[i];
    if (score > max_score) {
      max_score = score;
      predicted = i;
    }
  }

  String result;
  switch (predicted) {
    case 0: result = "sphere"; break;
    case 1: result = "cube"; break;
    case 2: result = "cylinder"; break;
    default: result = "unknown"; break;
  }

  Serial.println("Detected: " + result);
  send_result(result);
}

void send_result(String data) {
  WiFiClient client;
  if (!client.connect(host, port)) {
    Serial.println("Connection failed");
    return;
  }

  client.print("Detected=" + data);
  client.stop();
}
