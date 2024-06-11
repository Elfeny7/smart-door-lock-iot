#include "Arduino.h"
#include "WiFi.h"
#include "esp_camera.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "driver/rtc_io.h"
#include <LittleFS.h>
#include <FS.h>
#include <esp_now.h>
#include <HTTPClient.h>

#define FILE_PHOTO_PATH "/photo.jpg"

#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

typedef struct message_struct {
  char name[50];
  char role[10];
  char class_name[10];
} message_struct;

message_struct myMessage;

const char* ssid = "Servant";
const char* password = "fgogamebaik";
boolean takeNewPhoto = false;
String ip = "34.101.35.208";

void OnDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len) {
  memcpy(&myMessage, incomingData, sizeof(myMessage));
  Serial.println(myMessage.name);
  Serial.println(myMessage.role);
  Serial.println(myMessage.class_name);
  Serial.println("");
  takeNewPhoto = true;
}

void initWiFi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.print("[WIFI] Connected: ");
  Serial.println(WiFi.localIP());
}

void capturePhotoSaveLittleFS(void) {
  camera_fb_t* fb = NULL;
  for (int i = 0; i < 4; i++) {
    fb = esp_camera_fb_get();
    esp_camera_fb_return(fb);
    fb = NULL;
  }

  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    delay(1000);
    ESP.restart();
  }

  File file = LittleFS.open(FILE_PHOTO_PATH, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file in writing mode");
  } else {
    file.write(fb->buf, fb->len);
    Serial.print("The picture has been saved in ");
    Serial.print(FILE_PHOTO_PATH);
    Serial.print(" - Size: ");
    Serial.print(fb->len);
    Serial.println(" bytes");
  }
  file.close();
  esp_camera_fb_return(fb);
}

void initLittleFS() {
  if (!LittleFS.begin(true)) {
    Serial.println("An Error has occurred while mounting LittleFS");
    ESP.restart();
  } else {
    delay(500);
    Serial.println("LittleFS mounted successfully");
  }
}

void initCamera() {
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
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_LATEST;

  if (psramFound()) {
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 6;
    config.fb_count = 1;
  } else {
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 6;
    config.fb_count = 1;
  }
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    ESP.restart();
  }
}

void UploadToDataverse() {
  // Check WiFi connection status
  if (WiFi.status() == WL_CONNECTED) {
    WiFiClient client;

    const char* server = ip.c_str();  // Server URL
    const int port = 8000;

    if (!client.connect(server, port)) {
      Serial.println("Connection failed!");
      return;
    }
    Serial.println("Connected to the Dataverse.");

    // Generate boundary
    const char* bound = "boundary";

    // Prepare headers
    String head = "--" + String(bound) + "\r\n";
    head += "Content-Disposition: form-data; name=\"name\"\r\n\r\n" + String(myMessage.name) + "\r\n";
    head += "--" + String(bound) + "\r\n";
    head += "Content-Disposition: form-data; name=\"role\"\r\n\r\n" + String(myMessage.role) + "\r\n";
    head += "--" + String(bound) + "\r\n";
    head += "Content-Disposition: form-data; name=\"class_name\"\r\n\r\n" + String(myMessage.class_name) + "\r\n";
    head += "--" + String(bound) + "\r\n";
    head += "Content-Disposition: form-data; name=\"image\";filename=\"IMAGE.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";

    String tail = "\r\n--" + String(bound) + "--\r\n";

    // Read image file from LittleFS
    File file = LittleFS.open(FILE_PHOTO_PATH, FILE_READ);
    if (!file) {
      Serial.println("Failed to open file in reading mode");
      return;
    }

    // Calculate total content length
    uint32_t extraLen = head.length() + tail.length();
    uint32_t totalLen = file.size() + extraLen;

    // Prepare request headers
    String request = "POST /api/logs\r\n";
    request += "Host: " + String(server) + "\r\n";
    request += "Content-Length: " + String(totalLen) + "\r\n";
    request += "Content-Type: multipart/form-data; boundary=" + String(bound) + "\r\n\r\n";

    // Send request headers
    client.print(request);

    // Send headers
    client.print(head);

    // Send image data
    while (file.available()) {
      client.write(file.read());
    }

    // Send tail
    client.print(tail);

    // Wait for response
    String response;
    unsigned long timeout = millis();
    while (client.available() == 0) {
      if (millis() - timeout > 5000) {
        Serial.println(">>> Client Timeout !");
        client.stop();
        return;
      }
    }

    // Read response
    while (client.available()) {
      char c = client.read();
      response += c;
    }

    Serial.println("Server response: ");
    Serial.println(response);

    // Close file
    file.close();

    // Close connection
    client.stop();
  } else {
    Serial.println("WiFi Disconnected");
    // Reconnect to WiFi if disconnected
  }
}

void setup() {
  Serial.begin(115200);
  initWiFi();
  initLittleFS();
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  initCamera();

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  if (takeNewPhoto) {
    capturePhotoSaveLittleFS();
    UploadToDataverse();
    takeNewPhoto = false;
  }
  delay(300);
}
