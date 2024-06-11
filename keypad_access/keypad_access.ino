#include <Arduino.h>
#include <Keypad.h>
#include <LiquidCrystal_I2C.h>
#include <esp_now.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
uint8_t broadcastAddress1[] = { 0x08, 0xD1, 0xF9, 0x6A, 0xF2, 0xE8 };

#define Buzzer 15
#define LED_Green 19
// #define LED_Blue 5
#define Magnetic 18
#define NOTE_C4 262
#define ECHO_PIN 5
#define TRIG_PIN 23

// const int relayCam = 4;
const int relaySolenoid = 4;

const byte ROWS = 4;
const byte COLS = 4;

typedef struct message_struct {
  char name[50];
  char role[10];
  char class_name[10];
} message_struct;

message_struct message;

// typedef struct message_struct {
//   char pin[7];
// } message_struct;

esp_now_peer_info_t peerInfo;

const char* ssid = "Servant";
const char* passwordWifi = "fgogamebaik";
const char* door_id = "1";
String ip = "34.101.35.208";

char hexaKeys[ROWS][COLS] = {
  { '1', '2', '3', 'A' },
  { '4', '5', '6', 'B' },
  { '7', '8', '9', 'C' },
  { '*', '0', '#', 'D' }
};

byte rowPins[ROWS] = { 13, 12, 14, 27 };
byte colPins[COLS] = { 26, 25, 33, 32 };

Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);
LiquidCrystal_I2C lcd(0x27, 16, 2);

const char* password = "1234";
char enteredPassword[7];
int passwordIndex = 0;
bool doorOpened = false;
unsigned long doorOpenTime = 0;
float X = 400 / 397.63;

void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  Serial.print("Packet to: ");
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
  Serial.print(" send status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void initWiFi() {
  WiFi.begin(ssid, passwordWifi);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
}

void getUserByPin(const char* pin) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    String url = "http://" + ip + ":8000/api/user/pin/" + String(pin);
    http.begin(url);
    int httpResponseCode = http.GET();

    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);

    if (httpResponseCode == 200) {
      String response = http.getString();
      Serial.println(response);

      DynamicJsonDocument doc(2048);
      DeserializationError error = deserializeJson(doc, response);

      if (!error) {
        if (doc["success"] == true) {
          JsonObject data = doc["data"];
          strncpy(message.name, data["name"].as<String>().c_str(), sizeof(message.name));
          strncpy(message.role, data["role"].as<String>().c_str(), sizeof(message.role));
        } else {
          Serial.println("Data pengguna tidak ditemukan atau ada kesalahan");
        }
      } else {
        Serial.print("Failed to parse JSON: ");
        Serial.println(error.c_str());
      }
    } else {
      Serial.println("Failed to fetch user info");
      strncpy(message.name, "Unknown", sizeof(message.name));
      strncpy(message.role, "Unknown", sizeof(message.role));
    }
    http.end();
  } else {
    Serial.println("WiFi not connected");
  }
}

void getDoor() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    String url = "http://" + pi + ":8000/api/door/" + String(door_id);
    http.begin(url);
    int httpResponseCode = http.GET();

    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);

    if (httpResponseCode == 200) {
      String response = http.getString();
      Serial.println(response);  // Print the response for debugging

      DynamicJsonDocument doc(2048);  // Increase the size if needed
      DeserializationError error = deserializeJson(doc, response);

      if (!error) {
        if (doc["success"] == true) {
          JsonObject data = doc["data"];
          strncpy(message.class_name, data["class_name"].as<String>().c_str(), sizeof(message.class_name));
        } else {
          Serial.println("Data door tidak ditemukan atau ada kesalahan");
        }
      } else {
        Serial.print("Failed to parse JSON: ");
        Serial.println(error.c_str());
      }
    } else {
      Serial.println("Failed to fetch door info");
      strncpy(message.class_name, "Unknown", sizeof(message.class_name));
    }
    http.end();  // End the HTTP connection
  } else {
    Serial.println("WiFi not connected");
  }
}


void sendMessage(const char* pin) {
  // message_struct message;
  getUserByPin(pin);
  delay(300);
  getDoor();
  delay(300);
  // strcpy(message.pin, pin);
  // Serial.print("Pin: ");
  // Serial.println(message.pin);
  Serial.print("name: ");
  Serial.println(message.name);
  Serial.print("role: ");
  Serial.println(message.role);
  Serial.print("class_name: ");
  Serial.println(message.class_name);
  delay(300);
  esp_err_t result = esp_now_send(0, (uint8_t*)&message, sizeof(message_struct));
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  } else {
    Serial.print("Error sending the data: ");
    Serial.println(result);
  }
}


bool checkAccess(const char* pin) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin("http://" + ip + ":8000/api/check-access");
    http.addHeader("Content-Type", "application/json");

    String payload = "{\"pin\":\"" + String(pin) + "\",\"door_id\":\"" + String(door_id) + "\"}";
    int httpResponseCode = http.POST(payload);

    if (httpResponseCode == 200) {
      String response = http.getString();
      Serial.println("Access granted");
      http.end();
      return true;
    } else {
      Serial.println("Access denied");
      http.end();
      return false;
    }
  } else {
    Serial.println("WiFi not connected");
    return false;
  }
}

float readUltrasonicDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  int duration = pulseIn(ECHO_PIN, HIGH, 30000);
  float distance = duration * 0.034 * X / 2;

  // Serial.print("Distance: ");
  // Serial.println(distance);
  delay(100);

  return distance;
}


/// Setup dan Loop



void setup() {
  lcd.init();
  lcd.backlight();
  lcd.home();
  pinMode(Buzzer, OUTPUT);
  // pinMode(relayCam, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(relaySolenoid, OUTPUT);
  pinMode(Buzzer, OUTPUT);
  pinMode(LED_Green, OUTPUT);
  // pinMode(LED_Blue, OUTPUT);
  pinMode(Magnetic, INPUT_PULLUP);

  Serial.begin(115200);
  initWiFi();

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  memcpy(peerInfo.peer_addr, broadcastAddress1, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Password:");
}

void loop() {
  char customKey = customKeypad.getKey();
  digitalWrite(LED_Green, HIGH);
  // digitalWrite(LED_Blue, HIGH);
  float distance = readUltrasonicDistance();

  if (distance < 4 && distance > 0) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Pintu Terbuka");
    digitalWrite(relaySolenoid, HIGH);
    delay(3000);  // Keep the door open for 3 seconds
    digitalWrite(relaySolenoid, LOW);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Password :");
  }

  if (customKey) {

    if (passwordIndex < 6) {
      lcd.setCursor(passwordIndex, 1);
      lcd.print('*');
      enteredPassword[passwordIndex] = customKey;
      passwordIndex++;
    }

    if (passwordIndex == 6) {
      bool accessGranted = checkAccess(enteredPassword);
      delay(300);
      sendMessage(enteredPassword);
      // bool accessGranted = false;
      enteredPassword[passwordIndex] = '\0';

      if (accessGranted) {
        if (digitalRead(Magnetic) == LOW) {
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Password Benar");
          digitalWrite(relaySolenoid, HIGH);
          tone(Buzzer, 265, 300);
          tone(Buzzer, 466, 200);
          tone(Buzzer, 600, 200);
          noTone(Buzzer);
          digitalWrite(LED_Green, LOW);
          delay(2000);
          lcd.clear();
          unsigned long startTime = millis();
          unsigned long elapsedTime = 0;
          unsigned long duration = 6000;
          for (int i = 0; i < 1;) {
            lcd.setCursor(0, 0);
            lcd.print("Menunggu Terbuka");
            lcd.setCursor(0, 1);
            lcd.print("Waktu: ");
            unsigned long currentTime = millis();
            elapsedTime = currentTime - startTime;
            lcd.print((duration - elapsedTime) / 1000);
            if (digitalRead(Magnetic) == HIGH) {
              i++;
            } else if (duration - elapsedTime < 1000) {
              i++;
            }
          }
          if (digitalRead(Magnetic) == HIGH) {
            lcd.clear();
            doorOpenTime = millis();
            duration = 6000;
            unsigned long elapsedTime = 0;
            for (int i = 0; i < 1;) {
              lcd.setCursor(0, 0);
              lcd.print("Pintu Terbuka");
              lcd.setCursor(0, 1);
              lcd.print("Tertutup: ");
              unsigned long currentTime = millis();
              elapsedTime = currentTime - doorOpenTime;
              lcd.print((duration - elapsedTime) / 1000);
              if (duration - elapsedTime < 1000) {
                i++;
              } else if (digitalRead(Magnetic) == LOW) {
                i++;
              }
            }

            if (millis() - doorOpenTime >= 5000) {
              if (digitalRead(Magnetic) == HIGH) {
                lcd.clear();
                for (int i = 0; i < 1;) {
                  lcd.setCursor(0, 0);
                  lcd.print("Tutup Pintunya!");
                  // digitalWrite(LED_Blue, LOW);
                  tone(Buzzer, 1535);
                  if (digitalRead(Magnetic) == LOW) {
                    noTone(Buzzer);

                    i++;
                  }
                }
              }
            }
          } else if (digitalRead(Magnetic) == LOW) {
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Pintu Terkunci!");
            digitalWrite(relaySolenoid, LOW);
            delay(2000);
          }
        } else if (digitalRead(Magnetic) == HIGH) {
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Pintu belum");
          lcd.setCursor(0, 1);
          lcd.print("Tertutup!");
          // digitalWrite(LED_Blue, LOW);
          delay(3000);
        }
      } else {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Password Salah");
        delay(2000);
      }

      digitalWrite(relaySolenoid, LOW);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Password :");
      passwordIndex = 0;
    }
  }
}
