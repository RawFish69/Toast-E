/* 
Toaste-E - Thermal Bot Main Board
Author: RawFish69

 This code is for the main board of the Thermal Bot, which receives commands from a controller
 and sends thermal data to the controller. It uses ESP-NOW for communication and an AMG8833 sensor
 for thermal imaging.

 The code initializes the motor driver, sets up ESP-NOW communication, and handles incoming commands
 to control the motors. It also reads thermal data from the AMG8833 sensor and sends it to the controller
 at a specified interval.

 The code is designed to be used with an ESP32 microcontroller and requires the Adafruit AMG88xx library.

 The controller requires a separate code to send commands to the main board and receive thermal data, 
 check controller.ino for details.
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_AMG88xx.h>
#include <WiFi.h>
#include <esp_now.h>

// Motor driver pin definitions for TB6612FNG
#define AIN1 3   // Motor A direction pin 1
#define AIN2 2   // Motor A direction pin 2
#define PWMA 7   // Motor A PWM pin
#define BIN1 4   // Motor B direction pin 1
#define BIN2 5   // Motor B direction pin 2
#define PWMB 6   // Motor B PWM pin
#define SDA 10   // I2C SDA pin
#define SCL 9   // I2C SCL pin

// LEDC PWM settings
#define LEDC_RESOLUTION_BITS 10
#define LEDC_RESOLUTION      ((1 << LEDC_RESOLUTION_BITS) - 1)
#define LEDC_FREQUENCY       50
#define CAMERA_SEND_FREQUENCY 5  // times per second

Adafruit_AMG88xx amg;
#define ROWS 8
#define COLS 8
float pixels[ROWS * COLS];

// globals for received commands
int cmdSpeed = 0;
int cmdTurn = 0;
bool cmdForward = true;

// timing for camera data send
unsigned long lastCameraSendTime = 0;
unsigned long cameraSendInterval = 1000 / CAMERA_SEND_FREQUENCY;
// Controller MAC address (hard-coded)
uint8_t controllerMAC[6] = {0xE4, 0xB3, 0x23, 0xD3, 0x66, 0xB8};  // Controller's STA MAC
bool useHardcodedMAC = true;  // Flag to use the hard-coded MAC or captured one

// Debug print interval
#define DEBUG_PRINT_INTERVAL 1000
unsigned long lastDebugPrintTime = 0;

// callback for ESP-NOW data reception
void onDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
  unsigned long currentMillis = millis();
  bool shouldPrint = currentMillis - lastDebugPrintTime >= DEBUG_PRINT_INTERVAL;
  
  // Only print debug info once per second
  if (shouldPrint) {
    Serial.print("Command received from ");
    for (int i = 0; i < 6; i++) {
      Serial.print(recv_info->src_addr[i], HEX);
      if (i < 5) Serial.print(":");
    }
    Serial.print(" len="); Serial.println(len);
    lastDebugPrintTime = currentMillis;
  }

  // Store sender MAC for future responses (but only if not using hard-coded)
  if (!useHardcodedMAC) {
    memcpy(controllerMAC, recv_info->src_addr, 6);
  }

  // Parse joystick command string
  char msg[len+1];
  memcpy(msg, data, len);
  msg[len] = '\0';
  
  if (shouldPrint) {
    Serial.print("Msg: "); Serial.println(msg);
  }

  // parse key=value pairs separated by '&'
  char *p = strtok(msg, "&");
  int speed = 0;
  int turn = 0;
  bool forward = true;
  while (p) {
    if (strncmp(p, "speed=", 6) == 0) speed = atoi(p + 6);
    else if (strncmp(p, "forwardBackward=", 16) == 0) forward = (strcmp(p + 16, "forward") == 0);
    else if (strncmp(p, "turnRate=", 9) == 0) turn = atoi(p + 9);
    p = strtok(NULL, "&");
  }
  
  if (shouldPrint) {
    Serial.print("Parsed speed="); Serial.print(speed);
    Serial.print(" turn="); Serial.print(turn);
    Serial.print(" forward="); Serial.println(forward);
  }

  // map 0-100 to 0-255
  cmdSpeed = map(constrain(speed, 0, 100), 0, 100, 0, 255);
  // map -100..100 to -255..255
  cmdTurn = map(constrain(turn, -100, 100), -100, 100, -255, 255);
  cmdForward = forward;
}

void setup() {
  // configure direction pins
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  // configure PWM pins
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);

  // set default forward direction
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);

  // attach PWM to pins
  ledcAttach(PWMA, LEDC_FREQUENCY, LEDC_RESOLUTION_BITS);
  ledcAttach(PWMB, LEDC_FREQUENCY, LEDC_RESOLUTION_BITS);

  // initialize Serial and AMG8833 sensor
  Serial.begin(115200);
  Serial.println("\nThermal Bot Main Board");
  Wire.begin(SDA, SCL); // SDA, SCL
  if (!amg.begin()) {
    Serial.println("AMG8833 not found");
    while (1);
  }
  Serial.println("AMG8833 initialized");

  // Set the controller MAC from your debug output
  // Controller's STA MAC address in array format
  uint8_t controller_sta_mac[] = {0xE4, 0xB3, 0x23, 0xD3, 0x66, 0xB8};
  memcpy(controllerMAC, controller_sta_mac, 6);
  
  // Or uncomment to use broadcast instead
  // memset(controllerMAC, 0xFF, 6);
  
  Serial.print("Using controller MAC: ");
  for (int i = 0; i < 6; i++) {
    Serial.print(controllerMAC[i], HEX);
    if (i < 5) Serial.print(":");
  }
  Serial.println();

  // initialize WiFi and ESP-NOW
  WiFi.mode(WIFI_STA);
  Serial.print("Main board MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
  } else {
    Serial.println("ESP-NOW initialized");
  }
  esp_now_register_recv_cb(onDataRecv);

  // Add controller MAC as peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, controllerMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add controller peer");
  } else {
    Serial.println("Added controller peer");
  }
  
  // Add broadcast address as peer (fallback)
  const uint8_t broadcastAddr[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
  memcpy(peerInfo.peer_addr, broadcastAddr, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add broadcast peer");
  } else {
    Serial.println("Added broadcast peer");
  }

  // Debug blink
  for (int i = 0; i < 3; i++) {
    digitalWrite(AIN1, HIGH);
    delay(100);
    digitalWrite(AIN1, LOW);
    delay(100);
  }
}

void motorA(int speed) {
  int pwm = map(constrain(speed, 0, 255), 0, 255, 0, LEDC_RESOLUTION);
  ledcWrite(PWMA, pwm);
}

void motorB(int speed) {
  int pwm = map(constrain(speed, 0, 255), 0, 255, 0, LEDC_RESOLUTION);
  ledcWrite(PWMB, pwm);
}

void loop() {
  unsigned long currentMillis = millis();
  bool shouldPrint = currentMillis - lastDebugPrintTime >= DEBUG_PRINT_INTERVAL;
  
  // read thermal sensor and print values
  amg.readPixels(pixels);
  
  // Only print thermal data once per second
  if (shouldPrint) {
    for (int i = 0; i < ROWS * COLS; i++) {
      Serial.print(pixels[i], 1);
      if (i < ROWS * COLS - 1) Serial.print(",");
    }
    Serial.println();
  }

  // drive motors according to last received command
  if (shouldPrint) {
    Serial.print("cmdSpeed="); Serial.print(cmdSpeed);
    Serial.print(" cmdTurn="); Serial.print(cmdTurn);
    Serial.print(" cmdForward="); Serial.println(cmdForward);
  }

  if (cmdForward) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  }
  int mA = constrain(cmdSpeed + cmdTurn, 0, 255);
  int mB = constrain(cmdSpeed - cmdTurn, 0, 255);
  
  if (shouldPrint) {
    Serial.print("mA="); Serial.print(mA);
    Serial.print(" mB="); Serial.println(mB);
    lastDebugPrintTime = currentMillis;
  }

  motorA(mA);
  motorB(mB);

  // send thermal data at configured rate 
  if (millis() - lastCameraSendTime >= cameraSendInterval) {
    lastCameraSendTime = millis();
    // send compact binary thermal data (64 bytes)
    uint8_t buf[ROWS * COLS];
    for (int i = 0; i < ROWS * COLS; i++) {
      buf[i] = (uint8_t)round(pixels[i]);  // integer degrees
    }
    
    // Only print send status if debug interval passed
    bool printSendStatus = (currentMillis - lastDebugPrintTime >= DEBUG_PRINT_INTERVAL);
    if (printSendStatus) {
      Serial.print("Sending thermal data to: ");
      for (int i = 0; i < 6; i++) {
        Serial.print(controllerMAC[i], HEX);
        if (i < 5) Serial.print(":");
      }
    }
    
    esp_err_t res = esp_now_send(controllerMAC, buf, sizeof(buf));
    if (res != ESP_OK) {
      // Always print errors
      Serial.print(" - ERROR: "); Serial.println(res);
      // If failed and not already broadcasting, fall back to broadcast
      if (controllerMAC[0] != 0xFF) {
        Serial.println("Falling back to broadcast");
        memset(controllerMAC, 0xFF, 6);  // Set to broadcast address
        res = esp_now_send(controllerMAC, buf, sizeof(buf));
        if (res != ESP_OK) {
          Serial.print("Broadcast also failed: "); Serial.println(res);
        }
      }
    } else if (printSendStatus) {
      Serial.println(" - OK");
    }
  }

  delay(100);
}
