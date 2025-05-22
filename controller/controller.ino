/*
Joystick Control for ESP32 with ESP-NOW
Author: RawFish69
*/

#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <WebServer.h>

#define JOYSTICK_X_PIN 2 
#define JOYSTICK_Y_PIN 1 
#define EMERGENCY_STOP_PIN 0 

// Debug print interval
#define DEBUG_PRINT_INTERVAL 1000
unsigned long lastDebugPrintTime = 0;

IPAddress local_IP(192, 168, 1, 101);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

int joyCenterX = 0;
int joyCenterY = 0;

// ESP-NOW peer configurations
uint8_t DEFAULT_DRIVE_MAC[6] = {0xE4, 0xB3, 0x23, 0xD2, 0xD1, 0x5C}; // Drive board STA MAC
uint8_t DRIVE_BOARD_MAC[6];
uint8_t MY_MAC[6]; // Store controller's own MAC

bool firstSetup = true;

unsigned long lastJoystickCommandTime = 0;
const unsigned long commandInterval = 200; // in ms

float lastSpeed = 0, lastTurn = 0;
String lastDir = "";
String lastDelivery = "Sending...";
static unsigned long lastSendTime = 0;

// define thermal image dimensions and storage
#define ROWS 8
#define COLS 8
float thermalPixels[ROWS * COLS];

WebServer server(80);

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    lastDelivery = "Success";
  } else {
    lastDelivery = "Fail";
  }
}

void sendEspNowMessage(const String &msg) {
  unsigned long now = millis();
  if (now - lastSendTime < 20) {
    delay(20 - (now - lastSendTime));
  }
  lastSendTime = millis();
  esp_err_t ret = esp_now_send(DRIVE_BOARD_MAC, (const uint8_t*)msg.c_str(), msg.length() + 1);
  if (ret != ESP_OK) {
    lastDelivery = "SendFail(" + String(ret) + ")";
  }
}

void showSimpleCommand(float spd, const String &dir, float trn) {
  lastSpeed = spd;
  lastDir = dir;
  lastTurn = trn;
  lastDelivery = "Sending...";
}

void initEspNowAndPeer() {
  if (!firstSetup) {
    esp_now_del_peer(DRIVE_BOARD_MAC);
    esp_now_deinit();
  }
  memcpy(DRIVE_BOARD_MAC, DEFAULT_DRIVE_MAC, 6);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESPNOW init fail");
    delay(2000);
    ESP.restart();
  }
  esp_now_register_send_cb(onDataSent);
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, DRIVE_BOARD_MAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Peer Add FAIL");
  }
  firstSetup = false;
}

// Calibration function: average joystick readings over 3 seconds
void calibrateJoystick() {
  Serial.println("Calibrating joystick... Please keep joystick centered.");
  unsigned long startTime = millis();
  long sumX = 0, sumY = 0;
  int samples = 0;
  while (millis() - startTime < 3000) {
    sumX += analogRead(JOYSTICK_X_PIN);
    sumY += analogRead(JOYSTICK_Y_PIN);
    samples++;
    delay(10);
  }
  joyCenterX = sumX / samples;
  joyCenterY = sumY / samples;
  Serial.print("Calibration done. Center X: ");
  Serial.print(joyCenterX);
  Serial.print(", Center Y: ");
  Serial.println(joyCenterY);
}

// Add helper function to map joystick raw values to -100 to 100 range based on the calibrated center
int mapJoystickValue(int raw, int center) {
  if (raw >= center)
    return map(raw, center, 4095, 0, 100);
  else
    return map(raw, 0, center, -100, 0);
}

// receive callback for thermal data
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  // Only print debug information once per second
  unsigned long currentMillis = millis();
  bool shouldPrint = currentMillis - lastDebugPrintTime >= DEBUG_PRINT_INTERVAL;
  
  if (shouldPrint) {
    String macStr = "";
    for (int i = 0; i < 6; i++) {
      macStr += String(info->src_addr[i], HEX);
      if (i < 5) macStr += ":";
    }
    Serial.print("Received from "); 
    Serial.print(macStr);
    Serial.print(" data length: ");
    Serial.println(len);
  }

  // parse binary degrees payload
  if (len == ROWS*COLS) {
    for (int i = 0; i < ROWS*COLS; i++) {
      thermalPixels[i] = (float)data[i];
    }
    
    // print to serial only if debug interval passed
    if (shouldPrint) {
      Serial.println("Thermal Data Received:");
      for (int r = 0; r < ROWS; r++) {
        for (int c = 0; c < COLS; c++) {
          Serial.print(thermalPixels[r*COLS + c], 1);
          Serial.print(" ");
        }
        Serial.println();
      }
      lastDebugPrintTime = currentMillis;
    }
  } else if (shouldPrint) {
    Serial.println("Not thermal data (wrong length)");
    // Try to interpret as a command string
    char msg[len+1];
    memcpy(msg, data, len);
    msg[len] = '\0';
    Serial.print("Message: ");
    Serial.println(msg);
    lastDebugPrintTime = currentMillis;
  }
}

// JSON endpoint for thermal data
void handleData() {
  String json = "[";
  for (int r = 0; r < ROWS; r++) {
    json += "[";
    for (int c = 0; c < COLS; c++) {
      json += String(thermalPixels[r*COLS + c], 1);
      if (c < COLS - 1) json += ',';
    }
    json += "]";
    if (r < ROWS - 1) json += ',';
  }
  json += "]";
  server.send(200, "application/json", json);
}

// serve graphical thermal view
void handleRoot() {
  String html = "<html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width, initial-scale=1'>"
                "<title>Thermal Bot</title><style>html,body{margin:0;padding:0;overflow:hidden;}"  
                "#status{position:absolute;top:0;left:0;padding:10px;background:rgba(0,0,0,0.6);color:#fff;font-family:sans-serif;z-index:1;}"  
                "canvas{display:block;width:100vw;height:100vh;}</style></head><body>";
  html += "<div id='status'>Speed: <span id='speed'>0</span> Direction: <span id='dir'>N/A</span> Turn: <span id='turn'>0</span> Delivery: <span id='del'>-</span></div>";
  html += "<canvas id='canvas'></canvas>";
  html += "<script>const rows=8,cols=8;const canvas=document.getElementById('canvas');const ctx=canvas.getContext('2d');"
          "canvas.width=window.innerWidth;canvas.height=window.innerHeight;"
          "function draw(){fetch('/data').then(r=>r.json()).then(d=>{"
            "const w=canvas.width/cols,h=canvas.height/rows;"
            "for(let i=0;i<rows;i++){for(let j=0;j<cols;j++){"
              "let v=d[i][j],t=(v-20)/40;t=Math.max(0,Math.min(1,t));"
              "ctx.fillStyle='rgb('+Math.floor(255*t)+',0,'+Math.floor(255*(1-t))+')';"
              "ctx.fillRect(j*w,i*h,w,h);} }"
          "});fetch('/status').then(r=>r.json()).then(s=>{"
            "document.getElementById('speed').innerText=s.speed;"
            "document.getElementById('dir').innerText=s.direction;"
            "document.getElementById('turn').innerText=s.turn;"
            "document.getElementById('del').innerText=s.delivery;"
          "});}"
          "setInterval(draw,200);window.addEventListener('resize',()=>{canvas.width=window.innerWidth;canvas.height=window.innerHeight;});"
          "window.onload=draw;</script></body></html>";
  server.send(200, "text/html", html);
}

void setup() {
  Serial.begin(115200);
  
  // start Wi-Fi in combined AP+STA mode and launch SoftAP for web UI
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP("ThermalView", "");
  WiFi.softAPConfig(local_IP, gateway, subnet);
  // Get and store our own MAC for debug
  WiFi.macAddress(MY_MAC);
  Serial.print("Controller MAC: ");
  for (int i = 0; i < 6; i++) {
    Serial.print(MY_MAC[i], HEX);
    if (i < 5) Serial.print(":");
  }
  Serial.println();
  
  Serial.print("SoftAP IP: "); 
  Serial.println(WiFi.softAPIP());

  pinMode(JOYSTICK_X_PIN, INPUT);
  pinMode(JOYSTICK_Y_PIN, INPUT);
  pinMode(EMERGENCY_STOP_PIN, INPUT_PULLUP);
  calibrateJoystick();

  // initialize ESP-NOW
  initEspNowAndPeer();
  esp_now_register_recv_cb(onDataRecv);

  // setup web server routes
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/status", []() {
    String json = "{";
    json += "\"speed\":" + String(lastSpeed,1) + ",";
    json += "\"direction\":\"" + lastDir + "\",";
    json += "\"turn\":" + String(lastTurn,1) + ",";
    json += "\"delivery\":\"" + lastDelivery + "\"";
    json += "}";
    server.send(200, "application/json", json);
  });

  server.begin();
  Serial.println("Web server started");

  Serial.println("Joystick Control Setup complete");
}

void loop() {
  server.handleClient();
  unsigned long currentMillis = millis();
  
  if(digitalRead(EMERGENCY_STOP_PIN) == LOW) {
    // Always print emergency stop messages
    showSimpleCommand(0, "stop", 0);
    String query = "mode=default&speed=0&forwardBackward=stop&turnRate=0";
    sendEspNowMessage(query);
    Serial.println("Emergency Stop Activated");
    delay(10);
    return;  
  }
  
  if (millis() - lastJoystickCommandTime >= commandInterval) {
    lastJoystickCommandTime = millis();
    int rawX = analogRead(JOYSTICK_X_PIN);
    int rawY = analogRead(JOYSTICK_Y_PIN);
    int mappedX = mapJoystickValue(rawX, joyCenterX);
    int mappedY = mapJoystickValue(rawY, joyCenterY);
    
    float speed = abs(mappedY);
    String direction = (mappedY >= 0) ? "forward" : "backward";
    float turn = -mappedX;  // Inverted to correct left/right flipping
    showSimpleCommand(speed, direction, turn);
    String query = "mode=default&speed=" + String(speed, 0)
                 + "&forwardBackward=" + direction
                 + "&turnRate=" + String(turn, 0);
    sendEspNowMessage(query);
    
    // Only print joystick values once per second
    if (currentMillis - lastDebugPrintTime >= DEBUG_PRINT_INTERVAL) {
      Serial.print("X: ");
      Serial.print(rawX);
      Serial.print(" (");
      Serial.print(mappedX);
      Serial.print("%), ");
      Serial.print("Y: ");
      Serial.print(rawY);
      Serial.print(" (");
      Serial.print(mappedY);
      Serial.println("%)");
      lastDebugPrintTime = currentMillis;
    }
  }
  delay(100);
}