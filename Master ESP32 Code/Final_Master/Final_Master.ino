/*
  =============================== MASTER ESP32 ===============================
  Mode: SoftAP + UDP command server + Serial command bridge
  Features:
    • 9 Relays: R<0..8>:<0|1>      -> e.g., "R3:1"
    • Motor  (L298N):              -> "MOTOR F<duty>", "MOTOR R<duty>", "MOTOR B", "MOTOR C", "MOTOR D<duty>", "MOTOR S"
    • Status:                      -> "STATUS"
    • Servo1 continuous:           -> "LEFT", "RIGHT", "STOP", "CENTER" (+ speed: "LEFT:8", "RIGHT:3")
    • Servo2 continuous:           -> "LEFT2", "RIGHT2", "STOP2", "CENTER2" (+ speed: "LEFT2:8", "RIGHT2:3")
    • Servo1 absolute:             -> "S1:<deg>"
    • Servo2 absolute:             -> "S2:<deg>"
    • Pair absolute:               -> "<A>,<B>"  (comma-separated integers)

  ESP-NOW protocol to SLAVE:
    - Continuous control packet (magic 0xA5):
        struct { uint8_t magic=0xA5, cmd(0/1/2/3), which(0:servo1,1:servo2), speed(1..10) }
    - Single-servo absolute packet (magic 0xA6):
        struct { uint8_t magic=0xA6, which(0/1), uint16_t deg(0..180) }
    - Pair absolute angles packet (no magic):
        struct { uint16_t s1_deg, s2_deg }

  Slave MAC: EC:E3:34:22:79:18

  Notes:
  - Keep line ending in Serial Monitor to "Both NL & CR" (115200 baud)
  - GUI sends same strings over UDP port 4210
  - Master sends via ESP-NOW on AP interface; no callbacks used here
  ============================================================================
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "driver/ledc.h"
#include <esp_now.h>

// ---------------------------- WiFi SoftAP Config ----------------------------
static const char* AP_SSID = "ESP-MASTER";
static const char* AP_PASS = "12345678";
static const IPAddress AP_IP   (192, 168, 4, 1);
static const IPAddress AP_GW   (192, 168, 4, 1);
static const IPAddress AP_MASK (255, 255, 255, 0);

static const uint16_t UDP_PORT = 4210;

// ---------------------------- UDP & Serial State ----------------------------
WiFiUDP udp;
String   __serialLine;

// ---------------------------- Relay Config (9 ch) ---------------------------
static const int RELAY_PINS[9] = {27, 32, 4, 5, 18, 19, 21, 22, 23};
uint16_t relayStateBits = 0;  // bit i -> relay i ON=1

// ---------------------------- Motor (L298N) ---------------------------------
static const int PIN_ENB = 33;  // PWM
static const int PIN_IN3 = 26;  // Direction
static const int PIN_IN4 = 25;  // Direction

// LEDC PWM config
static const ledc_mode_t       LEDC_MODE      = LEDC_LOW_SPEED_MODE;
static const ledc_timer_t      LEDC_TIMER     = LEDC_TIMER_0;
static const ledc_channel_t    LEDC_CHANNEL   = LEDC_CHANNEL_0;
static const ledc_timer_bit_t  LEDC_DUTY_BITS = LEDC_TIMER_8_BIT; // 0..255
static const uint32_t          LEDC_FREQ_HZ   = 1000;

uint8_t motorDuty = 0;

// ---------------------------- ESP-NOW / Servo --------------------------------
static uint8_t SLAVE_PEER_MAC[6] = {0xEC, 0xE3, 0x34, 0x22, 0x79, 0x18};

typedef struct __attribute__((packed)) {
  uint16_t s1_deg; // 0..180
  uint16_t s2_deg; // 0..180
} ServoPacketAngles; // pair, no magic

typedef struct __attribute__((packed)) {
  uint8_t magic;   // 0xA5
  uint8_t cmd;     // 0=STOP,1=LEFT,2=RIGHT,3=CENTER
  uint8_t which;   // 0=servo1, 1=servo2
  uint8_t speed;   // 1..10
} ServoPacketCmd;

typedef struct __attribute__((packed)) {
  uint8_t  magic;  // 0xA6
  uint8_t  which;  // 0=servo1, 1=servo2
  uint16_t deg;    // 0..180
} ServoPacketSingle;

enum : uint8_t { CMD_STOP=0, CMD_LEFT=1, CMD_RIGHT=2, CMD_CENTER=3 };

static esp_now_peer_info_t nowPeer{};
static bool nowPeerAdded = false;

static uint16_t lastServo1 = 90;
static uint16_t lastServo2 = 90;

static inline uint16_t clampDeg(int v) {
  if (v < 0)   return 0;
  if (v > 180) return 180;
  return (uint16_t)v;
}

// ---------------------------- Utility: PWM -----------------------------------
inline void pwmWrite(uint8_t duty) {
  ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
  ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

// ---------------------------- Motor Helpers ----------------------------------
void motorForward(uint8_t duty) {
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, HIGH);
  pwmWrite(duty);
  motorDuty = duty;
  Serial.printf("[MOTOR] FORWARD duty=%u\n", motorDuty);
}

void motorReverse(uint8_t duty) {
  digitalWrite(PIN_IN3, HIGH);
  digitalWrite(PIN_IN4, LOW);
  pwmWrite(duty);
  motorDuty = duty;
  Serial.printf("[MOTOR] REVERSE duty=%u\n", motorDuty);
}

void motorBrake() {
  digitalWrite(PIN_IN3, HIGH);
  digitalWrite(PIN_IN4, HIGH);
  pwmWrite(0);
  motorDuty = 0;
  Serial.println("[MOTOR] BRAKE");
}

void motorCoast() {
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, LOW);
  pwmWrite(0);
  motorDuty = 0;
  Serial.println("[MOTOR] COAST");
}

// ---------------------------- Status & Replies -------------------------------
String makeStatus() {
  char rel[16];
  for (int i = 0; i < 9; i++) rel[i] = (relayStateBits & (1 << i)) ? '1' : '0';
  rel[9] = '\0';

  char buf[96];
  snprintf(buf, sizeof(buf), "STATUS RELAYS=%s duty=%u", rel, motorDuty);
  return String(buf);
}

void sendReply(const IPAddress& ip, uint16_t port, const String& msg) {
  udp.beginPacket(ip, port);
  udp.print(msg);
  udp.endPacket();
}

// ---------------------------- ESP-NOW Senders --------------------------------
bool espNowSendAngles(uint16_t a, uint16_t b) {
  if (!nowPeerAdded) {
    Serial.println("[ESP-NOW] sendAngles: peer not added!");
    return false;
  }
  ServoPacketAngles pkt{ clampDeg((int)a), clampDeg((int)b) };
  esp_err_t err = esp_now_send(SLAVE_PEER_MAC, (uint8_t*)&pkt, sizeof(pkt));
  if (err != ESP_OK) {
    Serial.printf("[ESP-NOW] sendAngles error=%d\n", (int)err);
    return false;
  }
  Serial.printf("[ESP-NOW] ANGLES S1=%u S2=%u\n", pkt.s1_deg, pkt.s2_deg);
  return true;
}

bool espNowSendCmd(uint8_t cmd, uint8_t which, uint8_t speed) {
  if (!nowPeerAdded) {
    Serial.println("[ESP-NOW] sendCmd: peer not added!");
    return false;
  }
  if (speed < 1) speed = 1;
  if (speed > 10) speed = 10;
  ServoPacketCmd pkt{ 0xA5, cmd, (uint8_t)(which ? 1 : 0), speed };
  esp_err_t err = esp_now_send(SLAVE_PEER_MAC, (uint8_t*)&pkt, sizeof(pkt));
  if (err != ESP_OK) {
    Serial.printf("[ESP-NOW] sendCmd error=%d\n", (int)err);
    return false;
  }
  Serial.printf("[ESP-NOW] CMD magic=0xA5 cmd=%u which=%u speed=%u\n", pkt.cmd, pkt.which, pkt.speed);
  return true;
}

bool espNowSendSingle(uint8_t which, uint16_t deg) {
  if (!nowPeerAdded) {
    Serial.println("[ESP-NOW] sendSingle: peer not added!");
    return false;
  }
  ServoPacketSingle pkt{ 0xA6, (uint8_t)(which ? 1 : 0), clampDeg((int)deg) };
  esp_err_t err = esp_now_send(SLAVE_PEER_MAC, (uint8_t*)&pkt, sizeof(pkt));
  if (err != ESP_OK) {
    Serial.printf("[ESP-NOW] sendSingle error=%d\n", (int)err);
    return false;
  }
  Serial.printf("[ESP-NOW] SINGLE magic=0xA6 which=%u deg=%u\n", pkt.which, pkt.deg);
  return true;
}

// High-level convenience used by parser
bool setServos(int a, int b) { lastServo1 = clampDeg(a); lastServo2 = clampDeg(b); return espNowSendAngles(lastServo1, lastServo2); }
bool setServo1(int a)        { lastServo1 = clampDeg(a); return espNowSendSingle(0, lastServo1); }
bool setServo2(int b)        { lastServo2 = clampDeg(b); return espNowSendSingle(1, lastServo2); }
bool steerLeft (uint8_t sp=5, uint8_t which=0)  { return espNowSendCmd(CMD_LEFT,   which, sp); }
bool steerRight(uint8_t sp=5, uint8_t which=0)  { return espNowSendCmd(CMD_RIGHT,  which, sp); }
bool steerStop (uint8_t which=0)                { return espNowSendCmd(CMD_STOP,   which, 5); }
bool steerCenter(uint8_t which=0)               { return espNowSendCmd(CMD_CENTER, which, 5); }

// ---------------------------- ESP-NOW Init -----------------------------------
void beginSlaveControl() {
  Serial.println("[ESP-NOW] Initializing…");

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] init FAILED");
    return;
  }
  Serial.println("[ESP-NOW] init OK");

  memset(&nowPeer, 0, sizeof(nowPeer));
  memcpy(nowPeer.peer_addr, SLAVE_PEER_MAC, 6);
  nowPeer.channel = 0;            // use current AP channel
  nowPeer.encrypt = false;
  nowPeer.ifidx   = WIFI_IF_AP;   // we are running as SoftAP

  if (!esp_now_is_peer_exist(SLAVE_PEER_MAC)) {
    if (esp_now_add_peer(&nowPeer) == ESP_OK) {
      nowPeerAdded = true;
      Serial.println("[ESP-NOW] Peer added");
    } else {
      Serial.println("[ESP-NOW] Failed to add peer");
    }
  } else {
    nowPeerAdded = true;
    Serial.println("[ESP-NOW] Peer already exists");
  }
}

// ---------------------------- Command Parser ---------------------------------
void handleCommand(String line, const IPAddress& ip, uint16_t port) {
  line.trim();
  if (line.length() == 0) return;

  Serial.printf("[CMD] \"%s\" from %s:%u\n", line.c_str(), ip.toString().c_str(), port);

  // ---- STATUS ----
  if (line.startsWith("STATUS")) {
    String msg = makeStatus();
    sendReply(ip, port, msg);
    Serial.println(msg);
    return;
  }

  // ---- RELAYS: R<idx>:<0|1> ----
  if (line.startsWith("R")) {
    int colon = line.indexOf(':');
    if (colon > 1) {
      int idx = line.substring(1, colon).toInt();
      int val = line.substring(colon + 1).toInt();
      if (idx >= 0 && idx < 9) {
        digitalWrite(RELAY_PINS[idx], val ? HIGH : LOW);
        if (val) relayStateBits |=  (1 << idx);
        else     relayStateBits &= ~(1 << idx);
        String msg = "OK R" + String(idx) + " " + String(val);
        sendReply(ip, port, msg);
        Serial.println(msg);
      } else {
        String msg = "ERR bad relay index";
        sendReply(ip, port, msg);
        Serial.println(msg);
      }
    } else {
      String msg = "ERR relay syntax";
      sendReply(ip, port, msg);
      Serial.println(msg);
    }
    return;
  }

  // ---- MOTOR ----
  if (line.startsWith("MOTOR")) {
    String rest = line.substring(5);
    rest.trim();
    char c = rest.length() ? rest.charAt(0) : 0;

    if (c == 'F') {
      int d = rest.substring(1).toInt();
      if (d <= 0) d = motorDuty ? motorDuty : 200;
      motorForward(constrain(d, 0, 255));
      String msg = "OK MOTOR F " + String(motorDuty);
      sendReply(ip, port, msg); Serial.println(msg);
      return;
    }
    if (c == 'R') {
      int d = rest.substring(1).toInt();
      if (d <= 0) d = motorDuty ? motorDuty : 200;
      motorReverse(constrain(d, 0, 255));
      String msg = "OK MOTOR R " + String(motorDuty);
      sendReply(ip, port, msg); Serial.println(msg);
      return;
    }
    if (c == 'B') {
      motorBrake();
      String msg = "OK MOTOR B";
      sendReply(ip, port, msg); Serial.println(msg);
      return;
    }
    if (c == 'C') {
      motorCoast();
      String msg = "OK MOTOR C";
      sendReply(ip, port, msg); Serial.println(msg);
      return;
    }
    if (c == 'D') {
      int d = rest.substring(1).toInt();
      motorDuty = constrain(d, 0, 255);
      pwmWrite(motorDuty);
      String msg = "OK MOTOR D " + String(motorDuty);
      sendReply(ip, port, msg); Serial.println(msg);
      return;
    }
    if (c == 'S') {
      String msg = makeStatus();
      sendReply(ip, port, msg); Serial.println(msg);
      return;
    }

    String msg = "ERR motor syntax";
    sendReply(ip, port, msg); Serial.println(msg);
    return;
  }

  // ---- SERVO1 continuous: LEFT/RIGHT/STOP/CENTER (+speed) ----
  if (line.equalsIgnoreCase("LEFT"))   { steerLeft(5, 0);  String m="OK LEFT";    sendReply(ip,port,m); Serial.println(m); return; }
  if (line.equalsIgnoreCase("RIGHT"))  { steerRight(5, 0); String m="OK RIGHT";   sendReply(ip,port,m); Serial.println(m); return; }
  if (line.equalsIgnoreCase("STOP"))   { steerStop(0);     String m="OK STOP";    sendReply(ip,port,m); Serial.println(m); return; }
  if (line.equalsIgnoreCase("CENTER")) { steerCenter(0);   String m="OK CENTER";  sendReply(ip,port,m); Serial.println(m); return; }
  if (line.startsWith("LEFT:"))  { uint8_t sp = line.substring(5).toInt(); steerLeft(sp, 0);  String m="OK LEFT:"+String(sp);  sendReply(ip,port,m); Serial.println(m); return; }
  if (line.startsWith("RIGHT:")) { uint8_t sp = line.substring(6).toInt(); steerRight(sp, 0); String m="OK RIGHT:"+String(sp); sendReply(ip,port,m); Serial.println(m); return; }

  // ---- SERVO2 continuous: LEFT2/RIGHT2/STOP2/CENTER2 (+speed) ----
  if (line.equalsIgnoreCase("LEFT2"))   { steerLeft(5, 1);  String m="OK LEFT2";    sendReply(ip,port,m); Serial.println(m); return; }
  if (line.equalsIgnoreCase("RIGHT2"))  { steerRight(5, 1); String m="OK RIGHT2";   sendReply(ip,port,m); Serial.println(m); return; }
  if (line.equalsIgnoreCase("STOP2"))   { steerStop(1);     String m="OK STOP2";    sendReply(ip,port,m); Serial.println(m); return; }
  if (line.equalsIgnoreCase("CENTER2")) { steerCenter(1);   String m="OK CENTER2";  sendReply(ip,port,m); Serial.println(m); return; }
  if (line.startsWith("LEFT2:"))  { uint8_t sp = line.substring(6).toInt(); steerLeft(sp, 1);  String m="OK LEFT2:"+String(sp);  sendReply(ip,port,m); Serial.println(m); return; }
  if (line.startsWith("RIGHT2:")) { uint8_t sp = line.substring(7).toInt(); steerRight(sp, 1); String m="OK RIGHT2:"+String(sp); sendReply(ip,port,m); Serial.println(m); return; }

  // ---- Single-servo ABSOLUTE ----
  if (line.startsWith("S1:")) {
    int a = line.substring(3).toInt();
    setServo1(a);
    String m = "OK S1:" + String(a);
    sendReply(ip, port, m); Serial.println(m);
    return;
  }
  if (line.startsWith("S2:")) {
    int b = line.substring(3).toInt();
    setServo2(b);
    String m = "OK S2:" + String(b);
    sendReply(ip, port, m); Serial.println(m);
    return;
  }

  // ---- Pair ABSOLUTE "A,B" ----
  int comma = line.indexOf(',');
  if (comma > 0) {
    int a = line.substring(0, comma).toInt();
    int b = line.substring(comma + 1).toInt();
    setServos(a, b);
    String m = "OK ANGLES " + String(a) + "," + String(b);
    sendReply(ip, port, m); Serial.println(m);
    return;
  }

  // ---- Unknown ----
  String msg = "ERR Unknown cmd: " + line;
  sendReply(ip, port, msg); Serial.println(msg);
}

// ---------------------------- Serial Bridge ----------------------------------
void handleSerialLine() {
  __serialLine.trim();
  if (__serialLine.length() == 0) return;
  // For serial-originated commands, we pass AP_IP/UDP_PORT as reply target (harmless)
  handleCommand(__serialLine, AP_IP, UDP_PORT);
  Serial.printf("[SERIAL] %s\n", __serialLine.c_str());
  __serialLine = "";
}

// ---------------------------- PWM Setup --------------------------------------
void setupPWM() {
  ledc_timer_config_t tcfg = {};
  tcfg.speed_mode      = LEDC_MODE;
  tcfg.duty_resolution = LEDC_DUTY_BITS;
  tcfg.timer_num       = LEDC_TIMER;
  tcfg.freq_hz         = LEDC_FREQ_HZ;
  tcfg.clk_cfg         = LEDC_AUTO_CLK;
  ledc_timer_config(&tcfg);

  ledc_channel_config_t ccfg = {};
  ccfg.gpio_num   = PIN_ENB;
  ccfg.speed_mode = LEDC_MODE;
  ccfg.channel    = LEDC_CHANNEL;
  ccfg.intr_type  = LEDC_INTR_DISABLE;
  ccfg.timer_sel  = LEDC_TIMER;
  ccfg.duty       = 0;
  ccfg.hpoint     = 0;
  ledc_channel_config(&ccfg);
}

// ---------------------------- setup() ----------------------------------------
void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("=== MASTER START ===");

  // Relays init
  for (int i = 0; i < 9; i++) {
    pinMode(RELAY_PINS[i], OUTPUT);
    digitalWrite(RELAY_PINS[i], LOW);
  }
  relayStateBits = 0;

  // Motor init
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  setupPWM();
  motorCoast();

  // WiFi AP
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP, AP_GW, AP_MASK);
  WiFi.softAP(AP_SSID, AP_PASS);

  Serial.println("[WiFi] SoftAP started");
  Serial.print  ("       SSID: "); Serial.println(AP_SSID);
  Serial.print  ("       IP:   "); Serial.println(WiFi.softAPIP());

  // UDP server
  udp.begin(UDP_PORT);
  Serial.printf("[UDP] Listening on port %u\n", UDP_PORT);

  // ESP-NOW
  beginSlaveControl();

  Serial.println("=== MASTER READY ===");
}

// ---------------------------- loop() -----------------------------------------
void loop() {
  // Serial bridge
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      handleSerialLine();
    } else {
      __serialLine += c;
    }
  }

  // UDP handler
  int pktSize = udp.parsePacket();
  if (pktSize) {
    String line = udp.readString();
    handleCommand(line, udp.remoteIP(), udp.remotePort());
  }
}
