#include <Arduino.h>
#include "ESPNowCam.h"

#include "lgfx_custom_ili9341_conf.hpp"
#include <LGFX_TFT_eSPI.hpp>

// pins for Metro ESP32S3
/*
#define TFT_CS 2
#define TFT_DC 3
#define TFT_MOSI 42
#define TFT_MISO 21
#define TFT_SCK 39
#define TFT_RST -1 // can set to -1 if tied to Arduino RESET pin
*/

// MAC addr of sender (Metro S3)
// 0x80, 0xB5, 0x4E, 0xCD, 0x29, 0x20

// MAC addr of receiver (XIAO ESP32-S3 Sense)
static const uint8_t MAC_RECV[6] = {0xE8, 0xF6, 0x0A, 0x8B, 0xB4, 0x94};

// ===== Radio (ESP-NOW) =====
ESPNowCam radio;

static TFT_eSPI lcd;              // Instance of LGFX

// panel logical size
static const int W = 320;
static const int H = 240;

// display globals
int32_t dw = 320;
int32_t dh = 240;

// *** buffers ***
static const size_t JPG_MAX = 96 * 1024;
static uint8_t *jpg = nullptr;

// simple FPS/diag
static uint32_t last_ms = 0;
static uint32_t frames = 0;


// === input defines ===
// Left joystick: 2-axis only
#define LEFT_STICK_X_PIN  A2
#define LEFT_STICK_Y_PIN  A3

// Right joystick: 2-axis + push button
#define RIGHT_STICK_X_PIN A0
#define RIGHT_STICK_Y_PIN A1
#define RIGHT_STICK_SW_PIN 7

static const int ADC_MAX = 4095;
static const int STICK_DEADZONE = 60;
static const int DRIVE_DIRECTION_THRESHOLD = 450;
static const int CAMERA_DIRECTION_THRESHOLD = 450;
// static const uint32_t STICK_PRINT_INTERVAL_MS = 100;
static const bool PRINT_CONTROL_SEND = false;
static const uint32_t SEND_FAIL_PRINT_INTERVAL_MS = 1000;
static const int ADC_SAMPLES_PER_AXIS = 4;

static const int LEFT_STICK_X_CENTER = 2653;
static const int LEFT_STICK_Y_CENTER = 2690;
static const int RIGHT_STICK_X_CENTER = 2725;
static const int RIGHT_STICK_Y_CENTER = 2720;

// structure to hold control states
struct controlState {
  int8_t drive_throttle;   // -100..100
  int8_t drive_turn;       // -100..100
  int8_t cam_yaw;          // -100..100
  uint8_t flags;           // bit 0 = automated sequence start
};

// last sent control state
controlState lastSend = {0, 0, 0, 0};
// time since last send
unsigned long lastSentMs = 0;
// unsigned long lastStickPrintMs = 0;
unsigned long lastSendFailPrintMs = 0;
// maximum ms between sends
const unsigned long HEARTBEAT_MS = 200; // 200ms for 5Hz


static void onDataReady(uint32_t length) {
  if (!length || length > JPG_MAX) {
    Serial.printf("onDataReady: bad len=%u (max=%u)\n", (unsigned)length, (unsigned)JPG_MAX);
    return;
  }

  lcd.startWrite();
  // LovyanGFX decodes JPEG and pushes in tiles safely for ESP32-S3
  lcd.drawJpg(jpg, length, 0, 0);   // x=0, y=0
  lcd.endWrite();

  // crude FPS
  frames++;
  uint32_t now = millis();
  if (now - last_ms >= 1000) {
    // Serial.printf("fps=%u, last_jpg_bytes=%u\n", frames, (unsigned)length);
    frames = 0;
    last_ms = now;
  }
}

// prints joystick and button states
// button state is printed only once when first pressed
// this simulates the single-use CST device functionality

static int8_t quantizeAxis(int raw, int center, bool invert = false) {
  int delta = raw - center;
  if (invert) delta = -delta;

  if (abs(delta) <= STICK_DEADZONE) {
    return 0;
  }

  int positiveSpan = ADC_MAX - center;
  int negativeSpan = center;
  int span = (delta >= 0) ? positiveSpan : negativeSpan;
  span = max(1, span - STICK_DEADZONE);

  int magnitude = min(abs(delta) - STICK_DEADZONE, span);
  float normalized = float(magnitude) / float(span);
  // Mild expo keeps center control gentle without losing full travel.
  float shaped = normalized * normalized;
  int value = int(shaped * 100.0f + 0.5f);
  return (delta < 0) ? -value : value;
}

static int readStableAxis(int pin) {
  // ESP32 ADC mux changes can bleed across channels with higher source impedance.
  // Throw away the first sample, then average a few reads for a cleaner value.
  analogRead(pin);
  delayMicroseconds(150);

  uint32_t sum = 0;
  for (int i = 0; i < ADC_SAMPLES_PER_AXIS; ++i) {
    sum += (uint32_t)analogRead(pin);
    delayMicroseconds(50);
  }

  return (int)(sum / ADC_SAMPLES_PER_AXIS);
}

static void quantizeDriveDirection(int leftX, int leftY, int8_t &driveThrottle, int8_t &driveTurn) {
  driveThrottle = 0;
  driveTurn = 0;

  int throttleDelta = leftY - LEFT_STICK_Y_CENTER;
  int turnDelta = leftX - LEFT_STICK_X_CENTER;
  int throttleMagnitude = abs(throttleDelta);
  int turnMagnitude = abs(turnDelta);
  int strongestInput = max(throttleMagnitude, turnMagnitude);

  if (strongestInput < DRIVE_DIRECTION_THRESHOLD) {
    return;
  }

  if (throttleMagnitude >= turnMagnitude) {
    driveThrottle = (throttleDelta > 0) ? 100 : -100;
  } else {
    driveTurn = (turnDelta > 0) ? 100 : -100;
  }
}

static int8_t quantizeCameraDirection(int rightX) {
  int delta = rightX - RIGHT_STICK_X_CENTER;

  if (abs(delta) < CAMERA_DIRECTION_THRESHOLD) {
    return 0;
  }

  return (delta > 0) ? 100 : -100;
}

static controlState quantizeInputs() {
  controlState c = {0, 0, 0, 0};

  int leftX = readStableAxis(LEFT_STICK_X_PIN);
  int leftY = readStableAxis(LEFT_STICK_Y_PIN);
  int rightX = readStableAxis(RIGHT_STICK_X_PIN);

  quantizeDriveDirection(leftX, leftY, c.drive_throttle, c.drive_turn);
  c.cam_yaw = quantizeCameraDirection(rightX);

  if (digitalRead(RIGHT_STICK_SW_PIN) == LOW) {
    c.flags |= 0x01;
  }

  return c;
}

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.print("\r\nLast Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}


void setup() {
  Serial.begin(115200);
  delay(1000);

  // ===== Display Init =====
  lcd.init();
  lcd.invertDisplay(true);
  lcd.setRotation(0);
  lcd.setBrightness(255);
  lcd.setColorDepth(16);
  lcd.fillScreen(TFT_BLACK);

  // ===== Radio Init =====
  //radio.setTarget(MAC_RECV);
  //radio.init();

 // ---- Buffers ----
  jpg = (uint8_t*) malloc(JPG_MAX);
  if (!jpg) {
    Serial.println("jpg alloc failed");
    while (1) delay(1000);
  }


  // ---- ESPNOW  receiver ----
  radio.setRecvBuffer(jpg);             // receive compressed JPEG into 'jpg'
  radio.setRecvCallback(onDataReady);

  // ---- ESPNOW init ----
  radio.setTarget(MAC_RECV);
  if (radio.init()) {
    Serial.println("ESPNow Init Success");
    lcd.setCursor(6, 6);
    lcd.println("ESPNow Init Success");
  } else {
    Serial.println("ESPNow Init FAILED");
    lcd.setCursor(6, 6);
    lcd.println("ESPNow Init FAILED");
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Transmitted packet
  esp_now_register_send_cb(OnDataSent);

  analogSetAttenuation(ADC_11db); // set full range 3.3V for the analog pins

  pinMode(RIGHT_STICK_SW_PIN, INPUT_PULLUP); // set pin mode for right stick button
}

void loop(void) {
  uint32_t now = millis();

  // if (now - lastStickPrintMs >= STICK_PRINT_INTERVAL_MS) {
  //   lastStickPrintMs = now;
  //   int leftX = analogRead(LEFT_STICK_X_PIN);
  //   int leftY = analogRead(LEFT_STICK_Y_PIN);
  //   int rightX = analogRead(RIGHT_STICK_X_PIN);
  //   int rightY = analogRead(RIGHT_STICK_Y_PIN);
  //   int rightSw = digitalRead(RIGHT_STICK_SW_PIN);
  //
  //   Serial.printf(
  //     "LX=%d LY=%d RX=%d RY=%d SW=%d\n",
  //     leftX,
  //     leftY,
  //     rightX,
  //     rightY,
  //     rightSw
  //   );
  // }

  controlState cur = quantizeInputs();

  bool stateChanged =
    (cur.drive_throttle != lastSend.drive_throttle) ||
    (cur.drive_turn != lastSend.drive_turn) ||
    (cur.cam_yaw != lastSend.cam_yaw) ||
    (cur.flags != lastSend.flags);
  bool heartbeat = (now - lastSentMs) >= HEARTBEAT_MS; // 5 Hz

  // if state changed or heartbeat timeout, send update
  if (stateChanged || heartbeat) {
    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(MAC_RECV, (uint8_t *) &cur, sizeof(cur));
    if (result == ESP_OK) {
      //Serial.println("Sent with success");
    } else {
      if (now - lastSendFailPrintMs >= SEND_FAIL_PRINT_INTERVAL_MS) {
        lastSendFailPrintMs = now;
        Serial.println("ESPNOW Send Failed");
      }
    }
    lastSend = cur;
    lastSentMs = millis();

    if (PRINT_CONTROL_SEND) {
      Serial.printf(
        "Sent: thr=%d turn=%d yaw=%d flags=0x%02X\n",
        (int)cur.drive_throttle,
        (int)cur.drive_turn,
        (int)cur.cam_yaw,
        (unsigned)cur.flags
      );
    }
  }
}
