#include <Arduino.h>
#include "esp_camera.h"
#include <ESPNowCam.h>

// ===== controller side ESP32 MAC (Metro S3) =====
static const uint8_t MAC_RECV[6] = {0x80, 0xB5, 0x4E, 0xCD, 0x29, 0x20};

// ===== radio =====
ESPNowCam radio;

// Throttle to avoid saturating ESPNOW
static uint32_t lastSend = 0;
static const uint32_t SEND_INTERVAL_MS = 120;

// ===== frame settings =====
static int CAM_JPEG_QUALITY = 45;                    // lower = better quality
static framesize_t CAM_FRAMESIZE = FRAMESIZE_HVGA;   // fallback default
#define CAM_FB_COUNT 1
#define CAM_XCLK_HZ 10000000

// XIAO camera pin map from Arduino CameraWebServer example
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  10
#define SIOD_GPIO_NUM  40
#define SIOC_GPIO_NUM  39
#define Y9_GPIO_NUM    48
#define Y8_GPIO_NUM    11
#define Y7_GPIO_NUM    12
#define Y6_GPIO_NUM    14
#define Y5_GPIO_NUM    16
#define Y4_GPIO_NUM    18
#define Y3_GPIO_NUM    17
#define Y2_GPIO_NUM    15
#define VSYNC_GPIO_NUM 38
#define HREF_GPIO_NUM  47
#define PCLK_GPIO_NUM  13

// controls state from controller
struct controlState {
  int8_t drive_throttle;   // -100..100
  int8_t drive_turn;       // -100..100
  int8_t cam_yaw;          // -100..100
  uint8_t flags;           // bit 0 = automated sequence start
};

controlState latestControl = {0, 0, 0, 0};
volatile bool controlUpdated = false;

unsigned long lastSentMs = 0;
const unsigned long HEARTBEAT_MS = 200;

// External MCU command output (change if needed)
static const int CTRL_TX_PIN = D6; // GPIO43 on XIAO ESP32-S3

static void forwardControlToUno(const controlState &cs) {
  Serial.printf(
    "UNO ctrl: thr=%d turn=%d flags=0x%02X\n",
    (int)cs.drive_throttle,
    (int)cs.drive_turn,
    (unsigned)cs.flags
  );

  Serial2.printf(
    "%d,%d,%u\n",
    (int)cs.drive_throttle,
    (int)cs.drive_turn,
    (unsigned)cs.flags
  );
}

static camera_config_t camCfg() {
  camera_config_t c = {};
  c.ledc_timer   = LEDC_TIMER_0;
  c.ledc_channel = LEDC_CHANNEL_0;

  c.pin_d0 = Y2_GPIO_NUM;
  c.pin_d1 = Y3_GPIO_NUM;
  c.pin_d2 = Y4_GPIO_NUM;
  c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM;
  c.pin_d5 = Y7_GPIO_NUM;
  c.pin_d6 = Y8_GPIO_NUM;
  c.pin_d7 = Y9_GPIO_NUM;

  c.pin_xclk = XCLK_GPIO_NUM;
  c.pin_pclk = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM;
  c.pin_href = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM;
  c.pin_sccb_scl = SIOC_GPIO_NUM;

  c.pin_pwdn = PWDN_GPIO_NUM;
  c.pin_reset = RESET_GPIO_NUM;

  c.xclk_freq_hz = CAM_XCLK_HZ;
  c.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    c.fb_location = CAMERA_FB_IN_PSRAM;
    c.frame_size = FRAMESIZE_QVGA;
    c.jpeg_quality = 14;
    Serial.println("PSRAM found");
  } else {
    c.fb_location = CAMERA_FB_IN_DRAM;
    c.frame_size = FRAMESIZE_QVGA;
    c.jpeg_quality = 20;
    Serial.println("Using DRAM");
  }

  CAM_JPEG_QUALITY = c.jpeg_quality;
  CAM_FRAMESIZE = c.frame_size;
  c.fb_count = CAM_FB_COUNT;
  c.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

  return c;
}

static void initCameraOrHalt() {
  camera_config_t cfg = camCfg();
  esp_err_t err = esp_camera_init(&cfg);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed (0x%X)\n", (int)err);
    while (true) delay(1000);
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_quality(s, CAM_JPEG_QUALITY);
    s->set_framesize(s, CAM_FRAMESIZE);
    s->set_contrast(s, 0);
  }
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (len != sizeof(controlState)) {
    Serial.printf("Received invalid control data size: %d\n", len);
    return;
  }

  controlState tmp;
  memcpy(&tmp, incomingData, sizeof(controlState));

  noInterrupts();
  latestControl = tmp;
  controlUpdated = true;
  interrupts();
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nESPNowCam XIAO ESP32-S3 sender");

  // TX-only stream for control forwarding to downstream MCU
  Serial2.begin(115200, SERIAL_8N1, -1, CTRL_TX_PIN);

  initCameraOrHalt();

  radio.setTarget(MAC_RECV);
  if (!radio.init()) {
    Serial.println("ESPNow init failed");
  }

  esp_now_register_recv_cb(OnDataRecv);

  if (psramFound()) {
    size_t mb = esp_spiram_get_size() / 1048576;
    // Serial.printf("PSRAM: %u MB\n", (unsigned)mb);
  }
}

void loop() {
  controlState cs;
  static int count = 0;

  uint32_t now = millis();
  bool shouldForwardControl = false;

  noInterrupts();
  memcpy(&cs, (const void *)&latestControl, sizeof(cs));
  if (controlUpdated || (now - lastSentMs) >= HEARTBEAT_MS) {
    shouldForwardControl = true;
    controlUpdated = false;
  }
  interrupts();

  if (shouldForwardControl) {
    lastSentMs = now;
    forwardControlToUno(cs);
  }

  if (now - lastSend < SEND_INTERVAL_MS) {
    delay(1);
    return;
  }
  lastSend = now;

  delay(0);
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Capture failed");
    delay(5);
    return;
  }

  radio.sendData(fb->buf, fb->len);

  // if ((++count % 5) == 0) {
  //   Serial.printf("Sent %u bytes\n", (unsigned)fb->len);
  // }
  ++count;

  esp_camera_fb_return(fb);
  delay(0);
}

