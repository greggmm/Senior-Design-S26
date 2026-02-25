#include <Arduino.h>

// Basic functional test for:
// - DRV8825 #1 (NEMA 17 "shaking"): DIR=5, STEP=6
// - DRV8825 #2 (NEMA 11 "breaking"): DIR=10, STEP=11
// - MPA2006 pump/vacuum: PIN=4
//
// Upload to Arduino Uno R3, open Serial Monitor @ 115200.

const int PUMP_PIN      = 4;

const int M1_DIR_PIN    = 5;   // NEMA 17
const int M1_STEP_PIN   = 6;

const int M2_DIR_PIN    = 10;  // NEMA 11
const int M2_STEP_PIN   = 11;

const int STEPS_PER_REV = 200; // typical for 1.8 deg steppers (adjust if needed)

void stepMotor(int dirPin, int stepPin, bool dir, int steps, int pulseDelayUs) {
  digitalWrite(dirPin, dir ? HIGH : LOW);
  delay(5); // small settle time after direction change

  for (int i = 0; i < steps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(pulseDelayUs);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(pulseDelayUs);
  }
}

// Simple linear acceleration/deceleration to reduce resonance/screeching.
void stepMotorRamped(int dirPin, int stepPin, bool dir, int steps, int startDelayUs, int runDelayUs) {
  digitalWrite(dirPin, dir ? HIGH : LOW);
  delay(5);

  int rampSteps = steps / 4;
  if (rampSteps < 20) rampSteps = 20;
  if (rampSteps * 2 > steps) rampSteps = steps / 2;

  for (int i = 0; i < steps; i++) {
    int d = runDelayUs;

    if (i < rampSteps) {
      d = startDelayUs - ((startDelayUs - runDelayUs) * i) / rampSteps;
    } else if (i >= steps - rampSteps) {
      int down = i - (steps - rampSteps);
      d = runDelayUs + ((startDelayUs - runDelayUs) * down) / rampSteps;
    }

    digitalWrite(stepPin, HIGH);
    delayMicroseconds(d);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(d);
  }
}

void setup() {
  pinMode(PUMP_PIN, OUTPUT);

  pinMode(M1_DIR_PIN, OUTPUT);
  pinMode(M1_STEP_PIN, OUTPUT);

  pinMode(M2_DIR_PIN, OUTPUT);
  pinMode(M2_STEP_PIN, OUTPUT);

  digitalWrite(PUMP_PIN, LOW);

  Serial.begin(115200);
  Serial.println("Starting component test...");
}

void loop() {
  // 1) Pump test
  Serial.println("Pump ON");
  digitalWrite(PUMP_PIN, HIGH);
  delay(2000);
  Serial.println("Pump OFF");
  digitalWrite(PUMP_PIN, LOW);
  delay(1000);

  // 2) Motor 1 (NEMA 17) test
  Serial.println("Motor 1 (NEMA 17) CW");
  stepMotorRamped(M1_DIR_PIN, M1_STEP_PIN, true, 2 * STEPS_PER_REV, 1800, 1200);
  delay(500);

  Serial.println("Motor 1 (NEMA 17) CCW");
  stepMotorRamped(M1_DIR_PIN, M1_STEP_PIN, false, 2 * STEPS_PER_REV, 1800, 1200);
  delay(1000);

  // 3) Motor 2 (NEMA 11) test
  Serial.println("Motor 2 (NEMA 11) CW");
  stepMotor(M2_DIR_PIN, M2_STEP_PIN, true,  2 * STEPS_PER_REV, 1000);
  delay(500);

  Serial.println("Motor 2 (NEMA 11) CCW");
  stepMotor(M2_DIR_PIN, M2_STEP_PIN, false, 2 * STEPS_PER_REV, 1000);
  delay(2000);

  Serial.println("Test cycle complete. Repeating...\n");
}































