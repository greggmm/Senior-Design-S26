#include <Arduino.h>

#define DIR_PIN 10
#define STEP_PIN 11

// Target speed settings
#define START_DELAY_US 2000    // slow start (500 PPS)
#define MIN_DELAY_US 1250      // ~800 PPS (high torque region)
#define RAMP_STEPS 1000        // acceleration length

void stepMotor(int delayTime);

void setup() {
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);

  digitalWrite(DIR_PIN, HIGH);   // set rotation direction
}

void loop() {

  // Accelerate
  for (int i = 0; i < RAMP_STEPS; i++) {
    int delayTime = map(i, 0, RAMP_STEPS, START_DELAY_US, MIN_DELAY_US);
    stepMotor(delayTime);
  }

  // Run at max torque speed
  while (true) {
    stepMotor(MIN_DELAY_US);
  }
}

void stepMotor(int delayTime) {
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(5);     // minimum pulse width
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(delayTime);
}