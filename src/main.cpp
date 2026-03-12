#include <Arduino.h>
#include <Servo.h>

// -------------------------
// Breaker stepper (your proven tube-breaking motor)
// -------------------------
#define BREAK_DIR   10
#define BREAK_STEP  11

#define START_DELAY_US 2000
#define MIN_DELAY_US   1250
#define RAMP_STEPS     100
#define BREAK_RUN_STEPS 2000   // adjust as needed

// -------------------------
// Shaker stepper (new pins after breadboarding)
// -------------------------
#define SHAKE_DIR   12
#define SHAKE_STEP  13

#define SHAKE_TIME_SEC       10
#define SHAKE_START_DELAY_US 5000   // slower start
#define SHAKE_MIN_DELAY_US   600    // approx upper speed region, adjust if too fast
#define SHAKE_RAMP_STEPS     1000
#define SHAKE_FORWARD        HIGH   // change if direction is wrong

// -------------------------
// Pump
// -------------------------
#define PUMP_PIN     9
#define PUMP_TIME_MS 5000

// -------------------------
// Optional servo
// -------------------------
#define SERVO_PIN 8
Servo testServo;

// -------------------------
// Helper: one step pulse
// -------------------------
void stepPulse(int stepPin, int lowDelayUs)
{
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(5);   // step pulse width
    digitalWrite(stepPin, LOW);
    delayMicroseconds(lowDelayUs);
}

// -------------------------
// Pump: same style as original
// -------------------------
void runPump()
{
    digitalWrite(PUMP_PIN, HIGH);
    delay(PUMP_TIME_MS);
    digitalWrite(PUMP_PIN, LOW);
}

// -------------------------
// Breaker motor
// Based on the code that successfully broke the tube
// -------------------------
void runBreakerMotor()
{
    digitalWrite(BREAK_DIR, HIGH);

    // accelerate
    for (int i = 0; i < RAMP_STEPS; i++)
    {
        int delayTime = map(i, 0, RAMP_STEPS, START_DELAY_US, MIN_DELAY_US);
        stepPulse(BREAK_STEP, delayTime);
    }

    // continue at working speed
    for (int i = 0; i < BREAK_RUN_STEPS; i++)
    {
        stepPulse(BREAK_STEP, MIN_DELAY_US);
    }

    digitalWrite(BREAK_DIR, LOW);

    for (int i = 0; i < 500; i++)
    {
        stepPulse(BREAK_STEP, MIN_DELAY_US);
    }
}

// -------------------------
// Shaker motor
// Software version of the original idea:
// ramp up, run for SHAKE_TIME_SEC, then stop
// -------------------------
void runShakerMotor()
{
    digitalWrite(SHAKE_DIR, SHAKE_FORWARD);

    // ramp up like original shake() behavior
    for (int i = 0; i < SHAKE_RAMP_STEPS; i++)
    {
        int delayTime = map(i, 0, SHAKE_RAMP_STEPS,
                            SHAKE_START_DELAY_US, SHAKE_MIN_DELAY_US);
        stepPulse(SHAKE_STEP, delayTime);
    }

    // run at full shake speed for fixed time
    unsigned long endTime = millis() + (1000UL * SHAKE_TIME_SEC);
    while (millis() < endTime)
    {
        stepPulse(SHAKE_STEP, SHAKE_MIN_DELAY_US);
    }
}

void setup()
{
    Serial.begin(115200);

    pinMode(BREAK_DIR, OUTPUT);
    pinMode(BREAK_STEP, OUTPUT);

    pinMode(SHAKE_DIR, OUTPUT);
    pinMode(SHAKE_STEP, OUTPUT);

    pinMode(PUMP_PIN, OUTPUT);

    digitalWrite(PUMP_PIN, LOW);
    digitalWrite(BREAK_STEP, LOW);
    digitalWrite(SHAKE_STEP, LOW);

    testServo.attach(SERVO_PIN);   // attached but unused for now

    delay(10);

    Serial.println("System Test Starting...");

    // Run once immediately on power-up, like original code
    runPump();
    delay(500);

    runBreakerMotor();
    delay(500);

    runShakerMotor();

    Serial.println("Sequence Complete.");
}

void loop()
{
    // idle forever, same idea as original code
    while (1)
    {
        delay(1000);
    }
}