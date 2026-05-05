#include <Arduino.h>
#include <Servo.h>

// -------------------------
// Breaker stepper U2
// -------------------------
#define BREAK_ENABLE 2
#define BREAK_DIR    12
#define BREAK_STEP   13

#define START_DELAY_US 2000
#define MIN_DELAY_US   1250
#define RAMP_STEPS     100
#define BREAK_RUN_STEPS 100
#define BREAK_RETURN_STEPS 500

// -------------------------
// Shaker stepper U1
// -------------------------
#define SHAKE_DIR    7
#define SHAKE_STEP   8

#define SHAKE_TIME_SEC       10
#define SHAKE_START_DELAY_US 5000
#define SHAKE_MIN_DELAY_US   600
#define SHAKE_RAMP_STEPS     1000
#define SHAKE_FORWARD        HIGH

// -------------------------
// Pump
// -------------------------
#define PUMP_PIN     4
#define PUMP_TIME_MS 5000

// -------------------------
// Camera servo
// -------------------------
#define SERVO_PIN 3
#define SERVO_FB  A5

// -------------------------
// L298N motor pins
// -------------------------
#define MOTOR_IN1 5
#define MOTOR_IN2 6
#define MOTOR_IN3 10
#define MOTOR_IN4 11

#define MOTOR_SPEED 255 // 0-255, 127 ~= 50%

Servo cameraServo;

// -------------------------
// Step pulse helper
// -------------------------
void stepPulse(int stepPin, int lowDelayUs)
{
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(lowDelayUs);
}

// -------------------------
// Breaker enable control
// -------------------------
void enableBreakerMotor()
{
    digitalWrite(BREAK_ENABLE, LOW);
    delay(5);
}

void disableBreakerMotor()
{
    digitalWrite(BREAK_ENABLE, HIGH);
    delay(5);
}

// -------------------------
// Pump
// -------------------------
void runPump()
{
    Serial.println("Running pump...");
    digitalWrite(PUMP_PIN, HIGH);
    delay(PUMP_TIME_MS);
    digitalWrite(PUMP_PIN, LOW);
}

// -------------------------
// Breaker motor (U2)
// -------------------------
void runBreakerMotor()
{
    Serial.println("Breaking tube...");

    enableBreakerMotor();

    digitalWrite(BREAK_DIR, HIGH);

    // Ramp up
    for (int i = 0; i < RAMP_STEPS; i++)
    {
        int delayTime = map(i, 0, RAMP_STEPS, START_DELAY_US, MIN_DELAY_US);
        stepPulse(BREAK_STEP, delayTime);
    }

    // Run
    for (int i = 0; i < BREAK_RUN_STEPS; i++)
    {
        stepPulse(BREAK_STEP, MIN_DELAY_US);
    }

    // Slight reverse
    digitalWrite(BREAK_DIR, LOW);

    for (int i = 0; i < BREAK_RETURN_STEPS; i++)
    {
        stepPulse(BREAK_STEP, MIN_DELAY_US);
    }

    disableBreakerMotor();
}

// -------------------------
// Shaker motor (U1)
// -------------------------
void runShakerMotor()
{
    Serial.println("Shaking tube...");

    digitalWrite(SHAKE_DIR, SHAKE_FORWARD);

    // Ramp up
    for (int i = 0; i < SHAKE_RAMP_STEPS; i++)
    {
        int delayTime = map(i, 0, SHAKE_RAMP_STEPS,
                            SHAKE_START_DELAY_US, SHAKE_MIN_DELAY_US);
        stepPulse(SHAKE_STEP, delayTime);
    }

    // Run for time
    unsigned long endTime = millis() + (1000UL * SHAKE_TIME_SEC);

    while (millis() < endTime)
    {
        stepPulse(SHAKE_STEP, SHAKE_MIN_DELAY_US);
    }
}

// -------------------------
// Camera positions
// -------------------------
void cameraDriveView()
{
    Serial.println("Camera: drive view");
    cameraServo.write(90);
    delay(500);
}

void cameraCSTView()
{
    Serial.println("Camera: CST view");
    cameraServo.write(0);
    delay(500);
}

// -------------------------
// Wheel motor control
// -------------------------
void motorsForward()
{
    digitalWrite(MOTOR_IN1, MOTOR_SPEED);
    analogWrite(MOTOR_IN2, LOW);
    digitalWrite(MOTOR_IN3, LOW);
    analogWrite(MOTOR_IN4, MOTOR_SPEED);
}

void motorsReverse()
{
    digitalWrite(MOTOR_IN1, LOW);
    analogWrite(MOTOR_IN2, MOTOR_SPEED);
    digitalWrite(MOTOR_IN3, MOTOR_SPEED);
    analogWrite(MOTOR_IN4, LOW);
}

void motorsStop()
{
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, LOW);
    digitalWrite(MOTOR_IN3, LOW);
    digitalWrite(MOTOR_IN4, LOW);
}

// -------------------------
// Test functions
// -------------------------
void testWheelMotors()
{
    Serial.println("Testing wheel motors...");

    motorsForward();
    delay(3000);

    motorsReverse();
    delay(3000);

    motorsStop();
}

void testCameraServo()
{
    Serial.println("Testing camera servo...");

    cameraServo.write(0);
    delay(1000);

    cameraServo.write(180);
    delay(1000);

    cameraServo.write(90);
    delay(1000);

    int feedback = analogRead(SERVO_FB);
    Serial.print("Servo feedback A5: ");
    Serial.println(feedback);
}

// -------------------------
// Full CST sequence
// -------------------------
void runCSTSequence()
{
    Serial.println("Starting CST sequence...");

    cameraDriveView();

    runPump();
    delay(500);

    runBreakerMotor();
    delay(500);

    runShakerMotor();
    delay(500);

    cameraCSTView();

    Serial.println("CST sequence complete.");
}

// -------------------------
// Setup
// -------------------------
void setup()
{
    Serial.begin(115200);

    pinMode(BREAK_ENABLE, OUTPUT);
    pinMode(BREAK_DIR, OUTPUT);
    pinMode(BREAK_STEP, OUTPUT);

    pinMode(SHAKE_DIR, OUTPUT);
    pinMode(SHAKE_STEP, OUTPUT);

    pinMode(PUMP_PIN, OUTPUT);

    pinMode(MOTOR_IN1, OUTPUT);
    pinMode(MOTOR_IN2, OUTPUT);
    pinMode(MOTOR_IN3, OUTPUT);
    pinMode(MOTOR_IN4, OUTPUT);

    digitalWrite(PUMP_PIN, LOW);
    digitalWrite(BREAK_STEP, LOW);
    digitalWrite(SHAKE_STEP, LOW);

    disableBreakerMotor();
    motorsStop();

    cameraServo.attach(SERVO_PIN);
    cameraDriveView();

    Serial.println("System Test Starting...");

    testWheelMotors();
    delay(1000);

    testCameraServo();
    delay(1000);

    // runCSTSequence();

    Serial.println("All tests complete.");
}

// -------------------------
// Loop
// -------------------------
void loop()
{
    delay(1000);
}