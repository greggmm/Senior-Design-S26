#include <Arduino.h>
#include <Servo.h>

// Stepper pins
#define DIR1 10
#define STEP1 11

#define DIR2 12
#define STEP2 13

// Pump
#define PUMP_PIN 9

// L298N motor pins
#define IN1 4
#define IN2 5
#define IN3 6
#define IN4 7

// Servo
#define SERVO_PIN 8
#define SERVO_FB A0

Servo testServo;

int stepDelay = 800; // microseconds

void stepMotor(int stepPin, int dirPin, bool dir, int steps)
{
    digitalWrite(dirPin, dir);

    for (int i = 0; i < steps; i++)
    {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(stepDelay);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(stepDelay);
    }
}

void setup()
{
    Serial.begin(115200);

    pinMode(DIR1, OUTPUT);
    pinMode(STEP1, OUTPUT);

    pinMode(DIR2, OUTPUT);
    pinMode(STEP2, OUTPUT);

    pinMode(PUMP_PIN, OUTPUT);

    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    testServo.attach(SERVO_PIN);

    Serial.println("System Test Starting...");
}

void loop()
{
    Serial.println("---- Stepper 1 Test ----");

    stepMotor(STEP1, DIR1, HIGH, 400);
    delay(500);
    stepMotor(STEP1, DIR1, LOW, 400);

    delay(1000);

    Serial.println("---- Stepper 2 Test ----");

    stepMotor(STEP2, DIR2, HIGH, 400);
    delay(500);
    stepMotor(STEP2, DIR2, LOW, 400);

    delay(1000);

    Serial.println("---- Pump Test ----");

    digitalWrite(PUMP_PIN, HIGH);
    delay(3000);
    digitalWrite(PUMP_PIN, LOW);

    delay(1000);

    Serial.println("---- L298N Motor Test ----");

    // Forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    delay(3000);

    // Reverse
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    delay(3000);

    // Stop
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);

    delay(1000);

    Serial.println("---- Servo Direction Test ----");

    Serial.println("Move to 0°");
    testServo.write(0);
    delay(1000);

    Serial.println("Move to 180°");
    testServo.write(180);
    delay(1000);

    Serial.println("Move to 90° (center)");
    testServo.write(90);
    delay(1000);

    int feedback = analogRead(SERVO_FB);
    Serial.print("Servo Feedback: ");
    Serial.println(feedback);

    Serial.println("---- Test Cycle Complete ----");

    delay(5000);
}