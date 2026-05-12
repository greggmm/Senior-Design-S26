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
#define SERVO_MIN_ANGLE 0
#define SERVO_DRIVE_ANGLE 90
#define SERVO_CST_ANGLE 180
#define SERVO_STEP_DEGREES 3
#define SERVO_UPDATE_MS 40

// -------------------------
// L298N motor pins
// -------------------------
#define MOTOR_IN1 5
#define MOTOR_IN2 6
#define MOTOR_IN3 10
#define MOTOR_IN4 11

#define MOTOR_SPEED 255 // 0-255, 127 ~= 50%
#define CONTROL_BUFFER_SIZE 32
#define CONTROL_TIMEOUT_MS 500
#define DRIVE_INPUT_THRESHOLD 50
#define CAMERA_INPUT_THRESHOLD 50

Servo cameraServo;

char controlBuffer[CONTROL_BUFFER_SIZE];
byte controlIndex = 0;
unsigned long lastControlMs = 0;
unsigned long lastServoUpdateMs = 0;
int cameraAngle = SERVO_DRIVE_ANGLE;
bool cstButtonWasPressed = false;
bool cstViewActive = false;

void runCSTSequence();

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
    cameraAngle = SERVO_DRIVE_ANGLE;
    cameraServo.write(cameraAngle);
    delay(500);
}

void cameraCSTView()
{
    Serial.println("Camera: CST view");
    cameraAngle = SERVO_CST_ANGLE;
    cameraServo.write(cameraAngle);
    delay(500);
}

// -------------------------
// Wheel motor control
// -------------------------
void setMotorPair(int leftSpeed, int rightSpeed)
{
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);

    if (leftSpeed >= 0)
    {
        digitalWrite(MOTOR_IN1, leftSpeed > 0 ? HIGH : LOW);
        digitalWrite(MOTOR_IN2, LOW);
    }
    else
    {
        digitalWrite(MOTOR_IN1, LOW);
        digitalWrite(MOTOR_IN2, HIGH);
    }

    if (rightSpeed >= 0)
    {
        digitalWrite(MOTOR_IN3, LOW);
        digitalWrite(MOTOR_IN4, rightSpeed > 0 ? HIGH : LOW);
    }
    else
    {
        digitalWrite(MOTOR_IN3, HIGH);
        digitalWrite(MOTOR_IN4, LOW);
    }
}

void motorsForward()
{
    Serial.println("Drive: forward");
    setMotorPair(MOTOR_SPEED, MOTOR_SPEED);
}

void motorsReverse()
{
    Serial.println("Drive: reverse");
    setMotorPair(-MOTOR_SPEED, -MOTOR_SPEED);
}

void motorsLeft()
{
    Serial.println("Drive: left");
    setMotorPair(-MOTOR_SPEED, MOTOR_SPEED);
}

void motorsRight()
{
    Serial.println("Drive: right");
    setMotorPair(MOTOR_SPEED, -MOTOR_SPEED);
}

void motorsStop()
{
    setMotorPair(0, 0);
}

bool parseControlPacket(char *packet, int &throttle, int &turn, int &cameraYaw, unsigned int &flags)
{
    char *first = strtok(packet, ",");
    char *second = strtok(NULL, ",");
    char *third = strtok(NULL, ",");
    char *fourth = strtok(NULL, ",");

    if (first == NULL || second == NULL || third == NULL || fourth == NULL || strtok(NULL, ",") != NULL)
    {
        return false;
    }

    char *end = NULL;
    long parsedThrottle = strtol(first, &end, 10);
    if (end == first || *end != '\0' || parsedThrottle < -100 || parsedThrottle > 100)
    {
        return false;
    }

    end = NULL;
    long parsedTurn = strtol(second, &end, 10);
    if (end == second || *end != '\0' || parsedTurn < -100 || parsedTurn > 100)
    {
        return false;
    }

    end = NULL;
    long parsedCameraYaw = strtol(third, &end, 10);
    if (end == third || *end != '\0' || parsedCameraYaw < -100 || parsedCameraYaw > 100)
    {
        return false;
    }

    end = NULL;
    unsigned long parsedFlags = strtoul(fourth, &end, 10);
    if (end == fourth || *end != '\0' || parsedFlags > 255)
    {
        return false;
    }

    throttle = (int)parsedThrottle;
    turn = (int)parsedTurn;
    cameraYaw = (int)parsedCameraYaw;
    flags = (unsigned int)parsedFlags;

    return true;
}

void driveFromControl(int throttle, int turn)
{
    int throttleMagnitude = abs(throttle);
    int turnMagnitude = abs(turn);

    if (throttleMagnitude < DRIVE_INPUT_THRESHOLD && turnMagnitude < DRIVE_INPUT_THRESHOLD)
    {
        motorsStop();
        return;
    }

    if (throttleMagnitude >= turnMagnitude)
    {
        if (throttle > 0)
        {
            motorsForward();
        }
        else
        {
            motorsReverse();
        }
    }
    else if (turn > 0)
    {
        motorsRight();
    }
    else
    {
        motorsLeft();
    }
}

void updateCameraFromControl(int cameraYaw)
{
    if (cstViewActive || abs(cameraYaw) < CAMERA_INPUT_THRESHOLD)
    {
        return;
    }

    unsigned long now = millis();
    if (now - lastServoUpdateMs < SERVO_UPDATE_MS)
    {
        return;
    }

    lastServoUpdateMs = now;

    if (cameraYaw > 0)
    {
        cameraAngle += SERVO_STEP_DEGREES;
    }
    else
    {
        cameraAngle -= SERVO_STEP_DEGREES;
    }

    cameraAngle = constrain(cameraAngle, SERVO_MIN_ANGLE, SERVO_CST_ANGLE);
    cameraServo.write(cameraAngle);
}

void handleCSTButton(bool cstButtonPressed)
{
    if (!cstButtonPressed || cstButtonWasPressed)
    {
        cstButtonWasPressed = cstButtonPressed;
        return;
    }

    motorsStop();

    if (cstViewActive)
    {
        cstViewActive = false;
        cameraDriveView();
    }
    else
    {
        cstViewActive = true;
        runCSTSequence();
        cameraCSTView();
    }

    cstButtonWasPressed = cstButtonPressed;
}

void handleControlPacket(char *packet)
{
    int throttle = 0;
    int turn = 0;
    int cameraYaw = 0;
    unsigned int flags = 0;

    if (!parseControlPacket(packet, throttle, turn, cameraYaw, flags))
    {
        Serial.println("Invalid control packet");
        return;
    }

    lastControlMs = millis();

    Serial.print("Control packet: throttle=");
    Serial.print(throttle);
    Serial.print(" turn=");
    Serial.print(turn);
    Serial.print(" yaw=");
    Serial.print(cameraYaw);
    Serial.print(" flags=");
    Serial.println(flags);

    bool cstButtonPressed = (flags & 0x01) != 0;
    handleCSTButton(cstButtonPressed);

    if (!cstButtonPressed)
    {
        driveFromControl(throttle, turn);
        updateCameraFromControl(cameraYaw);
    }
}

void readControlSerial()
{
    while (Serial.available() > 0)
    {
        char incoming = Serial.read();

        if (incoming == '\r')
        {
            continue;
        }

        if (incoming == '\n')
        {
            controlBuffer[controlIndex] = '\0';
            if (controlIndex > 0)
            {
                handleControlPacket(controlBuffer);
            }
            controlIndex = 0;
            continue;
        }

        if (controlIndex < CONTROL_BUFFER_SIZE - 1)
        {
            controlBuffer[controlIndex++] = incoming;
        }
        else
        {
            controlIndex = 0;
            Serial.println("Control packet too long");
        }
    }

    if (lastControlMs != 0 && millis() - lastControlMs > CONTROL_TIMEOUT_MS)
    {
        motorsStop();
        lastControlMs = 0;
    }
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

    cameraCSTView();

    runPump();
    delay(500);

    runBreakerMotor();
    delay(500);

    runShakerMotor();
    delay(500);

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

    Serial.println("Ready for control packets: throttle,turn,yaw,flags");
}

// -------------------------
// Loop
// -------------------------
void loop()
{
    readControlSerial();
}
