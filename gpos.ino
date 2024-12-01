#include <Arduino.h>
#include <Adafruit_AMG88xx.h>
#include <Wire.h>
#include <TFLI2C.h>

// AMG8833 thermal imager setup
Adafruit_AMG88xx amg;
const float temperatureThreshold = 45.0;

// TF-Luna LIDAR setup
TFLI2C tflI2C;
int16_t tfDist;
int16_t tfAddr = TFL_DEF_ADR; // default I2C address for TF-Luna
const int distanceThreshold = 20;

// motor pins
const int dirA = 12, pwmA = 3, brakeA = 9;
const int dirB = 13, pwmB = 11, brakeB = 8;

// timestamp variables
unsigned long lidarDetectionTime = 0;
unsigned long thermalDetectionTime = 0;
unsigned long motorStopTime = 0;

// motor state
bool motorsRunning = false;

// flags for conditions
bool fireDetected = false;
bool objectDetected = false;

void setup() {
    Serial.begin(115200);
    delay(2000); // delay for serial monitor
    Wire.begin();

    Serial.println("Initializing...");

    if (!amg.begin()) {
        Serial.println("AMG8833 not found!");
        while (1);
    }

    pinMode(dirA, OUTPUT);
    pinMode(pwmA, OUTPUT);
    pinMode(brakeA, OUTPUT);
    pinMode(dirB, OUTPUT);
    pinMode(pwmB, OUTPUT);
    pinMode(brakeB, OUTPUT);

    Serial.println("Setup complete.");
}

void loop() {
    unsigned long currentMillis = millis();

    // LIDAR detection
    if (tflI2C.getData(tfDist, tfAddr)) {
        bool newObjectDetected = (tfDist <= distanceThreshold);

        // record detection time when an object is first detected
        if (newObjectDetected && !objectDetected) {
            objectDetected = true;
            lidarDetectionTime = currentMillis;
            Serial.println("Object detected!");
            Serial.print("LIDAR Detection Time: ");
            Serial.println(lidarDetectionTime);
        }

        // clear object detection when it is removed
        if (!newObjectDetected && objectDetected) {
            objectDetected = false;
            Serial.println("Object cleared.");
        }
    } else {
        Serial.println("Failed to read from LIDAR!");
    }

    // thermal Detection
    float pixels[64];
    amg.readPixels(pixels);
    bool newFireDetected = false;

    for (int i = 0; i < 64; i++) {
        if (pixels[i] >= temperatureThreshold) {
            newFireDetected = true;

            // record detection time when fire is first detected
            if (!fireDetected) {
                fireDetected = true;
                thermalDetectionTime = currentMillis;
                Serial.println("High temperature detected!");
                Serial.print("Thermal Detection Time: ");
                Serial.println(thermalDetectionTime);
            }
            break;
        }
    }

    // clear fire detection when temperature normalizes
    if (!newFireDetected && fireDetected) {
        fireDetected = false;
        Serial.println("Temperature normalized.");
    }

    // motor Task
    if (!fireDetected && !objectDetected) {
        if (!motorsRunning) {
            runMotors();
        }
    } else if (motorsRunning) {
        // stop motors and calculate reaction time 
        stopMotors();
        motorStopTime = millis(); // record actual motor stop time

        // calculate reaction times when motors stop
        if (fireDetected && thermalDetectionTime > 0) {
            Serial.print("Reaction Time (Thermal): ");
            Serial.print(motorStopTime - thermalDetectionTime);
            Serial.println(" ms");
            thermalDetectionTime = 0; // reset to avoid duplicate logs
        }

        if (objectDetected && lidarDetectionTime > 0) {
            Serial.print("Reaction Time (LIDAR): ");
            Serial.print(motorStopTime - lidarDetectionTime);
            Serial.println(" ms");
            lidarDetectionTime = 0; // reset to avoid duplicate logs
        }
    }

    delay(100); // simulate additional processing time
}

// motor control
void runMotors() {
    analogWrite(pwmA, 200);
    digitalWrite(dirA, HIGH);
    analogWrite(pwmB, 200);
    digitalWrite(dirB, HIGH);

    digitalWrite(brakeA, LOW);
    digitalWrite(brakeB, LOW);

    if (!motorsRunning) {
        motorsRunning = true;
        Serial.println("Motors running...");
    }
}

void stopMotors() {
    analogWrite(pwmA, 0);
    digitalWrite(brakeA, HIGH);
    analogWrite(pwmB, 0);
    digitalWrite(brakeB, HIGH);

    if (motorsRunning) {
        motorsRunning = false;
        Serial.println("Motors stopped.");
    }
}
