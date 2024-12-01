#include <Arduino.h>
#include <Adafruit_AMG88xx.h>
#include <Wire.h>
#include <TFLI2C.h>

// AMG8833 thermal imager setup
Adafruit_AMG88xx amg;
const float temperatureThreshold = 45.0; // temp threshold in Celsius

// TF-Luna LIDAR setup
TFLI2C tflI2C;
int16_t tfDist;
int16_t tfAddr = TFL_DEF_ADR; // default I2C address for TF-Luna
const int distanceThreshold = 20; // distance threshold (cm)

// motor pins
const int dirA = 12, pwmA = 3, brakeA = 9;
const int dirB = 13, pwmB = 11, brakeB = 8;

// motor state
bool motorsRunning = false;

// cooldown timer variables
unsigned long lastStopTime = 0;
const unsigned long cooldownPeriod = 2000;

// motor timeout
unsigned long startTime = 0; 
const unsigned long maxRunTime = 60000; // max runtime (in ms)

// timestamps for detection
unsigned long detectionTime = 0;           // LIDAR detection time
unsigned long thermalDetectionTime = 0;    // thermal detection time

void setup() {
    Serial.begin(115200);
    Wire.begin();

    Serial.println("Initializing...");

    // init AMG8833
    if (!amg.begin()) {
        Serial.println("AMG8833 not found!");
        while (1);
    }

    // init motors
    pinMode(dirA, OUTPUT);
    pinMode(pwmA, OUTPUT);
    pinMode(brakeA, OUTPUT);
    pinMode(dirB, OUTPUT);
    pinMode(pwmB, OUTPUT);
    pinMode(brakeB, OUTPUT);

    digitalWrite(brakeA, LOW);
    digitalWrite(brakeB, LOW);

    Serial.println("Setup complete.");
}

void loop() {
    unsigned long currentTime = millis();

    // runtime debug log
    if (motorsRunning) {
        Serial.print("Elapsed runtime: ");
        Serial.println(currentTime - startTime);
    }

    // stop motors after max runtine
    if (motorsRunning && (currentTime - startTime >= maxRunTime)) {
        Serial.println("Maximum runtime reached. Stopping motors...");
        stopMotors();
        return;
    }

    // no checks during cooldown
    if ((currentTime - lastStopTime) < cooldownPeriod && !motorsRunning) {
        return;
    }

    // LIDAR distance check
    if (tflI2C.getData(tfDist, tfAddr)) {
        if (tfDist <= distanceThreshold) {
            if (!detectionTime) { 
                detectionTime = millis();
                Serial.println("Object detected!");
                Serial.print("Detection time: ");
                Serial.println(detectionTime);
            }

            if (motorsRunning) { // stop motors and log reaction time
                stopMotors();
                lastStopTime = currentTime; // begin cooldown
                unsigned long reactionTime = millis();
                Serial.print("Reaction Time (LIDAR): ");
                Serial.print(reactionTime - detectionTime);
                Serial.println(" ms");
                detectionTime = 0; // reset detection time for next event
            }
        } else {
            if (!motorsRunning) { // restart motors when object is removed
                Serial.println("Path clear. Motors running...");
                runMotors();
            }
        }
    }

    // Thermal imaging check
    float pixels[64];
    amg.readPixels(pixels);
    for (int i = 0; i < 64; i++) {
        if (pixels[i] >= temperatureThreshold) {
            if (!thermalDetectionTime) { 
                thermalDetectionTime = millis();
                Serial.println("High temperature detected!");
                Serial.print("Thermal detection time: ");
                Serial.println(thermalDetectionTime);
            }

            if (motorsRunning) { // stop motors and log reaction time
                stopMotors();
                lastStopTime = currentTime; // begin cooldown
                unsigned long reactionTime = millis();
                Serial.print("Reaction Time (Thermal): ");
                Serial.print(reactionTime - thermalDetectionTime);
                Serial.println(" ms");
                thermalDetectionTime = 0; // reset detection time for next event
            }
            return; 
        }
    }

    delay(100); // simulate additional processing time
}

void runMotors() {
    if (!motorsRunning) {
        analogWrite(pwmA, 200);
        digitalWrite(dirA, HIGH);
        analogWrite(pwmB, 200);
        digitalWrite(dirB, HIGH);

        // release brakes
        digitalWrite(brakeA, LOW);
        digitalWrite(brakeB, LOW);

        Serial.println("Motors running...");
        motorsRunning = true; 
        startTime = millis(); 
        Serial.print("Start time: ");
        Serial.println(startTime);
    }
}

void stopMotors() {
    if (motorsRunning) {
        analogWrite(pwmA, 0);
        digitalWrite(brakeA, HIGH);
        analogWrite(pwmB, 0);
        digitalWrite(brakeB, HIGH);

        Serial.println("Motors stopped.");
        motorsRunning = false; 
    }
}
