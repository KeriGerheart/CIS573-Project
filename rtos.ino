#include <Arduino_FreeRTOS.h>
#include <Adafruit_AMG88xx.h>
#include <TFLI2C.h>

// AMG8833 thermal imager setup
Adafruit_AMG88xx amg;
const float temperatureThreshold = 45.0; // temp threshold in Celsius

// TF-Luna LIDAR setup
TFLI2C tflI2C;
int16_t tfDist;
int16_t tfAddr = TFL_DEF_ADR; // default I2C address for TF-Luna
const int distanceThreshold = 20; // distance threshold (cm)
int16_t prevDistance = -1; // previous distance for logging

// motor pins
const int dirA = 12, pwmA = 3, brakeA = 9;
const int dirB = 13, pwmB = 11, brakeB = 8;

// shared flags
volatile bool fireDetected = false;
volatile bool objectDetected = false;
volatile bool motorsRunning = false;

// max run time (in ms)
const unsigned long maxRunTime = 60000;
unsigned long motorStartTime = 0;

// timestamps for detection
unsigned long lidarDetectionTime = 0;     // LIDAR detection time
unsigned long thermalDetectionTime = 0;  // thermal detection time

// task handles
TaskHandle_t motorTaskHandle;

// LIDAR detection task
void lidarTask(void *pvParameters) {
    while (true) {
        if (tflI2C.getData(tfDist, tfAddr)) {
            // Log significant distance changes
            if (abs(tfDist - prevDistance) >= 5) {
                Serial.print("Distance from LIDAR: ");
                Serial.print(tfDist);
                Serial.println(" cm");
                prevDistance = tfDist;
            }

            // update objectDetected flag on changes
            bool newObjectDetected = (tfDist <= distanceThreshold);
            if (newObjectDetected != objectDetected) {
                objectDetected = newObjectDetected;

                if (objectDetected) {
                    Serial.println("Object detected within range!");
                    lidarDetectionTime = millis(); // log detection time
                    Serial.print("LIDAR Detection Time: ");
                    Serial.println(lidarDetectionTime);
                } else {
                    Serial.println("Object no longer detected.");
                }
            }
        } else {
            Serial.println("Failed to read from LIDAR!");
        }
        vTaskDelay(pdMS_TO_TICKS(200)); // run every 200 ms
    }
}

// Thermal detection Task
void thermalTask(void *pvParameters) {
    float pixels[64];
    while (true) {
        amg.readPixels(pixels);
        bool newFireDetected = false;

        // check for high temp
        for (int i = 0; i < 64; i++) {
            if (pixels[i] >= temperatureThreshold) {
                newFireDetected = true;
                if (!fireDetected) {
                    Serial.println("High temperature detected!");
                    thermalDetectionTime = millis(); // log detection time
                    Serial.print("Thermal Detection Time: ");
                    Serial.println(thermalDetectionTime);
                }
                break;
            }
        }

        fireDetected = newFireDetected;

        vTaskDelay(pdMS_TO_TICKS(500)); // run every 500 ms
    }
}

// motor Control task
void motorTask(void *pvParameters) {
    while (true) {
        if (fireDetected || objectDetected) {
            if (motorsRunning) {
                Serial.println("Stopping motors due to obstacle or heat...");
                stopMotors();

                // log reaction time
                unsigned long reactionTime = millis();
                if (fireDetected) {
                    Serial.print("Reaction Time (Thermal): ");
                    Serial.print(reactionTime - thermalDetectionTime);
                    Serial.println(" ms");
                }
                if (objectDetected) {
                    Serial.print("Reaction Time (LIDAR): ");
                    Serial.print(reactionTime - lidarDetectionTime);
                    Serial.println(" ms");
                }
            }
        } else {
            if (!motorsRunning && (millis() - motorStartTime < maxRunTime)) {
                Serial.println("No obstacles. Motors running...");
                runMotors();
            } else if (millis() - motorStartTime >= maxRunTime) {
                Serial.println("Max runtime reached. Stopping motors...");
                stopMotors();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // run every 100 ms
    }
}

// motor control
void runMotors() {
    if (!motorsRunning) {
        analogWrite(pwmA, 200);
        digitalWrite(dirA, HIGH);
        analogWrite(pwmB, 200);
        digitalWrite(dirB, HIGH);

        digitalWrite(brakeA, LOW);
        digitalWrite(brakeB, LOW);

        motorsRunning = true;
        motorStartTime = millis();
        Serial.println("Motors running...");
    }
}

void stopMotors() {
    if (motorsRunning) {
        analogWrite(pwmA, 0);
        digitalWrite(brakeA, HIGH);
        analogWrite(pwmB, 0);
        digitalWrite(brakeB, HIGH);

        motorsRunning = false;
        Serial.println("Motors stopped.");
    }
}

// setup 
void setup() {
    Serial.begin(115200);
    Wire.begin();

    Serial.println("Initializing...");

    // init Thermal sensor
    if (!amg.begin()) {
        Serial.println("Failed to initialize AMG8833!");
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

    // create tasks
    xTaskCreate(lidarTask, "LIDAR Task", 256, NULL, 1, NULL);
    xTaskCreate(thermalTask, "Thermal Task", 256, NULL, 1, NULL);
    xTaskCreate(motorTask, "Motor Task", 256, NULL, 1, &motorTaskHandle);

    Serial.println("Setup complete. Starting FreeRTOS...");
    vTaskStartScheduler(); // start FreeRTOS
}

void loop() {
    // empty bc FreeRTOS handles task scheduling
}
