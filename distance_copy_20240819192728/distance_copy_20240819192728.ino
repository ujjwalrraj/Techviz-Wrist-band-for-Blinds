#include <Wire.h>
#include "Adafruit_VL53L1X.h"

// Define I2C pins for ESP32
#define I2C_SDA 21
#define I2C_SCL 22

// Define XSHUT and IRQ pins if needed
#define IRQ_PIN -1   // Set to -1 if not used
#define XSHUT_PIN -1 // Set to -1 if not used

// Define buzzer pin
#define BUZZER_PIN 4

Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN);

// Threshold distance below which the buzzer should start beeping
#define DISTANCE_THRESHOLD 1000 // in mm

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println(F("Adafruit VL53L1X sensor demo"));

  Wire.begin(I2C_SDA, I2C_SCL);  // Initialize I2C with specified pins
  if (!vl53.begin(0x29, &Wire)) {
    Serial.print(F("Error on init of VL sensor: "));
    Serial.println(vl53.vl_status);
    while (1) delay(10);
  }
  Serial.println(F("VL53L1X sensor OK!"));

  Serial.print(F("Sensor ID: 0x"));
  Serial.println(vl53.sensorID(), HEX);

  if (!vl53.startRanging()) {
    Serial.print(F("Couldn't start ranging: "));
    Serial.println(vl53.vl_status);
    while (1) delay(10);
  }
  Serial.println(F("Ranging started"));

  // Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500ms!
  vl53.setTimingBudget(50);
  Serial.print(F("Timing budget (ms): "));
  Serial.println(vl53.getTimingBudget());

  pinMode(BUZZER_PIN, OUTPUT); // Initialize buzzer pin
}

void loop() {
  if (vl53.dataReady()) {
    // New measurement is ready
    int16_t distance = vl53.distance();
    if (distance == -1) {
      // Something went wrong!
      Serial.print(F("Couldn't get distance: "));
      Serial.println(vl53.vl_status);
      return;
    }
    
    Serial.print(F("Distance: "));
    Serial.print(distance);
    Serial.println(" mm");

    if (distance < DISTANCE_THRESHOLD) {
      // Map the distance to a buzzer intensity
      // The closer the object, the higher the intensity
      int buzzerIntensity = map(distance, 0, DISTANCE_THRESHOLD, 255, 0);
      buzzerIntensity = constrain(buzzerIntensity, 0, 255); // Ensure the value is within range

      analogWrite(BUZZER_PIN, buzzerIntensity); // Set the buzzer intensity
    } else {
      analogWrite(BUZZER_PIN, 0); // Turn off the buzzer if distance is above the threshold
    }

    // Clear interrupt flag
    vl53.clearInterrupt();
  }

  delay(100); // Small delay to avoid flooding the serial output
}