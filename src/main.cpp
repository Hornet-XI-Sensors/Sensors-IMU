#include <Arduino.h>
#include "Adafruit9DOF.h"

Adafruit9DOF fusion;

// Complementary filter variables
float pitch = 0, roll = 0;
const float alpha = 0.98; // filter constant
unsigned long lastTime = 0;

void setup() {
    Serial.begin(9800);
    while (!Serial); // Wait for Serial to be ready

    if (!fusion.begin()) {
        Serial.println("Sensor init failed!");
        while (1);
    }
    Serial.println("Sensor init OK!");
}

void loop() {
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;

    fusion.readAll(ax, ay, az, gx, gy, gz, mx, my, mz);

    Serial.print("Accel: ");
    Serial.print(ax); Serial.print(", ");
    Serial.print(ay); Serial.print(", ");
    Serial.print(az); Serial.print(" | ");

    Serial.print("Gyro: ");
    Serial.print(gx); Serial.print(", ");
    Serial.print(gy); Serial.print(", ");
    Serial.print(gz); Serial.print(" | ");

    Serial.print("Mag: ");
    Serial.print(mx); Serial.print(", ");
    Serial.print(my); Serial.print(", ");
    Serial.print(mz);
    Serial.println();

    delay(200);
}

