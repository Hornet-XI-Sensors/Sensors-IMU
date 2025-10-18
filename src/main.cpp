#include <Arduino.h>
#include "Adafruit9DOF.h"
#include <Wire.h>
Adafruit9DOF fusion;

// Complementary filter variables
float pitch = 0, roll = 0;
const float alpha = 0.80; // filter constant
unsigned long lastTime = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial); // Wait for Serial to be ready

    if (!fusion.begin()) {
        Serial.println("Sensor init failed!");
        while (1);
    }
    Serial.println("Sensor init OK!");
}

void loop() {
    float mx, my, mz;
    float ax,ay,az;
    float gx,gy,gz;
    fusion.readAll(ax, ay, az, gx, gy, gz, mx, my, mz);
    
    //Time Stamp
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;
    //Accel Pitch and Roll
    float pitchAcc = atan2(-ax, sqrt(ay*ay + az*az));
    float rollAcc  = atan2(ay, az);

    //Gyro Integration
    pitch = alpha * (pitch + gx * dt * DEG_TO_RAD) + (1 - alpha) * pitchAcc;
    roll  = alpha * (roll  + gy * dt * DEG_TO_RAD) + (1 - alpha) * rollAcc;

    float total = sqrt(mx*mx + my*my + mz*mz);


    /*Serial.print("Accel: "); Serial.print(ax); Serial.print(", "); Serial.print(ay); Serial.print(", "); Serial.print(az);
    Serial.print(" m/s^2 ");
    Serial.print("Gyro: "); Serial.print(gx); Serial.print(", "); Serial.print(gy); Serial.print(", "); Serial.print(gz);
    Serial.print(" dps ");*/
    Serial.print("Pitch: "); Serial.print(pitch * RAD_TO_DEG);
    Serial.print(" Roll: "); Serial.print(roll * RAD_TO_DEG);
    Serial.print("Mag: "); Serial.print(total);
    Serial.println(" uTesla ");

}

