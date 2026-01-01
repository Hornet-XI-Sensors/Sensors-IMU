#include <Arduino.h>
#include <Wire.h>
#include "Adafruit9DOF.h"
#include "MadgwickAHRS.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Create sensor object
Adafruit9DOF imu;

// Timing for 50Hz updates
const unsigned long period = 20; // milliseconds (50Hz)
static unsigned long last_update = 0;

// Function to convert quaternion to Euler angles (roll, pitch, yaw)
void quaternionToEuler(float q0, float q1, float q2, float q3, float& roll, float& pitch, float& yaw) {
    // Roll (X-axis rotation)
    roll = atan2(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));

    // Pitch (Y-axis rotation)
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (sinp >= 1.0f) {
        pitch = M_PI / 2.0f;
    } else if (sinp <= -1.0f) {
        pitch = -M_PI / 2.0f;
    } else {
        pitch = asin(sinp);
    }

    // Yaw (Z-axis rotation)
    yaw = atan2(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));

    // Convert to degrees
    roll *= 180.0f / M_PI;
    pitch *= 180.0f / M_PI;
    yaw *= 180.0f / M_PI;
}

void setup() {
    Serial.begin(115200);
    while (!Serial); // Wait for Serial to be ready

    Wire.begin();

    if (!imu.begin()) {
        Serial.println("Adafruit 9DOF initialization failed!");
        while (1);
    }

    Serial.println("Adafruit 9DOF initialized successfully!");
    Serial.println("Roll\tPitch\tYaw");
}

void loop() {
    if (millis() - last_update >= period) {
        // Read sensor data
        float ax, ay, az; // accelerometer
        float gx, gy, gz; // gyroscope
        float mx, my, mz; // magnetometer

        imu.readAll(ax, ay, az, gx, gy, gz, mx, my, mz);

        // Convert gyroscope from degrees/s to radians/s for Madgwick
        float gx_rad = gx * DEG_TO_RAD;
        float gy_rad = gy * DEG_TO_RAD;
        float gz_rad = gz * DEG_TO_RAD;

        // Update Madgwick AHRS algorithm
        MadgwickAHRSupdate(gx_rad, gy_rad, gz_rad, ax, ay, az, mx, my, mz);

        // Convert quaternion to Euler angles
        float roll, pitch, yaw;
        quaternionToEuler(q0, q1, q2, q3, roll, pitch, yaw);

        // Print the corrected Roll, Pitch, Yaw values
        Serial.print(roll, 4);
        Serial.print("\t");
        Serial.print(pitch, 4);
        Serial.print("\t");
        Serial.println(yaw, 4);

        last_update = millis();
    }
}
