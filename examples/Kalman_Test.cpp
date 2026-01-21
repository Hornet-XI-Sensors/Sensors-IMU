#include <Arduino.h>
#include "Adafruit9DOF.h"
#include "Kalman.h"
#include <Wire.h>
#define WIRE Wire

Kalman kalmanX, kalmanY, kalmanYaw;
Adafruit9DOF fusion;

//Timer Variables
const unsigned long period = 8; // milliseconds (100Hz)
static unsigned long lastTime;

static unsigned long start_time;

//MPXH variables
int val;
float calc;
float scaledup;
float pres;

int i =0;
static const double EPS = 1e-8;

//IMU variables
float ax, ay, az;
float gx, gy, gz;   
float mx, my, mz;

float kalAngleX, kalAngleY, kalAngleYaw; // Calculated angles


float mag_offset_x, mag_offset_y, mag_offset_z;
float mag_scale_x, mag_scale_y, mag_scale_z;
float mag_avg_scale;

float roll, pitch, yaw;
float kalmanRoll, kalmanPitch;
float yaw_complementary = 0.0f;

// Velocity variables
static float vx = 0.0f, vy = 0.0f, vz = 0.0f;

// Angular velocity variables
static float wx = 0.0f, wy = 0.0f, wz = 0.0f;

// complementary filter coefficient (trust gyro this much)
const float YAW_ALPHA = 0.98f;



static float wrap180f(float a) {
    while (a > 180.0) a -= 360.0;
    while (a <= -180.0) a += 360.0;
    return a;
}

static float angle_diff(float to, float from) {
    float d = to - from;
    while (d > 180.0) d -= 360.0;
    while (d <= -180.0) d += 360.0;
    return d;
}

void setup() {
    Serial.begin(115200);
    WIRE.begin();
    // initialize IMU
    while (!Serial); // Wait for Serial to be ready

    if (!fusion.begin()) {
        Serial.println("Sensor init failed!");
        while (1);
    }
    Serial.println("Sensor init OK!");
    fusion.readAll(ax, ay, az, gx, gy, gz, mx, my, mz);
    
    double accXangle = atan2(ay, az) * RAD_TO_DEG;
    double accYangle = atan2(-ax, sqrt(ay*ay + az*az)) * RAD_TO_DEG;

    kalmanX.setAngle(accXangle);
    kalmanY.setAngle(accYangle);

    kalAngleX = accXangle;
    kalAngleY = accYangle;
    val = analogRead(A0); 
    calc = (val/1023.0) * (3.3);
    scaledup = calc * 1.5;
    pres = (scaledup+0.204)/0.0204;

    // initialize yaw from magnetometer (tilt compensated)
    {
        // read mag once (already read into mx,my,mz)
        // compute tilt-compensated heading
        float roll_rad = kalAngleX * DEG_TO_RAD;
        float pitch_rad = kalAngleY * DEG_TO_RAD;
        // tilt compensation
        float Xh = mx * cos(pitch_rad) + mz * sin(pitch_rad);
        float Yh = mx * sin(roll_rad) * sin(pitch_rad) + my * cos(roll_rad) - mz * sin(roll_rad) * cos(pitch_rad);
        float mag_heading = atan2(-Yh, Xh) * RAD_TO_DEG; // degrees
        mag_heading = wrap180f(mag_heading);
        yaw_complementary = mag_heading;
    }

    lastTime = millis();
    start_time = lastTime;
}

void loop() {

    if (millis() - start_time >= period) {
        fusion.readAll(ax, ay, az, gx, gy, gz, mx, my, mz);
        //Serial.println(millis() - start_time);
        unsigned long now = millis();
        double dt = (now - lastTime) / 1e3;
        lastTime = now;
    
        double accXangle = atan2(ay, az) * RAD_TO_DEG;
        double accYangle = atan2(-ax, sqrt(ay*ay + az*az)) * RAD_TO_DEG;

        double gyroXrate = gx; 
        double gyroYrate = gy;
        double gyroZrate = gz; // deg/s

        // Set angular velocities
        wx = gyroXrate;
        wy = gyroYrate;
        wz = gyroZrate;

        //Linear Velocities (simple integration) (Use Zero velocity update)
        vx += ax * dt;
        vy += ay * dt;
        vz += az * dt;

        kalAngleX = kalmanX.getAngle(accXangle, gyroXrate, dt);
        kalAngleY = kalmanY.getAngle(accYangle, gyroYrate, dt);

        // complementary filter for yaw
        // predict
        float pred = yaw_complementary + (float)(gyroZrate * dt);
        // compute mag heading (tilt-compensated) using current roll/pitch estimates
        float roll_rad = (float)kalAngleX * DEG_TO_RAD;
        float pitch_rad = (float)kalAngleY * DEG_TO_RAD;
        float Xh = mx * cos(pitch_rad) + mz * sin(pitch_rad);
        float Yh = mx * sin(roll_rad) * sin(pitch_rad) + my * cos(roll_rad) - mz * sin(roll_rad) * cos(pitch_rad);
        float mag_heading = atan2(-Yh, Xh) * RAD_TO_DEG;
        mag_heading = wrap180f(mag_heading);
        // correct on shortest angle difference
        float diff = angle_diff(mag_heading, pred);
        yaw_complementary = pred + (1.0f - YAW_ALPHA) * diff;
        yaw_complementary = wrap180f(yaw_complementary);

        Serial.print(kalAngleY); // Roll
        Serial.print("\t");
        Serial.print(kalAngleX); // Pitch
        Serial.print("\t");
        Serial.print(yaw_complementary);
        Serial.print("\t");
        
        // Linear accelerations
        Serial.print(ax);
        Serial.print("\t");
        Serial.print(ay);
        Serial.print("\t");
        Serial.print(az);
        Serial.print("\t");
        
        // Angular Velocities
        Serial.print(wx);
        Serial.print("\t");
        Serial.print(wy);
        Serial.print("\t");
        Serial.print(wz);

        //Linear Velocities
        Serial.print(vx);
        Serial.print("\t");
        Serial.print(vy);
        Serial.print("\t");
        Serial.println(vz);

        start_time = millis();
    };

    
}


