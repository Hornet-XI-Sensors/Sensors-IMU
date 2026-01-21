#include <Arduino.h>
#include "Adafruit9DOF.h"
#include "EKF.h"
#include "MS5837.h"
#include "SdFat.h"
#include <Wire.h>

#ifdef SDCARD_SS_PIN
const uint8_t SD_CS_PIN = SDCARD_SS_PIN;
#endif // SDCARD_SS_PIN
#define SPI_CLOCK SD_SCK_MHZ(50)
#define SD_CONFIG SdioConfig(FIFO_SDIO)

SdFs sd;
FsFile newFile;

double prevTime = 0;
double curTime  = 0;
double diff     = 0;
//double hz       = 0;

long i = 0;

#define WIRE Wire

// create EKF object
EKF ekf;
Adafruit9DOF fusion;
MS5837 bar30;

int val;
float calc;
float scaledup;
float pres;


const unsigned long period = 10; // milliseconds (100Hz)
static unsigned long start_time = millis();

void setup() {
    Serial.begin(115200);
    // initialize IMU
    while (!Serial); // Wait for Serial to be ready

    WIRE.begin();

    if (!fusion.begin()) {
        Serial.println("Sensor init failed!");
        while (1);
    }
    Serial.println("Sensor init OK!");

    bar30.init();

    if (!sd.begin(SD_CONFIG)) {
      return;
    }

    newFile = sd.open("log.txt", FILE_WRITE);
    Serial.println("Bar30 init OK!");
    bar30.setFluidDensity(997); // freshwater
}

void loop() {
    static unsigned long now = millis(); // Reset start time each loop
    float gx, gy, gz;
    float ax, ay, az;
    float mx, my, mz;
    fusion.readAll(ax, ay, az, gx, gy, gz, mx, my, mz);
    if (millis() - start_time >= period) {
        // read depth
        bar30.read();
        float depth = bar30.depth();
        float pressure = bar30.pressure(); // mbar
        float temp = bar30.temperature(); // deg C

        // read internal hull pressure
        val = analogRead(A0); 
        calc = (val/1023.0) * (3.3);
        scaledup = calc * 1.5;
        pres = (scaledup+0.204)/0.0204;

        // convert to EKF vectors
        float gx_rad = gx * DEG_TO_RAD;
        float gy_rad = gy * DEG_TO_RAD;
        float gz_rad = gz * DEG_TO_RAD;

        float norm_acc = sqrt(ax*ax + ay*ay + az*az);
        if (norm_acc == 0){
            norm_acc = 1; // prevent division by zero
        } 
        float axn = ax / norm_acc;
        float ayn = ay / norm_acc;
        float azn = az / norm_acc;

        float roll_acc  = atan2(ayn, azn);
        float pitch_acc = atan2(-axn, sqrt(ayn*ayn + azn*azn));

        float Xh = mx * cos(pitch_acc) + mz * sin(pitch_acc);
        float Yh = mx * sin(roll_acc)*sin(pitch_acc) + my*cos(roll_acc) - mz*sin(roll_acc)*cos(pitch_acc);
        float yaw_mag = atan2(-Yh, Xh);

        EKF::StateVector u = {gx_rad, gy_rad, gz_rad, ax, ay, az};
        EKF::MeasVector z  = {roll_acc, pitch_acc, yaw_mag, 0, 0, 0};
        

        // update EKF
        float dt = 0.01f; 
        ekf.predict(u, dt);
        ekf.update(z);

        // get filtered orientation
        auto x = ekf.getState();
        for(int i=0;i<2;i++) Serial.print(x[i]*57.2958f,4), Serial.print("\t");

        //Linear Velocities
        Serial.print(x[6]);
        Serial.print("\t");
        Serial.print(x[7]);
        Serial.print("\t");
        Serial.print(x[8]);

        //Print Bar 30 Depth
        Serial.print("\t");
        Serial.print(depth);
        Serial.print("\t");
        Serial.println(pres); // internal hull pressure
        now = millis();
        start_time = now;
    };

    unsigned long timestamp = millis();
    prevTime = micros();
  
    newFile.print(timestamp);
    newFile.print("\t");
    newFile.print(ax);
    newFile.print("\t");
    newFile.print(ay);
    newFile.print("\t");
    newFile.print(az);
    newFile.print("\t");
    newFile.print(gx);
    newFile.print("\t");        
    newFile.print(gy);
    newFile.print("\t");
    newFile.println(gz);
    i++;

    if (!(i % 100)){
    newFile.flush();
    newFile.close();
    newFile = sd.open("log.txt", FILE_WRITE);
    }
}
