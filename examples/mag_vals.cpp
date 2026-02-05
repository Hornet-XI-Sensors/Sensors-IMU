#include <Wire.h>
#include "SdFat.h"
#include <Arduino.h>
#include "Adafruit9DOF.h"

#ifdef SDCARD_SS_PIN
const uint8_t SD_CS_PIN = SDCARD_SS_PIN;
#endif // SDCARD_SS_PIN
#define SPI_CLOCK SD_SCK_MHZ(50)
#define SD_CONFIG SdioConfig(FIFO_SDIO)

SdFs sd;
FsFile newFile;
Adafruit9DOF imu;

double prevTime = 0;
double curTime  = 0;
double diff     = 0;
double period   = 0;
double hz       = 0;

long i = 0;

// Set I2C bus to use: Wire, Wire1, etc.
#define WIRE Wire

void setup() {
    WIRE.begin();
    if (!sd.begin(SD_CONFIG)) {
      return;
    }
    imu.begin();
  newFile = sd.open("mag_vals.txt", FILE_WRITE);
}


void loop() {
    float ax, ay, az; // accelerometer
    float gx, gy, gz; // gyroscope
    float mx, my, mz; // magnetometer

    imu.readAll(ax, ay, az, gx, gy, gz, mx, my, mz);

    newFile.print(mx);
    newFile.print("\t");
    newFile.print(my);
    newFile.print("\t");
    newFile.println(mz);
  i++;

  if (!(i % 100))
  {
    newFile.flush();
    newFile.close();
    newFile = sd.open("mag_vals.txt", FILE_WRITE);
  }
}