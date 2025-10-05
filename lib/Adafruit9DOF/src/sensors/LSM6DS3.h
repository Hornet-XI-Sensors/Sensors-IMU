#ifndef LSM6DS3_H
#define LSM6DS3_H

#include <Arduino.h>
#include <Wire.h>

class LSM6DS3 {
public:
    LSM6DS3(uint8_t address = 0x6A); // Default I2C address for LSM

    bool begin(TwoWire &wirePort = Wire);

    void readAccel(float &ax, float &ay, float &az);
    void readGyro(float &gx, float &gy, float &gz);

private:
    TwoWire *wire;
    uint8_t addr;

    // Helper functions
    void writeRegister(uint8_t reg, uint8_t value);
    void readRegisters(uint8_t reg, uint8_t *buffer, uint8_t len);
};

#endif
