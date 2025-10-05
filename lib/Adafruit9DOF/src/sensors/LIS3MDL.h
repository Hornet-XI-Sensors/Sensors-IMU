#ifndef LIS3MDL_H
#define LIS3MDL_H

#include <Arduino.h>
#include <Wire.h>

class LIS3MDL {
public:
	LIS3MDL(uint8_t address = 0x1C); // Default I2C address for LIS3MDL

	bool begin(TwoWire &wirePort = Wire);

	void readMag(float &mx, float &my, float &mz);

private:
	TwoWire *wire;
	uint8_t addr;

	void writeRegister(uint8_t reg, uint8_t value);
	void readRegisters(uint8_t reg, uint8_t *buffer, uint8_t len);
};

#endif
