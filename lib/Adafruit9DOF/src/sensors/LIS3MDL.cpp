#include "LIS3MDL.h"

// Register map (from LIS3MDL datasheet)
#define WHO_AM_I        0x0F
#define WHO_AM_I_EXPECTED 0x3D
#define CTRL_REG1       0x20
#define CTRL_REG2       0x21
#define CTRL_REG3       0x22
#define OUTX_L          0x28

LIS3MDL::LIS3MDL(uint8_t address) {
	addr = address;
	wire = &Wire;
}

bool LIS3MDL::begin(TwoWire &wirePort) {
	wire = &wirePort;
	wire->begin();

	// Check WHO_AM_I register
	wire->beginTransmission(addr);
	wire->write(WHO_AM_I);
	wire->endTransmission(false);
	wire->requestFrom(addr, (uint8_t)1);

	if (!wire->available()) {
		Serial.println("LIS3MDL: WHO_AM_I not available");
		return false;
	}
	uint8_t id = wire->read();
	Serial.print("LIS3MDL: WHO_AM_I = 0x"); Serial.println(id, HEX);
	if (id != WHO_AM_I_EXPECTED) {
		Serial.println("LIS3MDL: Unexpected WHO_AM_I value");
		return false;
	}

	// CTRL_REG1: Temp disabled, Ultra-high-performance, ODR = 80 Hz, FAST_ODR disabled
	writeRegister(CTRL_REG1, 0b11110000);
	// CTRL_REG2: Full scale ±4 gauss
	writeRegister(CTRL_REG2, 0b00000000);
	// CTRL_REG3: Continuous-conversion mode
	writeRegister(CTRL_REG3, 0b00000000);

	Serial.println("LIS3MDL: Init OK");
	return true;
}

void LIS3MDL::readMag(float &mx, float &my, float &mz) {
	uint8_t data[6];
	readRegisters(OUTX_L, data, 6);
	Serial.print("LIS3MDL Mag raw: ");
	for (int i = 0; i < 6; i++) {
		Serial.print(data[i], HEX); Serial.print(" ");
	}
	Serial.println();

	int16_t rawX = (int16_t)(data[1] << 8 | data[0]);
	int16_t rawY = (int16_t)(data[3] << 8 | data[2]);
	int16_t rawZ = (int16_t)(data[5] << 8 | data[4]);

	// ±4 gauss range → 0.14 mgauss/LSB
	const float scale = 0.14f; // mgauss per LSB
	mx = rawX * scale;
	my = rawY * scale;
	mz = rawZ * scale;
}

void LIS3MDL::writeRegister(uint8_t reg, uint8_t value) {
	wire->beginTransmission(addr);
	wire->write(reg);
	wire->write(value);
	wire->endTransmission();
}

void LIS3MDL::readRegisters(uint8_t reg, uint8_t *buffer, uint8_t len) {
	wire->beginTransmission(addr);
	wire->write(reg);
	wire->endTransmission(false);
	wire->requestFrom(addr, len);
	for (uint8_t i = 0; i < len && wire->available(); i++) {
		buffer[i] = wire->read();
	}
}
