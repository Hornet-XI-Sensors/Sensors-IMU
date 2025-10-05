#include "LSM6DS3.h"

// Register map (from LSM6DS3 datasheet)
#define WHO_AM_I        0x0F
#define WHO_AM_I_EXPECTED 0x6A
#define CTRL1_XL        0x10
#define CTRL2_G         0x11
#define OUTX_L_G        0x22
#define OUTX_L_XL       0x28

LSM6DS3::LSM6DS3(uint8_t address) {
    addr = address;
    wire = &Wire;
}

bool LSM6DS3::begin(TwoWire &wirePort) {
    wire = &wirePort;
    wire->begin();

    // Check WHO_AM_I register
    wire->beginTransmission(addr);
    wire->write(WHO_AM_I);
    wire->endTransmission(false);
    wire->requestFrom(addr, (uint8_t)1);

    if (!wire->available()) {
        Serial.println("LSM6DS3: WHO_AM_I not available");
        return false;
    }
    uint8_t id = wire->read();
    Serial.print("LSM6DS3: WHO_AM_I = 0x"); Serial.println(id, HEX);
    if (id != WHO_AM_I_EXPECTED) {
        Serial.println("LSM6DS3: Unexpected WHO_AM_I value");
        return false;
    }

    // Accelerometer: 104 Hz, ±2g, BW = 50 Hz
    // CTRL1_XL bits: ODR_XL[3:0]=0100 (104 Hz), FS_XL[1:0]=00 (±2g), BW_XL=00 (50 Hz)
    writeRegister(CTRL1_XL, 0b01000000);

    // Gyroscope: 104 Hz, ±2000 dps
    // CTRL2_G bits: ODR_G[3:0]=0100 (104 Hz), FS_G[1:0]=11 (±2000 dps)
    writeRegister(CTRL2_G, 0b01001100);

    Serial.println("LSM6DS3: Init OK");
    return true;
}

void LSM6DS3::readAccel(float &ax, float &ay, float &az) {
    uint8_t data[6];
    readRegisters(OUTX_L_XL, data, 6);
    Serial.print("LSM6DS3 Accel raw: ");
    for (int i = 0; i < 6; i++) {
        Serial.print(data[i], HEX); Serial.print(" ");
    }
    Serial.println();

    int16_t rawX = (int16_t)(data[1] << 8 | data[0]);
    int16_t rawY = (int16_t)(data[3] << 8 | data[2]);
    int16_t rawZ = (int16_t)(data[5] << 8 | data[4]);

    // ±2g range → 0.061 mg/LSB
    const float scale = 0.061f / 1000.0f * 9.80665f; // convert to m/s²
    ax = rawX * scale;
    ay = rawY * scale;
    az = rawZ * scale;
}

void LSM6DS3::readGyro(float &gx, float &gy, float &gz) {
    uint8_t data[6];
    readRegisters(OUTX_L_G, data, 6);
    Serial.print("LSM6DS3 Gyro raw: ");
    for (int i = 0; i < 6; i++) {
        Serial.print(data[i], HEX); Serial.print(" ");
    }
    Serial.println();

    int16_t rawX = (int16_t)(data[1] << 8 | data[0]);
    int16_t rawY = (int16_t)(data[3] << 8 | data[2]);
    int16_t rawZ = (int16_t)(data[5] << 8 | data[4]);

    // ±2000 dps range → 70 mdps/LSB
    const float scale = 70.0f / 1000.0f; // convert to deg/s
    gx = rawX * scale;
    gy = rawY * scale;
    gz = rawZ * scale;
}

void LSM6DS3::writeRegister(uint8_t reg, uint8_t value) {
    wire->beginTransmission(addr);
    wire->write(reg);
    wire->write(value);
    wire->endTransmission();
}

void LSM6DS3::readRegisters(uint8_t reg, uint8_t *buffer, uint8_t len) {
    wire->beginTransmission(addr);
    wire->write(reg);
    wire->endTransmission(false);
    wire->requestFrom(addr, len);
    for (uint8_t i = 0; i < len && wire->available(); i++) {
        buffer[i] = wire->read();
    }
}
