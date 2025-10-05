#include "Adafruit9DOF.h"

Adafruit9DOF::Adafruit9DOF() : imu(0x6A), mag(0x1C) {}

bool Adafruit9DOF::begin() {
	bool ok1 = imu.begin();
	bool ok2 = mag.begin();
	return ok1 && ok2;
}

void Adafruit9DOF::readAll(float &ax, float &ay, float &az,
						   float &gx, float &gy, float &gz,
						   float &mx, float &my, float &mz) {
	imu.readAccel(ax, ay, az);
	imu.readGyro(gx, gy, gz);
	mag.readMag(mx, my, mz);
}
