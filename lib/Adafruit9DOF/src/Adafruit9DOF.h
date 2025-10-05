#ifndef ADAFRUIT9DOF_H
#define ADAFRUIT9DOF_H

#include "sensors/LSM6DS3.h"
#include "sensors/LIS3MDL.h"

class Adafruit9DOF {
public:
	Adafruit9DOF();
	bool begin();
	void readAll(float &ax, float &ay, float &az,
				 float &gx, float &gy, float &gz,
				 float &mx, float &my, float &mz);

private:
	LSM6DS3 imu;
	LIS3MDL mag;
};

#endif
