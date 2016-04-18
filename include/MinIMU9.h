#ifndef _MINIMU9_H
#define _MINIMU9_H

#include "IMU.h"
#include "LSM6DS33.h"
#include "LIS3MDL.h"

class MinIMU9 : public IMU {
public:
    MinIMU9(const char * i2cDeviceName);

    LSM6DS33 lsm6;
    LIS3MDL lis3;

    virtual vector readAcc();
    virtual vector readMag();
    virtual vector readGyro();

    virtual void enable();
    virtual void loadCalibration();
    virtual void measureOffsets();
};

#endif
