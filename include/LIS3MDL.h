#ifndef _LIS3MDL_h
#define _LIS3MDL_h

#include "I2CBus.h"

#define _LIS3MDL_REG_WHO_AM_I 0x0f
#define _LIS3MDL_VAL_WHO_AM_I 0x3d
#define _LIS3MDL_REG_CTRL_REG1 0x20
#define _LIS3MDL_REG_CTRL_REG2 0x21
#define _LIS3MDL_REG_CTRL_REG3 0x22
#define _LIS3MDL_REG_OUT_X_L 0x28

class LIS3MDL
{
public:
    LIS3MDL(const char * i2cDeviceName);

    // gyro angular velocity readings
    int m[3];

    void enable(void);

    void writeReg(uint8_t reg, uint8_t value);
    uint8_t readReg(uint8_t reg);
    void read();

private:
    I2CBus i2c;
};

#endif
