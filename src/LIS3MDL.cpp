#include "LIS3MDL.h"
#include <stdexcept>

// Magnetometer

LIS3MDL::LIS3MDL(const char * i2cDeviceName) : i2c(i2cDeviceName)
{
    i2c.addressSet(0x1e);
}	

// Turns on the gyro and places it in normal mode.
void LIS3MDL::enable()
{
    // printf("Enabling LIS3MDL\n");
    /* uint8_t buf_whoami = readReg(_LIS3MDL_REG_WHO_AM_I);
    if (buf_whoami != _LIS3MDL_VAL_WHO_AM_I) {
        throw std::runtime_error("Could not detect gyro.");
    } */



    // Disable temperature sensor
    // Enable FAST_ODR mode
    // Select ultra-high-performance mode (155 Hz)
    // Disable self-test
    writeReg(_LIS3MDL_REG_CTRL_REG1, 0b01110010);

    // Set full-scale to +/- 4 gauss
    // corresponds to 6842 gauss/LSB (see page 8 of the LIS3MDL datasheet)
    // Operate in normal mode as opposed to rebooting memory content
    // Don't call the soft reset function
    writeReg(_LIS3MDL_REG_CTRL_REG2, 0b00000000);

    // Low-power mode off
    // Set SPI bus to default 4-wire interface
    // Power on device and enable continuous-conversion mode
    writeReg(_LIS3MDL_REG_CTRL_REG3, 0b00000000);
}

void LIS3MDL::writeReg(uint8_t reg, uint8_t value)
{
    i2c.writeByte(reg, value);
}

uint8_t LIS3MDL::readReg(uint8_t reg)
{
    return i2c.readByte(reg);
}

void LIS3MDL::read()
{
    uint8_t block[6];
    i2c.readBlock(0x80 | _LIS3MDL_REG_OUT_X_L, sizeof(block), block);

    m[0] = (int16_t)(block[1] << 8 | block[0]);
    m[1] = (int16_t)(block[3] << 8 | block[2]);
    m[2] = (int16_t)(block[5] << 8 | block[4]);
}
