#include "LSM6DS33.h"

/*
Relevant Pololu products:
#2738  LSM6DS33 + LIS3MDL Carrier (v5)
*/

// LSM6DS33 addresses:
#define DS33_SA0_HIGH_ADDRESS              0x6a
#define DS33_SA0_LOW_ADDRESS               0x6b

#define DS33_WHO_ID    0x69

LSM6DS33::LSM6DS33(const char * i2cDeviceName, int address) :
  i2c_gyro(i2cDeviceName), i2c_acc(i2cDeviceName)
{
    I2CBus i2c(i2cDeviceName);
    // Set the I2C addresses.
    i2c_acc.addressSet(address);
    i2c_gyro.addressSet(address);

    // Make sure we can actually read an accelerometer control register.
    // (This throws an exception if there is a problem.)
//    readAccReg(CTRL1_XL);
}

uint8_t LSM6DS33::readGyroReg(uint8_t reg)
{
    return i2c_gyro.readByte(reg);
}

uint8_t LSM6DS33::readAccReg(uint8_t reg)
{
    return i2c_acc.readByte(reg);
}

void LSM6DS33::writeGyroReg(uint8_t reg, uint8_t value)
{
    i2c_gyro.writeByte(reg, value);
}

void LSM6DS33::writeAccReg(uint8_t reg, uint8_t value)
{
    i2c_acc.writeByte(reg, value);
}

// Turns on the LSM6DS33's accelerometer and gyroscope and places them in normal
// mode.
void LSM6DS33::enable(void)
{
			// Accelerometer
		  // Sets accelerometer to 833 Hz, full-scale of +/- 4g, 400Hz filter BW
    	// corresponds to 0.122 mg/LSB (see page 15 of the LSM6DS33 datasheet)	
		writeAccReg(CTRL1_XL, 0b01111000);
			// Sets accelerometer to 833 Hz, full-scale of +/- 4g, 400Hz filter BW
			// corresponds to 0.122 mg/LSB (see page 15 of the LSM6DS33 datasheet)
    writeGyroReg(CTRL2_G, 0b01111000);
}

void LSM6DS33::readAcc(void)
{
    uint8_t block[6];
    i2c_acc.readBlock(OUTX_L_XL, sizeof(block), block);
    a[0] = (int16_t)(block[0] | block[1] << 8);
    a[1] = (int16_t)(block[2] | block[3] << 8);
    a[2] = (int16_t)(block[4] | block[5] << 8);
}

void LSM6DS33::readGyro(void)
{
    uint8_t block[6];
    i2c_gyro.readBlock(OUTX_L_G, sizeof(block), block);
    g[0] = (int16_t)(block[0] | block[1] << 8);
    g[1] = (int16_t)(block[2] | block[3] << 8);
    g[2] = (int16_t)(block[4] | block[5] << 8);
}

void LSM6DS33::read(void)
{
    readAcc();
    readGyro();
}
