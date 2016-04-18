#include "vector.h"
#include "MinIMU9.h"
#include "exceptions.h"
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <wordexp.h>
#include <unistd.h>

MinIMU9::MinIMU9(const char * i2cDeviceName) :
  lsm6(i2cDeviceName), lis3(i2cDeviceName)
{
}

void MinIMU9::enable()
{
    // printf("Enabling IMU compass\n");
    lsm6.enable();
    // printf("Enabling IMU gyro\n");
    lis3.enable();
}

void MinIMU9::loadCalibration()
{
    wordexp_t expansion_result;
    wordexp("~/.minimu9-ahrs-cal", &expansion_result, 0);

    std::ifstream file(expansion_result.we_wordv[0]);
    if (file.fail())
    {
        throw posix_error("Failed to open calibration file ~/.minimu9-ahrs-cal.");
    }
    
    file >> mag_min(0) >> mag_max(0) >> mag_min(1) >> mag_max(1) >> mag_min(2) >> mag_max(2);
    if (file.fail() || file.bad())
    {
        throw std::runtime_error("Failed to parse calibration file ~/.minimu9-ahrs-cal.");
    }
    
}

void MinIMU9::measureOffsets()
{
    gyro_offset = vector::Zero();
    const int sampleCount = 32;
    for(int i = 0; i < sampleCount; i++)
    {
        lsm6.read();
        gyro_offset += vector_from_ints(&lsm6.g);
        usleep(20*1000);
    }
    gyro_offset /= sampleCount;
}

vector MinIMU9::readMag()
{
    lis3.read();
    IMU::raw_m = int_vector_from_ints(&lis3.m);
    
    vector v;
    v(0) = (float)(lis3.m[0] - mag_min(0)) / (mag_max(0) - mag_min(0)) * 2 - 1;
    v(1) = (float)(lis3.m[1] - mag_min(1)) / (mag_max(1) - mag_min(1)) * 2 - 1;
    v(2) = (float)(lis3.m[2] - mag_min(2)) / (mag_max(2) - mag_min(2)) * 2 - 1;
    return v;
}

vector MinIMU9::readAcc()
{
    // Info about linear acceleration sensitivity from datasheets:
    // LSM303DLM: at FS = 8 g, 3.9 mg/digit (12-bit reading)
    // LSM303DLHC: at FS = 8 g, 4 mg/digit (12-bit reading probably an approximation)
    // LSM303DLH: at FS = 8 g, 3.9 mg/digit (12-bit reading)
    // LSM303D: at FS = 8 g, 0.244 mg/LSB (16-bit reading)
    const float accel_scale = 0.000244;
    //const float accel_scale = 0.122f / 1000.0f; // 0.000244;

    lsm6.readAcc();
    IMU::raw_a = int_vector_from_ints(&lsm6.a);
    return vector_from_ints(&lsm6.a) * accel_scale;
}

vector MinIMU9::readGyro()
{
    // Info about sensitivity from datasheets:
    // L3G4200D: at FS = 2000 dps, 70 mdps/digit
    // L3GD20: at FS = 2000 dps, 70 mdps/digit
    // L3GD20H: at FS = 2000 dps, 70 mdps/digit
    const float gyro_scale = 0.07 * 3.14159265 / 180;
    // const float gyro_scale = 35.0f / 1000.0f;

    lsm6.readGyro();
    IMU::raw_g = int_vector_from_ints(&lsm6.g);
    return ( vector_from_ints(&lsm6.g) - gyro_offset ) * gyro_scale;
}
