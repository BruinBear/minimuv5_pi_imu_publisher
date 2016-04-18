#ifndef MINIMU9_LIB
#define MINIMU9_LIB

#include "vector.h"
#include "MinIMU9.h"
#include "version.h"
#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include <system_error>
#include <boost/program_options.hpp>
#include <unistd.h>

#include "sensor_msgs/Imu.h"

#include "MinIMU9.h"

#define FLOAT_FORMAT std::fixed << std::setprecision(3) << std::setw(field_width)

std::ostream & operator << (std::ostream & os, const vector & vector);

std::ostream & operator << (std::ostream & os, const matrix & matrix);

std::ostream & operator << (std::ostream & os, const quaternion & quat);

typedef void rotation_output_function(quaternion& rotation);

void output_quaternion(quaternion & rotation);

void write_quaternion(sensor_msgs::Imu & msg, const quaternion & quat);

void output_matrix(quaternion & rotation);

void output_euler(quaternion & rotation);

int millis();

void streamRawValues(IMU& imu);

//! Uses the acceleration and magnetic field readings from the compass
// to get a noisy estimate of the current rotation matrix.
// This function is where we define the coordinate system we are using
// for the ground coords:  North, East, Down.
matrix rotationFromCompass(const vector& acceleration, const vector& magnetic_field);

typedef void fuse_function(quaternion& rotation, float dt, const vector& angular_velocity,
                  const vector& acceleration, const vector& magnetic_field);

void fuse_compass_only(quaternion& rotation, float dt, const vector& angular_velocity,
  const vector& acceleration, const vector& magnetic_field);

// Uses the given angular velocity and time interval to calculate
// a rotation and applies that rotation to the given quaternion.
// w is angular velocity in radians per second.
// dt is the time.
void rotate(quaternion& rotation, const vector& w, float dt);

void fuse_gyro_only(quaternion& rotation, float dt, const vector& angular_velocity,
  const vector& acceleration, const vector& magnetic_field);

void fuse_default(quaternion& rotation, float dt, const vector& angular_velocity,
  const vector& acceleration, const vector& magnetic_field);

void ahrs(IMU & imu, fuse_function * fuse, rotation_output_function * output);

#endif
