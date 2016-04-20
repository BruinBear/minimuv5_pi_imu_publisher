#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include <tf/transform_broadcaster.h>

#include <sstream>
#include <time.h>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>


#include "minimu9-ahrs.h"

std::string imu_name;
std::string topic;

bool enableMuxer(char mask) {
  int adapter_nr = 1;
  int fd;

  char filename[20];
  snprintf(filename, 19, "/dev/i2c-%d", adapter_nr);
  fd = open(filename, O_RDWR);
	printf("Enabling Mutiplexer\n");
  printf("ioctl = %d\n", ioctl(fd, I2C_SLAVE, 0x70));
  printf("buf = %02x\n", mask);
  printf("status = %d\n", write(fd, &mask, 1));
}

void broadcastTF(quaternion quat) {
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transorm.setOrigin(tf::Vector3(0,0,0));
	tf::Quaternion q((double) quat.x(),
										(double) quat.y(),
										(double) quat.z(),
										(double) quat.w());
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(tranform, ros::Time::now(), "world", imu_name);
}

int main(int argc, char **argv)
{
//	enableMuxer(0b00000011);

	if(argc != 4) {
		printf("Please supply address for lsm, lis, name\n");
		exit(0);
	}
	int lsmaddr, lisaddr;
	lsmaddr = (int) strtol(argv[1], NULL, 16);
	lisaddr = (int) strtol(argv[2], NULL, 16);

	imu_name = argv[3];
  ros::init(argc, argv, imu_name); 

	// Initializa IMU
	MinIMU9 imu("/dev/i2c-1", lsmaddr, lisaddr);
	imu.loadCalibration();
	imu.enable();
	imu.measureOffsets();

	int count = 0;
  ros::Rate loop_rate(50);
	quaternion rotation = quaternion::Identity();

	int start = millis();
  while (ros::ok())
  {
				int last_start = start;
				start = millis();
				float dt = (start-last_start)/1000.0;
				if (dt < 0){ throw std::runtime_error("Time went backwards."); }

				vector angular_velocity = imu.readGyro();
				vector acceleration = imu.readAcc();
				vector magnetic_field = imu.readMag();

				fuse_default(rotation, dt, angular_velocity, acceleration, magnetic_field);

				output_quaternion(rotation);
				std::cout << "  " << acceleration << "  " << magnetic_field << std::endl << std::flush;

				broadcastTF(rotation);
				// Ensure that each iteration of the loop takes at least 20 ms.
        while(millis() - start < 20)
        {
            usleep(1000);
        }
	}

  return 0;
}
