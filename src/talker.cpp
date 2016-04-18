#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"

#include <sstream>
#include <time.h>
#include <sys/time.h>

#include "minimu9-ahrs.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<sensor_msgs::Imu>("imu/data", 1000);
	std::string frame = "1";

	// Initializa IMU
	MinIMU9 imu("/dev/i2c-1");
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

				sensor_msgs::Imu msg;
				msg.header.frame_id = frame;
				msg.header.stamp = ros::Time::now();

				output_quaternion(rotation);
				std::cout << "  " << acceleration << "  " << magnetic_field << std::endl << std::flush;

				write_quaternion(msg, rotation);

				msg.linear_acceleration.x = acceleration[0];
				msg.linear_acceleration.y = acceleration[1];
				msg.linear_acceleration.z = acceleration[2];

				msg.angular_velocity.x = angular_velocity[0];
				msg.angular_velocity.y = angular_velocity[1];
				msg.angular_velocity.z = angular_velocity[2];

				chatter_pub.publish(msg);
      	// Ensure that each iteration of the loop takes at least 20 ms.
        while(millis() - start < 20)
        {
            usleep(1000);
        }
	}

  return 0;
}
