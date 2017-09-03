#include "ros/ros.h"
#include "turtlebot3_custom_hardware.h"
#include "controller_manager/controller_manager.h"

int main(int argc, char **argv) {
	ros::init(argc, argv, "turtlebot3_custom_hardware");
	ros::NodeHandle nh;
	Turtlebot3 robot(nh);
	controller_manager::ControllerManager cm(&robot);

	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::Time prev_time = ros::Time::now();
	ros::Rate loop_rate(30);
	while(ros::ok()) {
		const ros::Time time = ros::Time::now();
		const ros::Duration period = time - prev_time;

		robot.read(period);
		cm.update(time, period);
		robot.write();

		prev_time = time;
		loop_rate.sleep();
	}
	spinner.stop();

	return 0;
}
