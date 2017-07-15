#include <cmath>
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include "std_msgs/Float64.h"
#include <dynamic_reconfigure/server.h>
#include <turtlebot3_custom/Turtlebot3Config.h>

#define I2C_FILE "/dev/i2c-1" 
#define I2C_ADDR (0x50>>1)

class Turtlebot3 : public hardware_interface::RobotHW
{
	public:
		Turtlebot3(ros::NodeHandle nh) { 
			this->nh = nh;
			pub_left_wheel_state = nh.advertise<std_msgs::Float64>("/left_wheel/state", 10);
			pub_left_wheel_setpoint = nh.advertise<std_msgs::Float64>("/left_wheel/setpoint", 10);
			sub_left_wheel_set = nh.subscribe("/left_wheel/control_effort", 10, &Turtlebot3::cbLeftWheelSet, this);
			pub_right_wheel_state = nh.advertise<std_msgs::Float64>("/right_wheel/state", 10);
			pub_right_wheel_setpoint = nh.advertise<std_msgs::Float64>("/right_wheel/setpoint", 10);
			sub_right_wheel_set = nh.subscribe("/right_wheel/control_effort", 10, &Turtlebot3::cbRightWheelSet, this);

			// connect and register the joint state interface
			hardware_interface::JointStateHandle state_handle_left("wheel_left_joint", &pos[0], &vel[0], &eff[0]);
			jnt_state_interface.registerHandle(state_handle_left);
			hardware_interface::JointStateHandle state_handle_right("wheel_right_joint", &pos[1], &vel[1], &eff[1]);
			jnt_state_interface.registerHandle(state_handle_right);
			registerInterface(&jnt_state_interface);

			// connect and register the joint velocity interface
			hardware_interface::JointHandle joint_handle_left(state_handle_left, &cmd[0]);
			jnt_velocity_interface.registerHandle(joint_handle_left);
			hardware_interface::JointHandle joint_handle_right(state_handle_right, &cmd[1]);
			jnt_velocity_interface.registerHandle(joint_handle_right);
			registerInterface(&jnt_velocity_interface);

			dynamic_reconfigure::Server<turtlebot3_custom::Turtlebot3Config>::CallbackType f;
			f = boost::bind(&Turtlebot3::cbDynReconf, this, _1, _2);
			server.setCallback(f);
		}

		// Reads current velocity from hardware
		void read(ros::Duration period) {
			int file;
			uint8_t buf[32];
			int ret;
			std_msgs::Float64 msg_left;
			std_msgs::Float64 msg_right;
		       
			if ((file = open(I2C_FILE, O_RDWR)) < 0) {
				perror("open");
				goto error;
			}

			if (ioctl(file, I2C_SLAVE, I2C_ADDR) < 0) {
				perror("ioctl");
				goto error;
			}

			if ((ret = i2c_smbus_read_i2c_block_data(file, 0x60, 9, buf)) != 9) {
				perror("i2c_smbus_read_block_data");
				goto error;
			}

			if (buf[8] != 0xa5) {
				fprintf(stderr, "I2C Read error: Well known 0xa5 mismatch\n");
				goto error;
			} else if (std::isnan(vel[0])) {
				fprintf(stderr, "I2C Read error: Value 1 is NaN\n");
				goto error;
			} else if (std::isnan(vel[1])) {
				fprintf(stderr, "I2C Read error: Value 2 is NaN\n");
				goto error;
			}
			vel[0] = *(float*)(buf+0) *(2*M_PI)/60; // rpm to rad/s
			vel[1] = *(float*)(buf+4) *(2*M_PI)/60; // rpm to rad/s

			pos[0] += vel[0] * period.toSec();
			pos[1] += vel[1] * period.toSec();

			msg_left.data = vel[0];
			pub_left_wheel_state.publish(msg_left);
			msg_right.data = vel[1];
			pub_right_wheel_state.publish(msg_right);

			error:
				close(file);
		}

		// Writes current velocity command to hardware
		void write() {
			std_msgs::Float64 msg_left, msg_right;
			msg_left.data = cmd[0];
			pub_left_wheel_setpoint.publish(msg_left);
			msg_right.data = cmd[1];
			pub_right_wheel_setpoint.publish(msg_right);
		}

		void cbLeftWheelSet(const std_msgs::Float64::ConstPtr& msg) {
			int file;
			uint8_t buf[32];
			int ret;
			double val = msg->data;

			// Interpret pid values with wring polarity as 0
			if (val < min_pwm && cmd[0] > 0) val = min_pwm;
			else if (val > -min_pwm && cmd[0] < 0) val = -min_pwm;
			else if (cmd[0] == 0) val = 0;

			if ((file = open(I2C_FILE, O_RDWR)) < 0) {
				perror("open");
				goto error;
			}

			if (ioctl(file, I2C_SLAVE, I2C_ADDR) < 0) {
				perror("ioctl");
				goto error;
			}

			buf[0] = (int16_t)val;
			buf[1] = (int16_t)val>>8;
			if ((ret = i2c_smbus_write_i2c_block_data(file, 0x20, 2, buf)) != 0) {
				perror("i2c_smbus_write_block_data");
				goto error;
			}

			error:
				close(file);
		}

		void cbRightWheelSet(const std_msgs::Float64::ConstPtr& msg) {
			int file;
			uint8_t buf[32];
			int ret;
			double val = msg->data;

			// Interpret pid values with wring polarity as 0
			if (val < min_pwm && cmd[1] > 0) val = min_pwm;
			else if (val > -min_pwm && cmd[1] < 0) val = -min_pwm;
			else if (cmd[1] == 0) val = 0;

			if ((file = open(I2C_FILE, O_RDWR)) < 0) {
				perror("open");
				goto error;
			}

			if (ioctl(file, I2C_SLAVE, I2C_ADDR) < 0) {
				perror("ioctl");
				goto error;
			}

			buf[0] = (int16_t)val;
			buf[1] = (int16_t)val>>8;
			if ((ret = i2c_smbus_write_i2c_block_data(file, 0x22, 2, buf)) != 0) {
				perror("i2c_smbus_write_block_data");
				goto error;
			}

			error:
				close(file);
		}

		void cbDynReconf(turtlebot3_custom::Turtlebot3Config &config, uint32_t level) {
			min_pwm = config.min_pwm;
		}


	private:
		ros::NodeHandle nh;
		hardware_interface::JointStateInterface jnt_state_interface;
		hardware_interface::VelocityJointInterface jnt_velocity_interface;
		double cmd[2] = {0.0, 0.0};
		double pos[2] = {0.0, 0.0};
		double vel[2] = {0.0, 0.0};
		double eff[2] = {0.0, 0.0};
		double min_pwm;
		ros::Publisher pub_left_wheel_state;
		ros::Publisher pub_left_wheel_setpoint;
		ros::Subscriber sub_left_wheel_set;
		ros::Publisher pub_right_wheel_state;
		ros::Publisher pub_right_wheel_setpoint;
		ros::Subscriber sub_right_wheel_set;
		dynamic_reconfigure::Server<turtlebot3_custom::Turtlebot3Config> server;
};
