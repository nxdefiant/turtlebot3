#include <cmath>
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <byteswap.h>
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
			uint8_t chksum=0xa5;
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
			for (int i=0; i<8; i++) {
				chksum^=buf[i];
			}

			if (buf[8] != chksum) {
				ROS_ERROR("I2C Read error: Chksum mismatch, expected 0x%x got 0x%x", chksum, buf[8]);
				goto error;
			}
			vel[0] = *(float*)(buf+0) *(2*M_PI)/60; // rpm to rad/s
			vel[1] = *(float*)(buf+4) *(2*M_PI)/60; // rpm to rad/s

			pos[0] += vel[0] * period.toSec();
			pos[1] += vel[1] * period.toSec();

			error:
				close(file);
		}

		// Writes current velocity command to hardware
		void write() {
			int file;
			uint8_t buf[32];
			int ret;
			int16_t speed_l, speed_r;


			if ((file = open(I2C_FILE, O_RDWR)) < 0) {
				perror("open");
				goto error;
			}

			if (ioctl(file, I2C_SLAVE, I2C_ADDR) < 0) {
				perror("ioctl");
				goto error;
			}

			speed_l = cmd[0] * 60/(2*M_PI); // rad/s to rpm
			speed_r = cmd[1] * 60/(2*M_PI); // rad/s to rpm
			buf[0] = speed_l;
			buf[1] = speed_l>>8;
			buf[2] = speed_r;
			buf[3] = speed_r>>8;
			if ((ret = i2c_smbus_write_i2c_block_data(file, 0x20, 4, buf)) != 0) {
				perror("i2c_smbus_write_block_data");
				goto error;
			}

			error:
				close(file);
		}

		void cbDynReconf(turtlebot3_custom::Turtlebot3Config &config, uint32_t level) {
			int file;
			uint8_t buf[32];
			int ret;
			float kp=config.kp;
			float ki=config.ki;
			float kd=config.kd;

			ret = __bswap_32(*(int32_t *)&kp);
			kp = *(float *)&ret;
			ret = __bswap_32(*(int32_t *)&ki);
			ki = *(float *)&ret;
			ret = __bswap_32(*(int32_t *)&kd);
			kd = *(float *)&ret;

			if ((file = open(I2C_FILE, O_RDWR)) < 0) {
				perror("open");
				goto error;
			}

			if (ioctl(file, I2C_SLAVE, I2C_ADDR) < 0) {
				perror("ioctl");
				goto error;
			}

			memcpy(buf+0, &kp, 4);
			memcpy(buf+4, &ki, 4);
			memcpy(buf+8, &kd, 4);
			buf[12] = config.motor_inc_step;
			buf[13] = config.motor_min;
			if ((ret = i2c_smbus_write_i2c_block_data(file, 0x70, 14, buf)) != 0) {
				perror("i2c_smbus_write_block_data");
				goto error;
			}

			error:
				close(file);
		}


	private:
		ros::NodeHandle nh;
		hardware_interface::JointStateInterface jnt_state_interface;
		hardware_interface::VelocityJointInterface jnt_velocity_interface;
		double cmd[2] = {0.0, 0.0};
		double pos[2] = {0.0, 0.0};
		double vel[2] = {0.0, 0.0};
		double eff[2] = {0.0, 0.0};
		dynamic_reconfigure::Server<turtlebot3_custom::Turtlebot3Config> server;
};
