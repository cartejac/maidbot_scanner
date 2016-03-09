#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <iostream>
#include <string>
#include <cstdlib>
#include <cmath>
using namespace ros;
using std::cout;
using std::endl;
using std::string;

void get_node_parameters(double& scan_period)
{
	if (!ros::param::get("laser_scan_period", scan_period)) {
		cout << "Could not retrieve laser_scan_period from parameter server. Are you using the launch file> Terminating." << endl;
		exit(1);
	}

	if (scan_period < 0) {
		cout << "Invalid scan period: negative time - " << scan_period << "\n. Terminating." << endl;
		exit(1);
	}
}

void create_template_scan(sensor_msgs::LaserScan& template_msg, double scan_period)
{
	/* Obtain message-specific parameters from the parameter server */
	double scan_range = (3.0/4) * 3.14159; //TODO: Add this to the parameter server
	string scanner_frame("");
	if (!ros::param::get("scanner_frame_id", scanner_frame)) {
		cout << "Could not retrieve scanner_frame_id from parameter server. Are you using the launch file? Terminating." << endl;
		exit(1);
	}

	template_msg.header.frame_id = scanner_frame;
	template_msg.angle_min = ((2 * M_PI) - scan_range) / 2;
	template_msg.angle_max = (2 * M_PI) - template_msg.angle_min;
	template_msg.time_increment = scan_period; //TODO: Understand this parameter better
	template_msg.scan_time = scan_period;
	template_msg.range_min = 0.1;
	template_msg.range_max = 30;
}

void mk_scan(sensor_msgs::LaserScan& msg)
{

	msg.header.stamp = ros::Time::now();
}

int main(int argc, char** argv){
	ros::init(argc, argv, "maidbot_scanner_node");
	ros::NodeHandle nh;
	double scan_period;
	sensor_msgs::LaserScan msg;

	/* Obtain node-specific parameters */
	get_node_parameters(scan_period);
	
	ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 10);
	ros::Duration scan_sleeper(scan_period);
	while (ros::ok()) {
		mk_scan(msg);
		cout << "Publishing laser scan." << endl;
		scan_pub.publish(msg);

		scan_sleeper.sleep();
	}

	return EXIT_SUCCESS;
}
