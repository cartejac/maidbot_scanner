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

template <typename T>
void get_param_or_exit(const char* param_name, T& p)
{
	if (!ros::param::get(param_name, p)) {
		cout << "Could not retrieve " << param_name << " from parameter server. Are you using the launch file? Terminating." << endl;
		exit(1);
	}

}

void get_node_parameters(double& scan_period)
{
	get_param_or_exit("laser_scan_period", scan_period);
	if (scan_period < 0) {
		cout << "Invalid scan period: negative time - " << scan_period << "\n. Terminating." << endl;
		exit(1);
	}
}

/* Description: Many of the parameters of the laser scan are the same between publishes,
 *	so this node saves time by creating a "template" message and filling in what
 *	changes per publish. This function sets the shared values.
 */
void create_template_scan_msg(sensor_msgs::LaserScan& template_msg, double scan_period)
{
	/* Obtain message-specific parameters from the parameter server */
	double scan_range = 0;
	double angular_res = 0;
	string scanner_frame("");
	get_param_or_exit("scanner_frame_id", scanner_frame);
	get_param_or_exit("scanner_angular_res", angular_res);
	get_param_or_exit("scan_range", scan_range);
	
	/* Validate parameters */
	if (angular_res < 0) {
		cout << "Invalid (negative) angular resolution: " << angular_res << endl;
	}

	if (scan_range < 0 || scan_range > (2 * M_PI)) {
		cout << "Invalid scan_range: " << scan_range << endl;
	}

	/* Set the message's constants */
	unsigned int n_samples = scan_range / angular_res;
	template_msg.header.frame_id = scanner_frame;
	template_msg.angle_min = M_PI - (scan_range/2);
	template_msg.angle_max = M_PI + (scan_range/2);
	template_msg.angle_increment = angular_res;
	template_msg.time_increment = scan_period / n_samples;
	template_msg.scan_time = scan_period;
	template_msg.range_min = 0.1;
	template_msg.range_max = 30;
	template_msg.intensities.clear();
	template_msg.ranges.resize(n_samples);
}

/* Description: Sets the scan message's variable values
 */
void mk_scan(sensor_msgs::LaserScan& msg)
{
	msg.header.stamp = ros::Time::now();
	
	unsigned int n_samples = msg.ranges.size();
	double noise = 0;	/* No sensor is perfect */
	for (unsigned int i = 0; i < n_samples; i++) {
		noise = 0.03 * ((rand() % 100) / 100.0);
		msg.ranges[i] = 3 + noise;
	}
}

int main(int argc, char** argv){
	ros::init(argc, argv, "maidbot_scanner_node");
	srand(0);
	ros::NodeHandle nh;
	double scan_period;
	sensor_msgs::LaserScan msg;

	get_node_parameters(scan_period);

	create_template_scan_msg(msg, scan_period);

	ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 10);
	ros::Duration scan_T(scan_period);
	ros::Duration sleep_time;
	ros::Time start_time;
	while (ros::ok()) {
		start_time = ros::Time::now();
		mk_scan(msg);
		cout << "Publishing laser scan." << endl;
		scan_pub.publish(msg);

		/* Ensure the node publishes at a constant frequency, 
		   regardless of computation */
		sleep_time = scan_T - (ros::Time::now() - start_time);
		sleep_time.sleep();
	}

	return EXIT_SUCCESS;
}
