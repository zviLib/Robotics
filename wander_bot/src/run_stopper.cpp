#include "wander_bot.h"
int main(int argc, char **argv) {
// Initiate new ROS node named "stopper"
	ros::init(argc, argv, "wander_bot");
	ros::NodeHandle nh("~");
	double forwardSpeed = 0.5;
	double angleSpeed = 90.0 / 180 * M_PI;
	double minAngle = -30.0 / 180 * M_PI;
	double maxAngle = 30.0 / 180 * M_PI;

	if (nh.hasParam("forward_speed")) {
		nh.getParam("forward_speed", forwardSpeed);
	}

	if (nh.hasParam("rotation_speed")) {
			nh.getParam("rotation_speed", angleSpeed);
		}

	if (nh.hasParam("min_scan_angle")) {
			nh.getParam("min_scan_angle", minAngle);
		}

	if (nh.hasParam("max_scan_angle")) {
			nh.getParam("max_scan_angle", maxAngle);
		}

// Create new stopper object
	Stopper stopper(forwardSpeed,angleSpeed,minAngle,maxAngle);
// Start the movement
	stopper.startMoving();
	return 0;
}
