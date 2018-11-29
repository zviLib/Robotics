#include "wander_bot.h"
int main(int argc, char **argv) {
// Initiate new ROS node named "stopper"
	ros::init(argc, argv, "wander_bot");
	ros::NodeHandle nh("~");
	double forwardSpeed = 0.5;

	if (nh.hasParam("forward_speed")) {
		nh.getParam("forward_speed", forwardSpeed);
	}

// Create new stopper object
	Stopper stopper(forwardSpeed);
// Start the movement
	stopper.startMoving();
	return 0;
}
