#include "wander_bot.h"
#include "geometry_msgs/Twist.h"

Stopper::Stopper(double forwardSpeed) {
	this->forwardSpeed = forwardSpeed;
	this->origForward = forwardSpeed;
	this->angle = 0.0;
	keepMoving = true;
	currentAngle = 0.0;
	bestAngle = 0.0;
	isObstacleInFront = false;

// Advertise a new publisher for the robot's velocity command topic
	commandPub = node.advertise<geometry_msgs::Twist>(
			"/cmd_vel_mux/input/teleop", 1);
	// Subscribe to the robot's position
	poseSub = node.subscribe("odom", 1, &Stopper::poseCallback, this);
	// Subscribe to the simulated robot's laser scan topic
	laserSub = node.subscribe("scan", 1, &Stopper::scanCallback, this);
}

// Send a velocity command
void Stopper::moveForward() {
	geometry_msgs::Twist msg; // The default constructor will set all
	//commands to 0
	msg.linear.x = forwardSpeed;
	msg.angular.z = angle;

	commandPub.publish(msg);
}

// get robot's current angle
void Stopper::poseCallback(const nav_msgs::Odometry::ConstPtr& msg) {
	currentAngle = msg->pose.pose.orientation.z;
	ROS_INFO("current:[%f], best:[%f]",currentAngle,bestAngle);
}

// Process the incoming laser scan message
void Stopper::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {

	// rotate untill the robot's front is in the freest direction
	if (isObstacleInFront) {
		// return to normal movement
		if (std::abs(currentAngle - bestAngle) < 0.05) {
			this->forwardSpeed = this->origForward;
			this->angle = 0.0;
			isObstacleInFront = false;
			bestAngle = 0;
		}
		return;
	}

	// Find the closest range between the defined minimum and maximum angles
	int minIndex = ceil(
			(MIN_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);
	int maxIndex = floor(
			(MAX_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);

	for (int currIndex = minIndex + 1; currIndex <= maxIndex; currIndex++) {
		if (scan->ranges[currIndex] < MIN_DIST_FROM_OBSTACLE) {
			isObstacleInFront = true;
			break;
		}
	}

	if (isObstacleInFront) {

		// find the freest direction
		float bestIndex = 0;
		float bestDistance = 0;
		for (int i = 0; i < scan->ranges.size(); i++) {
			if (scan->ranges[i] > bestDistance) {
				bestDistance = scan->ranges[i];
				bestIndex = i;
			}
		}

		ROS_INFO("Stop! ");

		// set velocity, angle and wanted direction
		this->forwardSpeed = 0;
		bestAngle = bestIndex * scan->angle_increment + scan->angle_min;

		if (currentAngle > bestAngle)
			this->angle = -90.0 / 180 * M_PI;
		else
			this->angle = 90.0 / 180 * M_PI;

	} else { // if there is no obstacle - continue forward
		this->forwardSpeed = this->origForward;
		this->angle = 0.0;
	}
}

void Stopper::startMoving() {
	ros::Rate rate(1000);
	ROS_INFO("Start moving");
	// Keep spinning loop until user presses Ctrl+C or the robot got too close to an obstacle
	while (ros::ok()) {
		moveForward();
		ros::spinOnce(); // Need to call this function often to allow ROS to
		//process incoming messages
		rate.sleep();
	}
}
