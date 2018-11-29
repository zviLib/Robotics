#include "wander_bot.h"
#include "geometry_msgs/Twist.h"

Stopper::Stopper(double forwardSpeed, double angleSpeed, double minAngle,
		double maxAngle) {
	this->forwardSpeed = forwardSpeed;
	this->origAngle = angleSpeed;
	this->minScanAngle = minAngle;
	this->maxScanAngle = maxAngle;
	keepMoving = true;
	currentAngle = 0.0;
	bestAngle = 0.0;
	isObstacleInFront = false;

	// set movement starting parameters
	this->origForward = forwardSpeed;
	this->angle = 0;

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
	//ROS_INFO("current:[%f], best:[%f]",currentAngle,bestAngle);
}

// Process the incoming laser scan message
void Stopper::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {

	// rotate until the robot's front is in the freest direction found
	if (isObstacleInFront) {
		// return to normal movement
		if (std::abs(currentAngle - bestAngle) < 0.05) {
			this->forwardSpeed = this->origForward;
			this->angle = 0;
			isObstacleInFront = false;
		}
		return;
	}

	// Find the closest range between the defined minimum and maximum angles
	int minIndex = ceil(
			(this->minScanAngle - scan->angle_min) / scan->angle_increment);
	int maxIndex = floor(
			(this->maxScanAngle - scan->angle_min) / scan->angle_increment);

	for (int currIndex = minIndex + 1; currIndex <= maxIndex; currIndex++) {
		if (scan->ranges[currIndex] < MIN_DIST_FROM_OBSTACLE) {
			isObstacleInFront = true;
			break;
		}
	}

	// if an obstacle is blocking the way
	if (isObstacleInFront) {
		ROS_INFO("Looking for a new route! ");

		// find freest direction
		float bestIndex = 0;
		float bestCount = 0;
		int count = 0;

		// find largest section of NaN and aim to it's middle
		for (int currIndex = minIndex + 1; currIndex <= maxIndex; currIndex++) {
			//check if nan
			if (scan->ranges[currIndex] != scan->ranges[currIndex]) {
				count++;
			} else if (count > bestCount) {
				// aim to the middle of the free section
				bestIndex = (currIndex + (currIndex - count)) / 2;
				bestCount = count;
				count = 0;
			} else {
				count = 0;
			}
		}

		// set velocity, angle and wanted direction
		this->forwardSpeed = 0;
		bestAngle = bestIndex * scan->angle_increment + scan->angle_min;

		// set direction of the rotation
		if (currentAngle > bestAngle)
			this->angle = -origAngle;
		else
			this->angle = origAngle;

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
