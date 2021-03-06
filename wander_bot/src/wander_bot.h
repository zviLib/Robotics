#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
class Stopper {
public:
// Tunable parameter
	const static float MIN_DIST_FROM_OBSTACLE = 0.5; // Should be smaller
	//than sensor_msgs::LaserScan::range_max
	Stopper(double forwardSpeed, double angleSpeed, double minAngle,
			double maxAngle);
	void startMoving();
private:
	double forwardSpeed;
	double origForward;
	double origAngle;
	double minScanAngle;
	double maxScanAngle;
	double angle;
	double currentAngle;
	double bestAngle;
	bool isObstacleInFront;
	ros::NodeHandle node;
	ros::Publisher commandPub; // Publisher to the robot's velocity command
	//topic
	ros::Subscriber laserSub; // Subscriber to the robot's laser scan topic
	ros::Subscriber poseSub;
	bool keepMoving; // Indicates whether the robot should continue moving
	void moveForward();
	void moveAngular();
	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
	void poseCallback(const nav_msgs::Odometry::ConstPtr& msg);
};
