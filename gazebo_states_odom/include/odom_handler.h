#ifndef ODOMHANDLER_H
#define ODOMHANDLER_H

#define _USE_MATH_DEFINES

#include <mutex>
#include <vector>
#include <deque>

#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <odom_estimator.h>
#include <helper_functions.h>

struct OdomHandler
{
	ros::NodeHandle nh;
	ros::Subscriber gazeboClockSub;
	ros::Subscriber gazeboStatesSub;
	std::vector<OdomEstimator*> estimators;
	ros::Time tLast;
	std::mutex clockMutex;
	std::mutex stateMutex;
	bool firstClock;
	bool firstState;

	OdomHandler();

	void clockCB(const rosgraph_msgs::Clock::ConstPtr& msg);

	void stateCB(const gazebo_msgs::ModelStates::ConstPtr& msg);
};

#endif
