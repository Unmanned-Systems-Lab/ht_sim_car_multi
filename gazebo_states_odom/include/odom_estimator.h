#ifndef ODOMESTIMATOR_H
#define ODOMESTIMATOR_H

#define _USE_MATH_DEFINES

#include <ros/ros.h>
#include <std_msgs/Time.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <helper_functions.h>

struct OdomEstimator
{
	std::string name;
	ros::NodeHandle nh;
	ros::Publisher odomPub;
	tf::TransformBroadcaster br;
	ros::Time tLast;
	bool firstUpdate;
	Eigen::Matrix<float,13,1> xHat;
	Eigen::Matrix<float,13,13> P;
	Eigen::Matrix<float,13,13> R;

	OdomEstimator(std::string nameInit);

	void update(ros::Time t, geometry_msgs::Pose pose, geometry_msgs::Twist twist);
};

#endif
