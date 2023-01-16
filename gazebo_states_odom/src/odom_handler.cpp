#include <odom_handler.h>

OdomHandler::OdomHandler()
{
	gazeboClockSub = nh.subscribe("/clock",1,&OdomHandler::clockCB,this);
	gazeboStatesSub = nh.subscribe("/gazebo/model_states",1,&OdomHandler::stateCB,this);
	firstClock = true;
	firstState = true;
}

void OdomHandler::clockCB(const rosgraph_msgs::Clock::ConstPtr& msg)
{
	//protect read/write
	std::lock_guard<std::mutex> clockMutexGuard(clockMutex);
	ros::Time t = msg->clock;

	//if its the first clock initialize tlast
	if (firstClock)
	{
		tLast = t;
		firstClock = false;
	}
	tLast = t;
}

void OdomHandler::stateCB(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
	//protect read/write
	std::lock_guard<std::mutex> stateMutexGuard(stateMutex);

	std::vector<std::string> names = msg->name;
	std::vector<geometry_msgs::Pose> poses = msg->pose;
	std::vector<geometry_msgs::Twist> twists = msg->twist;

	//if the clock has not started return
	if (firstClock)
	{
		return;
	}

	//if first state callback, initialize the odoms
	//skip the ground_plane at 0
	if (firstState)
	{
		for (int ii = 1; ii < names.size(); ii++)
		{
			OdomEstimator* newOdomEstimator = new OdomEstimator(names.at(ii));
			estimators.push_back(newOdomEstimator);
		}
		firstState = false;
	}

	//stop the program if
	if (((names.size() - 1) - estimators.size()) != 0)
	{
		ROS_ERROR("Number of names changed, exiting.");

		//delete all the estimators and shutdown
		for (int ii = 0; ii < estimators.size(); ii++)
		{
			delete estimators.at(ii);
		}
		gazeboClockSub.shutdown();
		gazeboStatesSub.shutdown();
		ros::shutdown();
		return;
	}

	//using the last clock and current state to update the estimators
	//increment the pose and twist to skip ground_plane
	for (int ii = 0; ii < estimators.size(); ii++)
	{
		estimators.at(ii)->update(tLast,poses.at(ii+1),twists.at(ii+1));
	}
}
