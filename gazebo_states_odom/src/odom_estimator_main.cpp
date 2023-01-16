#include <odom_handler.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gazebo_odom_node");

	//handle to launch file parameters
	ros::NodeHandle nhp("~");

  OdomHandler odomHandler;

  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
