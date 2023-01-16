#include <odom_estimator.h>

OdomEstimator::OdomEstimator(std::string nameInit)
{
	name = nameInit;
	odomPub = nh.advertise<nav_msgs::Odometry>(name+"/odomEKF",1);
	firstUpdate = true;

	xHat = Eigen::Matrix<float,13,1>::Zero();
	P = 0.001*Eigen::Matrix<float,13,13>::Identity();
	R = 0.001*Eigen::Matrix<float,13,13>::Identity();
}

void OdomEstimator::update(ros::Time t, geometry_msgs::Pose pose, geometry_msgs::Twist twist)
{
	//ros::Time t = msg->header.stamp;
	Eigen::Vector3f p(pose.position.x,pose.position.y,pose.position.z);
	Eigen::Vector4f q(pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z);
	Eigen::Vector3f v(twist.linear.x,twist.linear.y,twist.linear.z);
	Eigen::Vector3f w(twist.angular.x,twist.angular.y,twist.angular.z);
	q /= q.norm();

	//initialize the measurement
	Eigen::Matrix<float,13,1> z;
	z.segment(0,3) = p;
	z.segment(3,4) = q;
	z.segment(7,3) = v;
	z.segment(10,3) = w;

	//initialize the state estimate and time if first callback
	if (firstUpdate)
	{
		xHat = z;
		tLast = t;
		firstUpdate = false;
	}

	//check for sign flip
	if ((xHat.segment(3,4) + q).norm() < (xHat.segment(3,4) - q).norm())
	{
		q *= -1.0;
	}

	float dt = (t-tLast).toSec();
	tLast = t;

	Eigen::Matrix<float,13,13> argK = P+R;
	Eigen::JacobiSVD<Eigen::MatrixXf> svdargK(argK, Eigen::ComputeThinU | Eigen::ComputeThinV);
	Eigen::Matrix<float,13,13> argKI = svdargK.solve(Eigen::Matrix<float,13,13>::Identity());
	Eigen::Matrix<float,13,13> K = P*argKI;
	xHat += (K*(z-xHat));
	P = (Eigen::Matrix<float,13,13>::Identity() - K)*P;

	Eigen::Vector4f qHat = xHat.segment(3,4)/xHat.segment(3,4).norm();

	//set stamp and frame id
	geometry_msgs::Pose poseMsg;
	geometry_msgs::Twist velMsg;
	nav_msgs::Odometry odomMsg;

	odomMsg.child_frame_id = name;
	odomMsg.header.frame_id = "ground_plane";
	odomMsg.header.stamp = tLast;

	//set data
	poseMsg.position.x = xHat(0); poseMsg.position.y = xHat(1); poseMsg.position.z = xHat(2);
	poseMsg.orientation.w = qHat(0); poseMsg.orientation.x = qHat(1); poseMsg.orientation.y = qHat(2); poseMsg.orientation.z = qHat(3);

	velMsg.linear.x = xHat(7); velMsg.linear.y = xHat(8); velMsg.linear.z = xHat(9);
	velMsg.angular.x = xHat(10); velMsg.angular.y = xHat(11); velMsg.angular.z = xHat(12);

	odomMsg.pose.pose = poseMsg;
	odomMsg.twist.twist = velMsg;

	tf::Vector3 tfposeP(poseMsg.position.x,poseMsg.position.y,poseMsg.position.z);
	tf::Quaternion tfposeQ(poseMsg.orientation.x,poseMsg.orientation.y,poseMsg.orientation.z,poseMsg.orientation.w);
	tf::Transform tftransform(tfposeQ,tfposeP);
	br.sendTransform(tf::StampedTransform(tftransform, tLast, "ground_plane", name));

	int poseind = 0;
	int twistind = 0;
	for (int ii = 0; ii < 13*13; ii++)
	{
		int row = ii/13;
		int col = ii%13;

		//std::cout << "\n ***** \n row " << row << " col " << col;

		//p-p components
		if(row < 3 && col < 3)
		{
			//std::cout << " p-p" << std::endl;
			odomMsg.pose.covariance.at(poseind) = P(row,col);
			poseind++;
		}
		//p-qv components
		else if(row < 3 && col > 3 && col < 7)
		{
			//std::cout << " p-qv" << std::endl;
			odomMsg.twist.covariance.at(poseind) = P(row,row);
			poseind++;
		}
		//qv-p components
		else if(row > 3 && row < 7 && col < 3)
		{
			//std::cout << " qv-p" << std::endl;
			odomMsg.pose.covariance.at(poseind) = P(row,col);
			poseind++;
		}
		//qv-qv components
		else if(row > 3 && row < 7 && col > 3 && col < 7)
		{
			//std::cout << " qv-qv" << std::endl;
			odomMsg.pose.covariance.at(poseind) = P(row,col);
			poseind++;
		}
		//v-v components
		else if(row >= 7 && row < 10 && col >= 7 && col < 10)
		{
			//std::cout << " v-v" << std::endl;
			odomMsg.twist.covariance.at(twistind) = P(row,col);
			twistind++;
		}
		//v-w components
		else if(row >= 7 && row < 10 && col >= 10)
		{
			//std::cout << " v-w" << std::endl;
			odomMsg.twist.covariance.at(twistind) = P(row,col);
			twistind++;
		}
		//w-v components
		else if(row >= 10 && col >= 7 && col < 10)
		{
			//std::cout << " w-v" << std::endl;
			odomMsg.twist.covariance.at(twistind) = P(row,col);
			twistind++;
		}
		//w-w components
		else if(row >= 10 && col >= 10)
		{
			//std::cout << " w-w" << std::endl;
			odomMsg.twist.covariance.at(twistind) = P(row,col);
			twistind++;
		}

		//std::cout << "\n ------ \n";
	}


	//publish
	odomPub.publish(odomMsg);
}
