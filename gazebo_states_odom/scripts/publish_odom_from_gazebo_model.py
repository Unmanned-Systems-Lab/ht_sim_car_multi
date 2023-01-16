#!/usr/bin/env python
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

from gazebo_msgs.msg import *
from gazebo_msgs.srv import *
import rospy

rospy.init_node('publish_odom_from_gazebo',anonymous=True)
car_name = 'car1'
topic_name = '/' + car_name + '/' + 'odom_gazebo'
ref_frame_id = "gazebo_world"
root_relative_entity_name = '' # this is the full model and not only the base_link
car_frame_id = '/' + car_name + '/' + 'base_link'
odom_pub = rospy.Publisher(topic_name, Odometry, queue_size=30)
rate = rospy.Rate(30)

while not rospy.is_shutdown():
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        gms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        msg = gms(car_name, root_relative_entity_name)
        # print(msg)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s"%e)

    send_Od_data = Odometry()
    send_Od_data.header.frame_id = ref_frame_id
    send_Od_data.header.stamp = rospy.Time.now()
    send_Od_data.child_frame_id = car_frame_id
    send_Od_data.pose.pose = msg.pose
	# send_Od_data.pose.pose.position.x = msg.pose.position.x
	# send_Od_data.pose.pose.position.y = msg.pose.position.y
	# send_Od_data.pose.pose.position.z = msg.pose.position.z
    send_Od_data.twist.twist = msg.twist
	# send_Od_data.twist.twist.angular.x = msg.twist.angular.x
	# send_Od_data.twist.twist.angular.y = msg.twist.angular.y
	# send_Od_data.twist.twist.angular.z = msg.twist.angular.z
    odom_pub.publish(send_Od_data)
    rate.sleep()
