#!/usr/bin/env python

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

from gazebo_msgs.msg import *
from gazebo_msgs.srv import *
import rospy

rospy.init_node('publish_odom_from_gazebo',anonymous=True)
car_name = rospy.get_param('car_name')
# car_name = 'car1'
topic_name = '/' + car_name + '/' + 'odom_gazebo_back_wheel'
ref_frame_id = "gazebo_world"
# root_relative_entity_name = '' # this is the full model and not only the base_link
link_name = 'base_link'
link_name_base_link = car_name + '::' + 'base_link'
link_name_re_right_link = car_name + '::' + 're_right_link'
link_name_re_left_link = car_name + '::' + 're_left_link'

car_frame_id = '/' + car_name + '/' + 'base_link'
ackermann_steering_controller_odom = '/' + car_name + '/' + 'ackermann_steering_controller/odom'
odom_pub = rospy.Publisher(topic_name, Odometry, queue_size=30)
# odom_sub = rospy.Subscriber('/car1/ackermann_steering_controller/odom', nav_msgs.Odometry, OdomCB)
rate = rospy.Rate(40)

while not rospy.is_shutdown():
    msg2 = rospy.wait_for_message(ackermann_steering_controller_odom,Odometry, timeout=None)
    # print( msg2)

    rospy.wait_for_service('/gazebo/get_link_state')
    try:
        gms = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        msg_link_name_base_link = gms(link_name_base_link, '')
        # print(msg_link_name_base_link)
        msg_link_name_re_right_link = gms(link_name_re_right_link, '')
        msg_link_name_re_left_link = gms(link_name_re_left_link, '')
        # print(msg)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s"%e)

    send_Od_data = Odometry()
    send_Od_data.header.frame_id = ref_frame_id
    send_Od_data.header.stamp = rospy.Time.now()
    send_Od_data.child_frame_id = car_frame_id

    send_Od_data.pose.pose.orientation = msg_link_name_base_link.link_state.pose.orientation
    send_Od_data.pose.pose.position.x = (msg_link_name_re_right_link.link_state.pose.position.x + msg_link_name_re_left_link.link_state.pose.position.x)/2
    send_Od_data.pose.pose.position.y = (msg_link_name_re_right_link.link_state.pose.position.y + msg_link_name_re_left_link.link_state.pose.position.y)/2
    send_Od_data.pose.pose.position.z = (msg_link_name_re_right_link.link_state.pose.position.z + msg_link_name_re_left_link.link_state.pose.position.z)/2
    send_Od_data.twist.twist = msg2.twist.twist
    odom_pub.publish(send_Od_data)
    rate.sleep()
