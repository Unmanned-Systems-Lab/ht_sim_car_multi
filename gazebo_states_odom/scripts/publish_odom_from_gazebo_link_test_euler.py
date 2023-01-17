#!/usr/bin/env python

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

from gazebo_msgs.msg import *
from gazebo_msgs.srv import *
import rospy


from tf import transformations

rospy.init_node('publish_odom_from_gazebo',anonymous=True)
car_name = 'car1'
topic_name = '/' + car_name + '/' + 'odom_gazebo'
ref_frame_id = "gazebo_world"
# root_relative_entity_name = '' # this is the full model and not only the base_link
link_name = 'base_link'
link_name_sum = car_name + '::' + link_name
car_frame_id = '/' + car_name + '/' + 'base_link'
odom_pub = rospy.Publisher(topic_name, Odometry, queue_size=30)
rate = rospy.Rate(30)

while not rospy.is_shutdown():
    rospy.wait_for_service('/gazebo/get_link_state')
    try:
        gms = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        msg = gms(link_name_sum, '')
        # store the information from odom
        (r,p,y) = transformations.euler_from_quaternion([
                                msg.link_state.pose.orientation.x, 
                                msg.link_state.pose.orientation.y, 
                                msg.link_state.pose.orientation.z, msg.link_state.pose.orientation.w
                                ], axes='sxyz')
        print("Pitch = {:.0f},Roll = {:.0f}, Yaw = {:.0f}".format(r*180/3.1415926,p*180/3.1415926,y*180/3.1415926))
        # print(msg)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s"%e)

    send_Od_data = Odometry()
    send_Od_data.header.frame_id = ref_frame_id
    send_Od_data.header.stamp = rospy.Time.now()
    send_Od_data.child_frame_id = car_frame_id
    send_Od_data.pose.pose = msg.link_state.pose
    send_Od_data.twist.twist = msg.link_state.twist
    odom_pub.publish(send_Od_data)
    rate.sleep()
