#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
import math as MT
import time
import os.path 

save_path = "/home/bija/Dropbox/#Supervision/2014_Irme_SocialTurtles/matlab"
name_of_file = time.strftime("%c")
name= os.path.join(save_path, name_of_file+".txt" )

f = open(name,'w')

CHICKEN_DIST = 1

pose_dict = {}

blackboard = {}
rostime = 0


def get_pose(msg):
    global rostime
    pose = {}    
    pose['sin'] = msg.pose.pose.orientation.z
    pose['cos'] = msg.pose.pose.orientation.w
    pose['theta'] = MT.atan(msg.pose.pose.orientation.z/msg.pose.pose.orientation.w)/MT.pi*360
    pose['x'] = msg.pose.pose.position.x
    pose['y'] = msg.pose.pose.position.y
    robot_id = msg.header.frame_id[7:-5]
    pose_dict[robot_id] = pose
    f.write(str(rospy.get_time()) + " " + str(robot_id) + " " +str (pose['x']) + " " +str (pose['y']) + " " +str (pose['theta'])  + "\n")     



def talker():
    pub = rospy.Publisher('chatter', String)
    rospy.init_node('talker', anonymous=True)

    for i in xrange(100):
        rospy.Subscriber('robot_' + str(i) + "/base_pose_ground_truth", Odometry, get_pose)
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        neighbors_list = []
        for key1 in pose_dict.keys():
            for key2 in pose_dict.keys():
                if key2 is not key1:
                    x1 = pose_dict[key1]['x']
                    y1 = pose_dict[key1]['y']
                    theta1 = pose_dict[key1]['theta']
                    x2 = pose_dict[key2]['x']
                    y2 = pose_dict[key2]['y']
                    if x2 == x1:
                        x2 = x2 + .01
                    relative_theta = MT.atan2((y2 - y1),(x2-x1))/MT.pi*180
                    # either robots see each other from distance or are too close that line of sight doesn't matter			
                    neighbors_distance = (MT.fabs(x1-x2) + MT.fabs(y1-y2))
		    if (neighbors_distance < CHICKEN_DIST and MT.fabs(relative_theta - theta1) < 120) :
#                        print relative_theta, theta1
                        
                        neighbors_list.append([int(key1),int(key2), neighbors_distance ])

#                    if int(key1) == 0 and int(key2) == 1:
#                        print theta1, relative_theta

            #str_variable = "hello world %s"%rospy.get_time()
            # rospy.loginfo(str_variable)
            # pub.publish(str(pose_dict)
        blackboard['neighbors_list'] = neighbors_list
        pub.publish(str(blackboard))
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
        f.close()
    except rospy.ROSInterruptException:
        pass
