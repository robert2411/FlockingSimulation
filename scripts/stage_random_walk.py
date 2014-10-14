#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import random as RD
from std_msgs.msg import String

global robot_number, mode, chicken_mode

mode = "STRAIGHT"
chicken_mode = False
MAX_SPEED = 0.8
MAX_ROTATION_SPEED = 1
import random as RD

COOPERATION = True
# the min_distance should be the same as the one in talker.py file!!
MIN_DISTANCE = 1 # obstacle avoidance distance, for simulator .5 works just fine while for real robots we should stick to something like 1.0
halt_time = 0;
opponent_distance = 1000; # initialize with a large number
chicken_turn_direction = None;

def GetLaser(msg):
    GetLaser2(msg,robot_number)


def GetLaser2(msg, robot_number):
    global mode, closest, MIN_DISTANCE 


    HALF_RANGE = len(msg.ranges)/2  # half of the laser resolution
    
    scan = msg.ranges # a list of all scanned ranges

    closest = min(scan[HALF_RANGE / 3 : 5 * HALF_RANGE / 4]) # distance to the closest obstacle
    closest_index = min( (v, i) for i, v in enumerate(scan) )[1] # index of closest obstacle
    furthest_index = max( (v, i) for i, v in enumerate(scan) )[1] # index of the furthest obstacle

    if closest > MIN_DISTANCE: # if there is no obstacle near by
        if RD.random() < .2:
            mode = RD.choice(["TURN CCW", "TURN CW"])
        else:          
            mode = "STRAIGHT"
    else:
        if mode == "STRAIGHT": 
            if sum(scan[:HALF_RANGE]) < sum(scan[HALF_RANGE:]): # if there is an obstacle, check which side there are less obstacles 
                mode = "TURN CCW"
            else:
                mode = "TURN CW"
    talker(mode,robot_number)

    
def CollisionDetect(msg):
	global chicken_mode, opponent_distance, halt_time

	blackboard = eval(msg.data)
	col_list = blackboard['neighbors_list']
	temp_flag = False
	for pair in col_list:
		if int(robot_number) is pair[0]:
		    temp_flag = True
		    opponent_distance = pair[2]

	if temp_flag:
	    chicken_mode = True

	else:
	    chicken_mode = False
	    halt_time = 0

def listener():
        rospy.init_node('listener',anonymous=True)
        rospy.Subscriber('robot_'+ robot_number + "/base_scan", LaserScan, GetLaser)
        rospy.Subscriber("/chatter", String, CollisionDetect)
        rospy.spin()

def set_twist(x,z):
    twist = Twist()
    twist.linear.x = x  # our forward speed
    twist.linear.y = 0 
    twist.linear.z = 0 # we can't use these!        
     
    twist.angular.x = 0 
    twist.angular.y = 0   
    twist.angular.z = z # rotate CCW
    
    return twist

def talker(mode, robot_number):
	'''
	determines the linear and angular twist according to mode (i.e., D or D), other obstacles, etc. 
	'''
	global chicken_turn_direction, halt_time

	pub = rospy.Publisher('robot_' + robot_number + '/cmd_vel', Twist)
	twist = set_twist(0,0)
	
	if halt_time < 100:
		if chicken_mode: 
		    if COOPERATION:
			if not chicken_turn_direction:
			    chicken_turn_direction = RD.choice([-1,1])
			twist = set_twist(0, chicken_turn_direction * MAX_ROTATION_SPEED)
		    else: 
			#print opponent_distance, MIN_DISTANCE
			if opponent_distance < MIN_DISTANCE : 
				twist = set_twist(0,0)
				halt_time = halt_time + 1
		else:
			chicken_turn_direction = 0;
			halt_time = 0
			if mode == "STRAIGHT":
			    twist = set_twist(MAX_SPEED,0)
			if mode == "TURN CCW":
			    twist = set_twist(0,MAX_ROTATION_SPEED)    
			if mode == "TURN CW":
			    twist = set_twist(0,-1*MAX_ROTATION_SPEED)    

	else:

		if not chicken_turn_direction:
			chicken_turn_direction = RD.choice([-1,1])
		twist = set_twist(-.3, chicken_turn_direction * 1 * MAX_ROTATION_SPEED)
		
		halt_time = halt_time - .3
 
	pub.publish(twist)


if __name__ == '__main__':
    global robot_number
    import sys
    try:
#        robot1 = sys.argv[1]
#        robot2 = sys.argv[2]
#        print sys.argv[1][-1]
        robot_number = sys.argv[1][:-1]
        robot_type = sys.argv[1][-1]
    except:

        print 'ERROR: no robot_name provided, try: \n$$ python stdr_random_walk.py robot[XX] '
        exit()
    if robot_type == 'C':
        print 'robot', robot_number, ' started to wander around as a COOPERATOR'       
        COOPERATION = True
    else:
        print 'robot', robot_number, ' started to wander around as a DEFECTOR'
        COOPERATION = False

    listener()
    rospy.spin()    

    

