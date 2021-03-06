#!/usr/bin/env python
import rospy
import numpy as np
import math as math
import copy as copy
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from ar_track_alvar_msgs.msg import AlvarMarkers

#global variables
bumper_list = [0, 0, 0]

aprilTagPose=[0,0,0]

controller_vel = [0.0, 0.0] # linear_x, angular_z
controller_buttons = [0, 0, 0, 0, 0, 0] # A, B, X, Y, Left Sholder

positional_data = [None, None] # x, y

linear_x_current = 0.0
desired_vel_x = 0.3
kp = 0.1
ki = 0.7
kd = 0
rate_sec = 1.0/20

kp_angle = 0.2

def bumperCallback(data):
    # bumper 2: right, bumper 1: middle, bumper 0: left
    global bumper_list
    bumper_list[data.bumper] = data.state

def odomCallback(data):
    #print data.twist.twist
    global linear_x_current
    global positional_data
    positional_data[0] = data.pose.pose.position.x
    positional_data[1] = data.pose.pose.position.y
    linear_x_current = data.twist.twist.linear.x
    
def joyCallback(data):
    #print data.axes
    global controller_vel
    global controller_buttons
    controller_vel[0] = data.axes[4]/2 # Linear velocity x
    controller_vel[1] = data.axes[3] # Angular velocity z

    controller_buttons[0] = data.buttons[0] # A - green button
    controller_buttons[1] = data.buttons[1] # B - red button
    controller_buttons[2] = data.buttons[2] # X - blue button
    controller_buttons[3] = data.buttons[3] # Y - yellow button
    controller_buttons[4] = data.buttons[4] # Left shoulder button
    controller_buttons[5] = data.buttons[5] # Right shoulder button
    #print "Linear x:", linear_x

def aprilTagCallback(data):
    global aprilTagPose
    try:
        marker=msg.markers[0]
        pose=marker.pose.pose.position
        aprilTagPose[0]=pose.x
        aprilTagPose[1]=pose.y
        aprilTagPose[2]=pose.z
    except:
        aprilTagPose=[-1,-1,-1] #Lost the April Tag. Need to start spinning


def main():
    vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=1)
    rospy.init_node('teleop_pid_bumper', anonymous=True)
    rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, bumperCallback)
    rospy.Subscriber("/odom", Odometry, odomCallback)
    rospy.Subscriber("/joy", Joy, joyCallback)
    rospy.Subscriber("/ar_pose_marker", AlvarMarker, aprilTagCallback)
    rate = rospy.Rate(1/rate_sec)
    vel = Twist()

    global bumper_list
    global linear_x_current
    global desired_vel_x
    global controller_vel
    global controller_buttons
    global kp
    global ki
    global kd
    global kp_angle

    global positional_data

    vel.angular.z = 0.0
    vel.linear.x = 0.0

    error = 0
    integral = 0
    prev_error=0

    count = 0

    goal_vel = 0.3
    goal_point = [1.5, 1.5]


    previous_pos = copy.copy(positional_data)
    previous_angle = 0.0
    current_angle = 0.0

    have_not_set_goal = True
    while not rospy.is_shutdown():
        if have_not_set_goal == True:
             if positional_data[0] == None:
                  continue

             print "It got here"
             goal_point = [1.5 + positional_data[0], 1.5 +  positional_data[1]]
             previous_pos = copy.copy(positional_data)
             have_not_set_goal = False

	goal_vel = desired_vel_x
        # Controller take over
	if (controller_buttons[4] == 1):
             goal_vel = controller_vel[0]

        error = goal_vel - linear_x_current
        integral += error*(rate_sec)
        derivative = (error-prev_error)/(rate_sec)

        output = kp*error + ki*integral + kd*derivative
        prev_error=error
        vel.linear.x = output
        
        previous_angle = math.atan2(positional_data[1] - previous_pos[1], positional_data[0] - previous_pos[0])
        
        goal_angle = math.atan2(goal_point[1] - positional_data[1], goal_point[0] - positional_data[0])
        gamma = (goal_angle - previous_angle)*kp_angle
        current_angle = output/0.1*math.tan(gamma)
        
#        print "Current angle: ", current_angle
        print "Current pos: ", positional_data
        print "Goal point ", goal_point
        

        vel.angular.z = current_angle
	
             # negative angular.z is turning left
             # positive angular.z is turning right
        if bumper_list[1] == 1:
             integral = 0
             previous_error = 0

             vel.linear.x = 0.0
             vel.angular.z = 1.0

        elif bumper_list[0]:
             integral = 0
             previous_error = 0

             vel.linear.x = 0.0
             vel.angular.z = -1.0

        elif bumper_list[2]:
             integral = 0
             previous_error = 0

             vel.linear.x = 0.0
             vel.angular.z = 1.0

	if (controller_buttons[4] == 1):
             vel.angular.z = controller_vel[1]

        if (controller_buttons[5] == 1):
             vel.linear.x = 0
             integral = 0
             prev_error = 0

        # Cruise Control take over
        if (controller_buttons[0] == 1):
             desired_vel_x = 0.3
        elif (controller_buttons[1] == 1):
             desired_vel_x = 0.0
        elif (controller_buttons[2] == 1):
             desired_vel_x = 0.5
        elif (controller_buttons[3] == 1):
             desired_vel_x = 0.6

        previous_pos = copy.copy(positional_data)
        if math.fabs(goal_point[0] - positional_data[0]) < 0.2:
             if math.fabs(goal_point[1] - positional_data[1]) < 0.2:
                  goal_vel = 0.0
                  vel.linear.x = 0.0
                  vel.angular.z = 0.0

        vel_pub.publish(vel)
        rate.sleep()


if __name__ == "__main__":
    main()
