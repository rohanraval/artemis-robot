#!/usr/bin/env python
import rospy
import numpy as np
import math as math
import copy as copy
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


positional_data = [0, 0]    # our pose
orientation_angle = 0       # our orientation
markers = []                # array of AR tag objects representing the tags in env
marker_seq = 0              # tells you about what data point you are working with
linear_x_current = 0        # our velocity

class ARTag:
    def __init__(self,marker):
        self.id=marker.id
        pose=marker.pose.pose.position
        self.x=pose.x
        self.y=pose.y
        self.z=pose.z
    def getPose(self):
        return [self.x,self.y,self.z]
    def getID(self):
        return self.id


def aprilTagCallback(data):
    global markers
    global marker_seq
    #print data.markers
    #print "April Call Back Data"
    marker_seq = data.header.seq

    markers=[]
    try:
        #print data.markers[0]
        for marker in data.markers:
            #print marker.pose.pose
            m=ARTag(marker)
            markers.append(m)
    except:
        print "exception"
        aprilTagPose=[-1,-1,-1] #Lost the April Tag. Need to start spinning



def odomCallback(data):
    global positional_data
    global orientation_angle
    global marker_seq
    global linear_x_current

    #print "Odometer Call Back"

    positional_data[0] = data.pose.pose.position.x
    positional_data[1] = data.pose.pose.position.y
    orientation_angle = math.atan2(2*(data.pose.pose.orientation.w * data.pose.pose.orientation.z + data.pose.pose.orientation.x * data.pose.pose.orientation.y), 1 - 2*(data.pose.pose.orientation.y * data.pose.pose.orientation.y + data.pose.pose.orientation.z * data.pose.pose.orientation.z))

    linear_x_current = data.twist.twist.linear.x


def main():
    global markers
    global positional_data
    global orientation_angle
    global linear_x_current

    print "testing main"
    vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=1)
    rospy.init_node("artemis")
    rospy.Subscriber("/odom", Odometry, odomCallback)
    rospy.Subscriber('/ar_pose_marker', AlvarMarkers, aprilTagCallback)
    #Publish to Navigation Stack Node HERE
    # LISTEN TO NAVIAGTION STACK OUTPUT HERE
    vel = Twist()

    rate_sec = 0.01
    print rate_sec

    current_marker_seq = 0

    # Temporary following goals
    x = 0
    y = 0

    # PID Variables
    error = 0
    integral = 0
    prev_error = 0
    kp = 0.1
    ki = 0.7
    kd = 0
    kp_angle = 5

    # turning counter
    turning_count = 0
    end_time = 8/rate_sec*1.30
    print end_time

    # Follow distance
    follow_distance = 1 # meters

    # MODES
    # 0: start state
    # 1: Wait for Tag state
    # 2: Follow Tag state
    # 3: Return home state
    mode = 0


    # negative angular.z is turning left
    # positive angular.z is turning right

    while not rospy.is_shutdown():
        vel.linear.x = 0
        vel.angular.z = 0

        # Start State
        if mode == 0:
            print "Start State"
            mode = 1
    
            # make robot turn once
            turning_count += 1    
            if turning_count >= end_time:
                mode = 1
            else:
                vel.angular.z = 3.14159265/4

        # Wait for tag state
        elif mode == 1:
            print "Wait for Tag State"

            # PUT TAG discernment here
            for ar_marks in markers:
                # we are only concerned with the tag of ID = 4
                if ar_marks.getID() == 4:

                    # check if this is actually new data that we are getting
                    # ie. we get data from AR sensor every X seconds, so we don't need to
                    # change the goal that many times, only set it every time we get new data
                    if current_marker_seq != marker_seq:

                        # get pose of and dist to AR tag
                        x = ar_marks.getPose()[0]
                        y = ar_marks.getPose()[1]
                        #print "X, Y: ", x, y
                        distance_to_tag = (x**2 + y**2)**0.5
                        #print "Distance to tag: ", distance_to_tag
                        
                        angle_to_goal = math.atan2(y,x)

                        #print "Angle to goal: ", angle_to_goal/math.pi*180
                        #print "Orientation: ", orientation_angle/math.pi*180
 
                           
                        # reduce dist to tag by 1 since we are basically at the goal if within 1m away
                        if distance_to_tag <= follow_distance:
                            distance_to_tag = 0
                        else:
                            distance_to_tag -= follow_distance

                        print "Distance to tag: ", distance_to_tag
                        x = positional_data[0] + math.cos(orientation_angle + angle_to_goal)*distance_to_tag
                        y = positional_data[1] + math.sin(orientation_angle + angle_to_goal)*distance_to_tag
                        current_marker_seq = copy.copy(marker_seq)

                    mode = 2

            vel.linear.x = 0
            vel.angular.z = 0

        # Follow Tag state
        elif mode == 2:

            print "Follow Tag State"
            
            # PUT TAG discernment here
            for ar_marks in markers:
                if ar_marks.getID() == 4:
                    if current_marker_seq != marker_seq:
                        print "different ---------------------------------------------------"
                        x = ar_marks.getPose()[0]
                        y = ar_marks.getPose()[1]

                        #print "X, Y: ", x, y
                        distance_to_tag = (x**2 + y**2)**0.5
                        #print "Distance to tag: ", distance_to_tag

                        angle_to_goal = math.atan2(y,x)
                        #print "angle to goal: ", angle_to_goal/math.pi*180
                        
                        if distance_to_tag <= follow_distance:
                            distance_to_tag = 0
                        else:
                            distance_to_tag -= follow_distance
                        
                        # goal x and goal y
                        x = positional_data[0] + math.cos(orientation_angle + angle_to_goal)*distance_to_tag
                        y = positional_data[1] + math.sin(orientation_angle + angle_to_goal)*distance_to_tag
                        current_marker_seq = copy.copy(marker_seq)
                else:
                    mode = 1

            # Difference between pose and goal
            x_diff = x - positional_data[0]
            y_diff = y - positional_data[1]
            diff = ((x_diff)**2 + (y_diff)**2)**0.5
            #print "Difference: ", diff, ',', x_diff, ',', y_diff

            margin = 0.1
            # if temporary goal is not met
            
            if (abs(x_diff) > margin) or (abs(y_diff) > margin):

                # Setting the goal velocity to be proportinal to the distance to the goal
                goal_vel = diff
                if goal_vel > 0.1:
                    goal_vel = 0.1

                # PID CONTROLLER
                error = goal_vel - linear_x_current
                integral += error*(rate_sec)
                derivative = (error-prev_error)/(rate_sec)
                output = kp*error + ki*integral + kd*derivative
                prev_error=error
                
                # Finding the angle to the goal point
                angle_to_tag = math.atan2(y_diff, x_diff)
                #print "X: ", x, "Y: ", y
                #print "Distance to Tag: ", distance_to_tag
                #print "Angle to Tag: ", angle_to_tag/math.pi*180
                #print "Orientation Angle: ", orientation_angle/math.pi*180

                angle_array = np.array([(angle_to_tag - orientation_angle + 2*math.pi), (angle_to_tag - orientation_angle - 2*math.pi), (angle_to_tag - orientation_angle)])

                angle = angle_array[np.argmin(abs(angle_array))]
                #print "Angle difference: ", angle 

                vel.linear.x = output
                vel.angular.z = (output)*(angle)* kp_angle

                
                
        elif mode == 3:
            print "Return home state"
        
        vel_pub.publish(vel)
        rospy.sleep(rate_sec)


if __name__ == "__main__":
    main()
