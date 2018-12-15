#!/usr/bin/env python
import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers

markers=[]

class ARTag:
    def __init__(self,marker):
        self.id=marker.id
        pose=marker.pose.pose
        self.x=pose.x
        self.y=pose.y
        self.z=pose.z
    def getPose(self):
        return [self.x,self.y,self.z]
    def getID(self):
        return self.id

def aprilTagCallback(data):
    global markers
    markers=[]
    try:
        for marker in data.markers:
            m=ARTag(marker)
            markers.append(Marker)
    except:
        aprilTagPose=[-1,-1,-1] #Lost the April Tag. Need to start spinning

def main():
    rate_sec=500;
    global markers
    rate = rospy.Rate(1/rate_sec)
    rospy.Subscriber("/ar_pose_marker", AlvarMarker, aprilTagCallback)
    #Publish to Navigation Stack Node
    while not rospy.is_shutdown():
        for m in markers:
            print "ID: "+m.getID()+"\nPose: "+m.getPose()

