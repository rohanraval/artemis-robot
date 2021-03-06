#!/usr/bin/env python
import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers

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
    print "test"
    print data
    markers=[]
    try:
        for marker in data.markers:
            m=ARTag(marker)
            markers.append(Marker)
    except:
        aprilTagPose=[-1,-1,-1] #Lost the April Tag. Need to start spinning

def main():
    global markers
    print "testing main"
    rospy.init_node("ar_follower")
    rospy.Subscriber('/ar_pose_marker', AlvarMarkers, aprilTagCallback)
    #Publish to Navigation Stack Node
    while not rospy.is_shutdown():
        print "testing is shutdown"
        rospy.sleep(2)


if __name__ == "__main__":
    main()
