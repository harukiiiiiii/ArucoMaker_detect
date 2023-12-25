#! /usr/bin/env python

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from std_msgs.msg import Float64MultiArray
from tf.transformations import quaternion_matrix
import math

class ARPose(object):
    def __init__(self):
        #topic_name = /ar_pose_marker type = AlvarMarkers
        self.sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.callback_alvmarker)
        self.pub = rospy.Publisher("/liner_pub",Float64MultiArray,queue_size=1)
    
    def callback_alvmarker(self, markers):
        rospy.loginfo("num: %s",2)
        for m in markers.markers:
            rospy.loginfo("num: %s",3)
            marker_id = m.id
            if marker_id != -1:
                marker_pose = m.pose.pose
                pos = marker_pose.position
                ori = marker_pose.orientation
                # rospy.loginfo("marker[%d] position (x,y,z) = (%3.3f: %3.3f: %3.3f), orientation (x,y,z,w) = (%3.3f: %3.3f: %3.3f: %3.3f)" 
                #               % (marker_id, pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w))
                matrix = quaternion_matrix([ori.w, ori.x, ori.y, ori.z])

                liner_x = (pos.x**2+pos.y**2)
                liner_z = matrix[0][2]

                rospy.loginfo("liner_x = %3.3f,liner_z = %s[rad]" % (liner_x,liner_z))                

                liner = Float64MultiArray(data=[liner_x,liner_z])

                self.publish(liner)
        return
    
    def publish(self,data):
        self.pub.publish(data)

    def printff(self):
        rospy.loginfo("num: %s",1)
        return


if __name__ == '__main__':
    rospy.init_node('ar_pose_changer')
    ar = ARPose()
    ar.printff()
    rospy.spin()