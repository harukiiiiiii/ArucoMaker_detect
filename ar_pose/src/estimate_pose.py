#! /usr/bin/env python

import cv2
from cv2 import aruco
import numpy as np
import time
import math
import rospy
from std_msgs.msg import Float64MultiArray

dict_aruco = aruco.Dictionary_get(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters_create()
marker_length = 0.20
cameraMatrix = np.array(
    [[656.7855625761821, 0, 331.9844553631086],
     [0, 659.6350301131262, 263.6177609622039],
     [0, 0, 1]])
distCoeffs = np.array([0.1176654594098569, -0.15234287159857, 0.005417730962466951, -0.001586813166007648, 0])


class MarkSearch :

    def __init__(self, cameraID):
        self.pub = rospy.Publisher("/liner_pub",Float64MultiArray,queue_size=1)
        self.cap = cv2.VideoCapture(cameraID)

    def get_mark_coordinate(self):
        XYZ = []
        RPY = []
        V_x = []
        V_y = []
        V_z = []

        ret, frame = self.cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, dict_aruco, parameters=parameters)

        if len(corners) ==0:
            return False

        rvec, tvec = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, cameraMatrix, distCoeffs)
        R = cv2.Rodrigues(rvec)[0] 
        R_T = R.T
        T = tvec[0].T

        xyz = np.dot(R_T, - T).squeeze()
        XYZ.append(xyz)
        
        rpy = np.deg2rad(cv2.RQDecomp3x3(R_T)[0])
        RPY.append(rpy)
    
        V_x.append(np.dot(R_T, np.array([1,0,0])))
        V_y.append(np.dot(R_T, np.array([0,1,0])))
        V_z.append(np.dot(R_T, np.array([0,0,1])))

        x,y,z = XYZ[0]#o_camera
        uz,vz,wz=V_z[0]#camera z_axis
        liner = math.sqrt(x**2+y**2+z**2)
        sin_angle = uz / math.sqrt(uz**2+wz**2)
        angle = math.asin(sin_angle)
        print("liner:",liner,"angle:",angle)

        info = Float64MultiArray(data=[liner,angle])
        self.publish(info)

        cv2.aruco.drawDetectedMarkers(frame, corners, ids, (0,255,255))
        cv2.aruco.drawAxis(frame, cameraMatrix, distCoeffs, rvec, tvec, marker_length/2)
        cv2.imshow('frame', frame)

        return True

    def publish(self,data):
        self.pub.publish(data)


if __name__ == "__main__" :
    rospy.init_node('ar_pose_publisher')
    cameraID = 0
    cam0_mark_search = MarkSearch(cameraID)

    try:
        while True:
            #print(' ----- get_mark_coordinate ----- ')
            if not cam0_mark_search.get_mark_coordinate():
                continue

            time.sleep(0.1)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cv2.destroyWindow('frame')
        cap.release()

    except KeyboardInterrupt:
        cv2.destroyWindow('frame')
        cam0_mark_search.cap.release()