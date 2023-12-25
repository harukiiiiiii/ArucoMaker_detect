#! /usr/bin/env python

import cv2
from cv2 import aruco
import numpy as np
import time

class MarkSearch :

    dict_aruco = aruco.Dictionary_get(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters_create()

    def __init__(self, cameraID):
        self.cap = cv2.VideoCapture(cameraID)

    def get_mark_coordinate(self):

        ret, frame = self.cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, dict_aruco, parameters=parameters)

        for num_id in np.ravel(ids) :
            if num_id is None:
                break
            index = np.where(ids == num_id)[0][0]
            cornerUL = corners[index][0][0]
            cornerUR = corners[index][0][1]
            cornerBR = corners[index][0][2]
            cornerBL = corners[index][0][3]

            center = [ (cornerUL[0]+cornerBR[0])/2 , (cornerUL[1]+cornerBR[1])/2 ]

            frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)
            cv2.imshow('frame', frame_markers)

            return center

        return None


if __name__ == "__main__" :

    import cv2
    from cv2 import aruco
    import numpy as np
    import time

    dict_aruco = aruco.Dictionary_get(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters_create()

    cameraID = 0
    cam0_mark_search = MarkSearch(cameraID)

    try:
        while True:
            print(' ----- get_mark_coordinate ----- ')
            print(cam0_mark_search.get_mark_coordinate())
            time.sleep(0.1)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cv2.destroyWindow('frame')
        cap.release()
    except KeyboardInterrupt:
        cv2.destroyWindow('frame')
        cam0_mark_search.cap.release()