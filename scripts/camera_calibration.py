#!/usr/bin/env python3

import math
import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo


class CameraCaliberation():

    def __init__(self):

        self.count = 0
        self.objpoints = [] # 3d point in real world space
        self.imgpoints = [] # 2d points in image plane.

        self.mtx = None
        self.dist = None
        self.rvecs = None
        self.tvecs = None
        self.gray = None

        self.bridge = CvBridge()
        self.image_raw_sub = rospy.Subscriber("/image_raw", Image, self.cc_cb)
        self.image_pub = rospy.Publisher("/image_processed", Image, queue_size=10)
        self.camera_pub = rospy.Publisher("/camera_info", CameraInfo, queue_size=10)   

        self.squareSize = 0.108
        self.objp = np.zeros((6*7,3), np.float32)
        self.objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2) * self.squareSize
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


    def cc_cb(self, msg):
        if self.count < 37:
            cv2_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            self.gray = cv2.cvtColor(cv2_image, cv2.COLOR_BGR2GRAY) 
            ret, corners = cv2.findChessboardCorners(self.gray, (7,6),None)

            if ret == True:
                self.objpoints.append(self.objp)
                corners2 = cv2.cornerSubPix(self.gray,corners,(11,11),(-1,-1), self.criteria)
                self.imgpoints.append(corners2)

                # Draw and display the corners
                img = cv2.drawChessboardCorners(cv2_image, (7,6), corners2,ret)
                image_message = self.bridge.cv2_to_imgmsg(img)
                self.image_pub.publish(image_message)

            self.count += 1

        elif self.count == 37:
            ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(self.objpoints, self.imgpoints, self.gray.shape[::-1], None, None)

            self.mtx = mtx
            self.dist = dist
            self.rvecs = rvecs
            self.tvecs = tvecs

            error = self.calc_reprojection_error()
            rospy.loginfo(error)
            self.count += 1

        else:
            camera_info = self.build_camera_info(msg)
            self.camera_pub.publish(camera_info)


    def calc_reprojection_error(self):
        mean_error = 0

        for i in range(len(self.objpoints)):
            imgpointsKnown,_ = cv2.projectPoints(self.objpoints[i], self.rvecs[i], self.tvecs[i], self.mtx, self.dist)
            error = cv2.norm(self.imgpoints[i], imgpointsKnown, cv2.NORM_L2)/len(imgpointsKnown)
            mean_error += error

        return mean_error/len(self.objpoints)


    def build_camera_info(self, msg):
        camera_info = CameraInfo()

        camera_info.width = msg.width
        camera_info.header = msg.header
        camera_info.height = msg.height
        camera_info.distortion_model = "plumb_bob"

        camera_info.D = self.dist.flatten()
        camera_info.K = self.mtx.flatten()
        camera_info.R = np.eye(3).flatten()
        camera_info.P = np.pad(self.mtx, ((0, 0), (0, 1))).flatten()

        return camera_info 


    @staticmethod
    def run():
        """
        This function keeps the node alive.

        """
        rospy.spin()


if __name__ == '__main__':
    try:
        rospy.init_node('camera_calibration')
        cc_node = CameraCaliberation()
        cc_node.run()
    except rospy.ROSInterruptException:
        pass
