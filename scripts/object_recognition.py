#!/usr/bin/env python3

import rospy
import cv2 as cv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class ObjectRecognition():

    def __init__(self):

        self.color_interest = (255,0,0)

        self.bridge = CvBridge()
        self.video_stream_sub = rospy.Subscriber("/observerbot/camera1/image_raw", Image, self.or_cb)
        self.processed_video_stream_pub = rospy.Publisher("/processed_video_stream", Image, queue_size=10)  
        self.processed_video_two_stream_pub = rospy.Publisher("/processed_video_two_stream", Image, queue_size=10)  


    def or_cb(self, msg):
        raw_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        hsv_frame = cv.cvtColor(raw_frame, cv.COLOR_BGR2HSV)
        hsv_frame = cv.blur(hsv_frame, (3,3))

        # Take every color out and allow only color defined in range!!! (FOR RED)
        masked_frame = cv.inRange(hsv_frame, (-10, 100, 100), (10, 255, 255)) 

        contours,_ = cv.findContours(masked_frame, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        if len(contours) != 0:
            # find the biggest countour (c) by the area
            c = max(contours, key=cv.contourArea)

            # draw a circle around the biggest countour area
            contours_poly = cv.approxPolyDP(c, 3, True)
            centers, radius = cv.minEnclosingCircle(contours_poly)
            cv.circle(raw_frame, (int(centers[0]), int(centers[1])), int(radius), (0, 255, 0), 6)

        # Publish color recognized video
        image_message = self.bridge.cv2_to_imgmsg(raw_frame)
        self.processed_video_stream_pub.publish(image_message)


    @staticmethod
    def run():
        """
        This function keeps the node alive.

        """
        rospy.spin()


if __name__ == '__main__':
    try:
        rospy.init_node('object_recognition')
        or_node = ObjectRecognition()
        or_node.run()
    except rospy.ROSInterruptException:
        pass
