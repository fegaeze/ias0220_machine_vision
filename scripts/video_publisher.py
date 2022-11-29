#!/usr/bin/env python3

import rospy
import rospkg
import cv2 as cv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class VideoPublisher():

    def __init__(self):	
        self.bridge = CvBridge()
        self.rospack = rospkg.RosPack()

        self.video_path = f'{ self.rospack.get_path("machine_vision_201400ivsm") }/data/movingDuck.mp4'
        self.cap = cv.VideoCapture(self.video_path)

        self.rate = rospy.Rate(25)
        self.publisher = rospy.Publisher("/video_stream", Image, queue_size=1000)


    def publish_video_frames(self):
        ret,frame = self.cap.read()

        if ret == True:
            image_message = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.publisher.publish(image_message)

        self.rate.sleep()

            
    def run(self):
        while not rospy.is_shutdown():
            self.publish_video_frames()



if __name__ == '__main__':
    try:
        rospy.init_node('video_publisher')
        vpnode = VideoPublisher()
        vpnode.run()
    except rospy.ROSInterruptException:
        pass
