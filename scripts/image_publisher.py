#!/usr/bin/env python3

import os
import cv2
import rospy
import rospkg
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class ImagePublisher():

    def __init__(self):	
        self.raw_images = []

        self.image = Image()
        self.image.header.frame_id = "camera"

        self.bridge = CvBridge()
        self.rospack = rospkg.RosPack()
        self.pkg_path = f'{ self.rospack.get_path("machine_vision_201400ivsm") }/data/images'

        self.rate = rospy.Rate(2)
        self.publisher = rospy.Publisher("/image_raw", Image, queue_size=1000)


    def load_images_from_file(self, imgs_path):
        for filename in os.listdir(imgs_path):
            img = cv2.imread(os.path.join(imgs_path, filename))
            if img is not None:
                image_message = self.bridge.cv2_to_imgmsg(img, "rgb8")
                self.raw_images.append(image_message)

    def publish_raw_images(self):
        if not self.raw_images:
            self.load_images_from_file(self.pkg_path)

        for image_message in self.raw_images:
            self.publisher.publish(image_message)
            self.rate.sleep()

    def run(self):
        while not rospy.is_shutdown():
            self.publish_raw_images()
            # self.rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('image_publisher')
        ipnode = ImagePublisher()
        ipnode.run()
    except rospy.ROSInterruptException:
        pass
