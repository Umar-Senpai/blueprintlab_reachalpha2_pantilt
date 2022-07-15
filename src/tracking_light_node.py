#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rospy 
import cv2

from sensor_msgs.msg import Image as Imagemsg
from cv_bridge import CvBridge, CvBridgeError
from tracking_light_app import track_light
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA


class ObjectDetector: 
    def __init__(self):
        self.bridge = CvBridge()

        ### SUBSCRIBER
        self.camera_sub  = rospy.Subscriber('/usb_cam/image_raw',Imagemsg,self.callback)
        # self.camera_sub  = rospy.Subscriber('/camera_gripper_prep/image_raw',Imagemsg,self.callback)

        ### PUBLISHER
        self.pub_center = rospy.Publisher('modem_center', Marker, queue_size=10)

        ### INIT VARIABLES
        self.prevImage = None
        self.x_center = 0
        self.y_center = 0

    def callback(self,data):
        try: 
            image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        except CvBridgeError as e:
            print("CvBridge could not convert images from realsense to opencv")
        height,width, channels = image.shape
        annotated, x_center, y_center = track_light(image, self.x_center, self.y_center, self.prevImage)
        self.prevImage = image
        self.x_center = x_center
        self.y_center = y_center

        camera_id    = "realsense_link"

        msg = Marker()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = camera_id
        msg.type = msg.ARROW
        msg.action = msg.ADD
        msg.pose.position.x = x_center
        msg.pose.position.y = y_center
        msg.pose.position.z = 0
        msg.pose.orientation.x = 0
        msg.pose.orientation.y = 0
        msg.pose.orientation.z = 0
        msg.pose.orientation.w = 1
        msg.scale.x = 0.08
        msg.scale.y = 0.016
        msg.scale.z = 0.016
        color=[1,0,0,1]
        c = ColorRGBA(color[0],color[1],color[2],color[3])
        msg.color = c

        self.pub_center.publish(msg)

        cv2.imshow("Image window", annotated)
        cv2.waitKey(3)
        
    print('Object detector running')


if __name__ == '__main__':
 
    rospy.init_node('tracking_light_node')
    node = ObjectDetector()
    rospy.spin()

