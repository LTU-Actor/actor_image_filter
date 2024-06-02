#!/usr/bin/env python3

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from actor_image_filter.cfg import CamFilterConfig

bridge = CvBridge()


# dynamic reconfigure callback
def dyn_rcfg_cb(config, level):
    global config_

    config_ = config

    return config


# image callback
def image_cb(ros_image):
    global config_
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        print(e)
        return

    # convert to binary
    gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    ret,bw_image = cv2.threshold(gray_image, config_.threshold, 255, cv2.THRESH_BINARY)
    
    
    debug_image = cv2.cvtColor(bw_image, cv2.COLOR_GRAY2BGR)
    output_image = bridge.cv2_to_imgmsg(debug_image, encoding='rgb8')
    image_pub.publish(output_image)


if __name__ == "__main__":
    rospy.init_node("image_filter", anonymous=True)

    image_pub = rospy.Publisher("bw_image", Image, queue_size=1)

    rospy.Subscriber(
        rospy.get_param("~imgtopic_name"), Image, callback=image_cb, queue_size=1
    )
    Server(CamFilterConfig, dyn_rcfg_cb)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
