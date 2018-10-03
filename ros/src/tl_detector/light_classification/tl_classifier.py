from styx_msgs.msg import TrafficLight
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class TLClassifier(object):
    def __init__(self):
        # TODO load classifier
        self.image_pub = rospy.Publisher("image_result", Image, queue_size=1)
        self.bridge = CvBridge()

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # TODO implement light color prediction

        # Reference: https://solarianprogrammer.com/2015/05/08/detect-red-circles-image-using-opencv/?
        image_src = image.copy()
        image_hsv = cv2.cvtColor(image_src, cv2.COLOR_BGR2HSV)
        lower_red_hue_range = cv2.inRange(image_hsv, np.array([0, 100, 100]), np.array([10, 255, 255]))
        upper_red_hue_range = cv2.inRange(image_hsv, np.array([160, 100, 100]), np.array([180, 255, 255]))
        image_red_hue = cv2.addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0)

        image_blurred = cv2.GaussianBlur(image_red_hue, (9, 9), 2)
        image_out = cv2.cvtColor(image_blurred, cv2.COLOR_GRAY2BGR)

        circles = cv2.HoughCircles(image_blurred, cv2.HOUGH_GRADIENT, dp=0.5, minDist=20,
                                   param1=50, param2=30, minRadius=4, maxRadius=100)

        if circles is not None:
            print("Detected {} red traffic lights".format(len(circles[0, :])))
            for (x, y, r) in circles[0, :]:
                print("(x={}, y={}, r={})".format(x, y, r))
                # Draw the outer circle
                cv2.circle(image_out, (x, y), r, (0, 255, 0), 3)
                # Draw the center of the circle
                # cv2.circle(image_out, (x, y), 2, (0, 0, 255), 3)
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(image_out, "bgr8"))
            return TrafficLight.RED
        else:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(image_out, "bgr8"))

        return TrafficLight.UNKNOWN
