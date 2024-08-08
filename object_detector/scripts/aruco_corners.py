#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from object_detector.msg import Corners  # Import the custom message
import cv2

class Detector:
    def __init__(self):
        rospy.init_node('aruco_detector', anonymous=True)
        self.bridge = CvBridge()
        self.image_subscriber = rospy.Subscriber('/quad_f450_camera/camera_link/raw_image', Image, self.image_callback)
        self.corners_publisher = rospy.Publisher('/corners', Corners, queue_size=10)  # Define the publisher
        
        # Define ArUco dictionary and parameters
        self.desired_aruco_dictionary = "DICT_4X4_1000"
        self.ARUCO_DICT = {
            "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
            "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
            "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
            "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
            "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
            "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
            "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
            "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
            "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
            "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
            "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
            "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
            "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
            "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
            "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
            "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
            "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
        }

        self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(self.ARUCO_DICT[self.desired_aruco_dictionary])
        self.aruco_parameters = cv2.aruco.DetectorParameters()

    def image_callback(self, img_msg):
        # Convert the ROS image message to an OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")

        # Detect ArUco marker in the image
        (corners, ids, rejected) = cv2.aruco.detectMarkers(cv_image, self.aruco_dictionary, parameters=self.aruco_parameters)

        # Check that at least one ArUco marker was detected
        if len(corners) > 0:
            ids = ids.flatten()

            # Loop over the detected ArUco corners
            for (marker_corner, marker_id) in zip(corners, ids):
                if marker_id == 80:
                    # Extract the marker corners
                    corners = marker_corner.reshape((4, 2))
                    (top_left, top_right, bottom_right, bottom_left) = corners

                    # Convert the (x, y) coordinate pairs to integers
                    top_right = (int(top_right[0]), int(top_right[1]))
                    bottom_right = (int(bottom_right[0]), int(bottom_right[1]))
                    bottom_left = (int(bottom_left[0]), int(bottom_left[1]))
                    top_left = (int(top_left[0]), int(top_left[1]))

                    # Calculate the centroid of the ArUco marker
                    center_x = int((top_left[0] + bottom_right[0]) / 2)
                    center_y = int((top_left[1] + bottom_right[1]) / 2)

                    # Create and populate the Corners message
                    corners_msg = Corners()
                    corners_msg.TopLeftX = top_left[0]
                    corners_msg.TopLeftY = top_left[1]
                    corners_msg.TopRightX = top_right[0]
                    corners_msg.TopRightY = top_right[1]
                    corners_msg.BottomLeftX = bottom_left[0]
                    corners_msg.BottomLeftY = bottom_left[1]
                    corners_msg.BottomRightX = bottom_right[0]
                    corners_msg.BottomRightY = bottom_right[1]
                    corners_msg.CenterX = center_x
                    corners_msg.CenterY = center_y

                    # Publish the corners message
                    self.corners_publisher.publish(corners_msg)

                    # Draw the bounding box of the ArUco detection
                    cv2.line(cv_image, top_left, top_right, (0, 255, 0), 2)
                    cv2.line(cv_image, top_right, bottom_right, (0, 255, 0), 2)
                    cv2.line(cv_image, bottom_right, bottom_left, (0, 255, 0), 2)
                    cv2.line(cv_image, bottom_left, top_left, (0, 255, 0), 2)

                    # Draw the ArUco marker ID on the video frame
                    cv2.putText(cv_image, str(marker_id),
                                (top_left[0], top_left[1] - 15),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.5, (0, 255, 0), 2)
        
        # Display the image
        cv2.imshow('Camera Feed', cv_image)
        cv2.waitKey(1)  # Adjust the delay as needed

if __name__ == '__main__':
    try:
        camera_subscriber = Detector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
