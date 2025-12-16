---
title: "Chapter 3.2: Computer Vision"
description: "Implementing object detection, tracking, and image processing pipelines using OpenCV and ROS 2."
sidebar_position: 2
---

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

# Chapter 3.2: Computer Vision for Robotics

In the last chapter, you learned how to acquire raw data from sensors like cameras. Now, we will learn how to turn those raw pixels into meaningful information. This chapter introduces you to the exciting field of **Computer Vision** and its application in robotics. We will use **OpenCV**, the industry-standard open-source computer vision library, and integrate it seamlessly into the ROS 2 ecosystem to build powerful perception pipelines.

:::tip[Learning Objectives]
*   Integrate the OpenCV library with ROS 2 using the `cv_bridge` package.
*   Perform fundamental image processing tasks like filtering, thresholding, and morphological operations.
*   Understand and implement feature detection algorithms like ORB to find keypoints in images.
*   Apply image segmentation techniques to isolate objects of interest.
*   Build a complete, real-time object detection and tracking node in ROS 2.
:::

## 3.2.1 Bridging ROS 2 and OpenCV with `cv_bridge`

ROS 2 and OpenCV are two different worlds. ROS 2 uses `sensor_msgs/Image` messages to transmit images, while OpenCV uses its own `Mat` data structure (represented as a NumPy array in Python). To work with images in a ROS 2 node, we need a way to convert between these two formats.

This is where `cv_bridge` comes in. It's a ROS 2 package that provides a simple and efficient way to convert `sensor_msgs/Image` messages to OpenCV images and vice-versa.

### Code Example: Using `cv_bridge`

This example expands on our camera subscriber from the previous chapter, showing the explicit use of `cv_bridge` to convert the ROS message to an OpenCV image that we can process.

```python
# T065: Example demonstrating the use of cv_bridge
# src/module3_examples/vision/cv_bridge_example.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class CvBridgeNode(Node):
    def __init__(self):
        super().__init__('cv_bridge_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.publisher = self.create_publisher(Image, '/camera/processed_image', 10)
        self.bridge = CvBridge()
        self.get_logger().info('CvBridge node started.')

    def image_callback(self, msg: Image):
        try:
            # 1. Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return

        # 2. Perform some image processing with OpenCV
        # Example: Convert to grayscale and apply a Gaussian blur
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        blurred_image = cv2.GaussianBlur(gray_image, (5, 5), 0)

        # 3. Convert the processed OpenCV image back to a ROS Image message
        try:
            processed_msg = self.bridge.cv2_to_imgmsg(blurred_image, 'mono8')
            # Publish the processed image
            self.publisher.publish(processed_msg)
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')

        self.get_logger().info('Received, processed, and republished an image.')

def main(args=None):
    rclpy.init(args=args)
    node = CvBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
This "subscribe -> convert -> process -> convert -> publish" pattern is fundamental to almost all ROS 2 vision nodes.

## 3.2.2 Fundamental Image Processing Techniques

Before we can perform high-level tasks like object detection, we often need to pre-process the image to make it cleaner and easier to analyze.

*   **Filtering**: Reduces noise in an image. Common filters include Gaussian blur (smooths the image) and median blur (effective against "salt-and-pepper" noise).
*   **Thresholding**: Converts a grayscale image into a binary image (black and white). This is extremely useful for separating an object from the background based on pixel intensity.
*   **Morphological Operations**: These are operations performed on binary images to modify the shape of objects.
    *   **Erosion**: Shrinks the boundaries of foreground objects.
    *   **Dilation**: Expands the boundaries of foreground objects.
    *   **Opening**: An erosion followed by a dilation. Useful for removing small noise.
    *   **Closing**: A dilation followed by an erosion. Useful for closing small holes inside objects.

## 3.2.3 Feature Detection and Matching

How can a robot recognize the same object from different angles or in different lighting conditions? One powerful way is by finding **features**—distinctive points in an image that are likely to be stable across different viewpoints.

A feature detector finds these keypoints, and a feature descriptor creates a unique "signature" for the region around each keypoint. You can then match descriptors between two images to find corresponding points.

**ORB (Oriented FAST and Rotated BRIEF)** is a popular feature detector and descriptor in robotics because it is fast, rotation-invariant, and free to use (unlike SIFT and SURF).

### Code Example: ORB Feature Detector
```python
# T067: Example of ORB feature detection
# src/module3_examples/vision/orb_detector.py
import cv2
import numpy as np

def detect_orb_features(image):
    # Initialize ORB detector
    orb = cv2.ORB_create(nfeatures=500) # You can specify the number of features to find

    # Find the keypoints and compute the descriptors
    keypoints, descriptors = orb.detectAndCompute(image, None)

    # Draw keypoints on the image
    image_with_keypoints = cv2.drawKeypoints(image, keypoints, None, color=(0, 255, 0), flags=0)
    
    return image_with_keypoints, keypoints, descriptors

# Example usage (not in a ROS node for simplicity)
if __name__ == '__main__':
    # Load a sample image
    # In a real node, this would come from a ROS message
    # For a standalone test, you'd load from file:
    # image = cv2.imread('sample_image.jpg', cv2.IMREAD_GRAYSCALE)
    
    # Create a dummy image for demonstration
    image = np.zeros((480, 640), dtype=np.uint8)
    cv2.putText(image, 'OpenCV', (100, 240), cv2.FONT_HERSHEY_SIMPLEX, 4, (255), 10)

    processed_image, _, _ = detect_orb_features(image)
    cv2.imshow("ORB Features", processed_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
```

## 3.2.4 Image Segmentation

**Image Segmentation** is the process of partitioning an image into multiple segments or regions, often to isolate specific objects.

*   **Traditional Methods (e.g., Color-based Segmentation)**: This involves converting the image to a different color space (like HSV, which is less sensitive to lighting changes) and then applying a color mask to isolate pixels that fall within a specific color range. This is simple and fast, but not very robust.
*   **Deep Learning-based Methods**: Modern approaches use Convolutional Neural Networks (CNNs) to perform segmentation. These models are trained on large datasets and can achieve incredibly accurate and robust results, distinguishing between many different object classes. Popular models include U-Net and Mask R-CNN.

## Hands-on Exercises

1.  **Color-based Object Detector**:
    *   Write a ROS 2 node that subscribes to a camera feed.
    *   Use OpenCV to convert the image from BGR to HSV color space.
    *   Define a color range for a specific object (e.g., a green ball).
    *   Apply a mask to the image to isolate only the object.
    *   Find the contours of the object in the mask and calculate its center point.
    *   Publish the center point as a `geometry_msgs/Point` message.
2.  **Feature Matching**:
    *   Write a node that takes two static images.
    *   Use the ORB detector to find keypoints and descriptors in both images.
    *   Use a feature matcher (like `cv2.BFMatcher`) to find the best matches between the two sets of descriptors.
    *   Draw the matches on the images and display the result.
3.  **Edge Detection Publisher**:
    *   Create a node that subscribes to an image topic.
    *   Use the Canny edge detection algorithm (`cv2.Canny`) to find the edges in the image.
    *   Publish the resulting black-and-white edge image to a new topic. Visualize both the original and the edge image in `rqt_image_view`.

## Capstone Project: Real-time Object Tracker

**Goal**: Build a complete ROS 2 vision pipeline that can detect an object based on its color and track its movement in real-time.

**System Components**:
1.  **`camera_node`**: A node that publishes a live video feed (you can use a webcam or a simulated camera in Gazebo).
2.  **`object_tracker_node`**: This will be your main node.
    *   Subscribes to the raw image topic from the `camera_node`.
    *   Performs color-based segmentation to isolate an object of a specific color (e.g., a red cup).
    *   Uses morphological operations (opening and closing) to clean up the binary mask and remove noise.
    *   Finds the largest contour in the mask, which corresponds to your object.
    *   Calculates the bounding box of the contour.
    *   Draws the bounding box on the original image.
    *   Publishes the processed image with the bounding box to a new topic (e.g., `/object_tracker/image_processed`).
    *   Publishes the center coordinates and size of the bounding box to another topic (e.g., `/object_tracker/bounding_box`).
3.  **Visualization**:
    *   Use `rqt_image_view` to watch both the raw and processed video streams side-by-side.
    *   Use `ros2 topic echo` to monitor the bounding box data.

This capstone project will test your ability to create a complete perception pipeline, from raw pixels to high-level object information, a skill that is fundamental to almost every robotics application.

---
**End of Chapter 3.2**
