#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
from ultralytics import YOLO, SAM
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
import message_filters
from sklearn.decomposition import PCA
from tf.transformations import quaternion_about_axis
import tf
import time
from std_msgs.msg import String

class GPUObjectDetector:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('gpu_object_detector', anonymous=True)
        
        # Load parameters
        self.model_path = "yolov8n.pt" #rospy.get_param("~model_path", "yolov8n.pt")
        self.confidence = 0.1 #rospy.get_param("~confidence", 0.5)
        self.sam_ckpt = "sam2_b.pt" #rospy.get_param("~sam_ckpt")
        #self.object = "cell phone"#rospy.get_param("~object")
        self.requested_object = None

        self.request_sub = rospy.Subscriber(
            '/object_request', String, self._request_cb, queue_size=1
        )

        # Initialize models on GPU
        self.model = YOLO(self.model_path)
        self.sam_model = SAM(model=self.sam_ckpt)
        
        # Initialize CV bridge and tf broadcaster
        self.bridge = CvBridge()
        self.tf_broadcaster = tf.TransformBroadcaster()
        
        # Camera intrinsics
        self.K = None  # Camera matrix
        self.D = None  # Distortion coefficients
        
        # Get camera info once
        rospy.loginfo("Waiting for camera info...")
        camera_info = rospy.wait_for_message('/camera/color/camera_info', CameraInfo)
        self.setup_camera_info(camera_info)
        rospy.loginfo("Received camera info")
        rospy.loginfo("K: ")
        rospy.loginfo(self.K)
        rospy.loginfo("D: ")
        rospy.loginfo(self.D)
        
        # Subscribe to RGB and depth topics
        self.rgb_sub = message_filters.Subscriber('/camera/color/image_raw/compressed', CompressedImage)
        self.depth_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
        
        # Time synchronizer
        ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=10,
            slop=0.1
        )
        ts.registerCallback(self.callback)

    # new
    def _request_cb(self, msg: String):
        """Update which object we’re looking for."""
        self.requested_object = msg.data
        rospy.loginfo(f"[Detector] Now seeking: {self.requested_object!r}")

    def setup_camera_info(self, camera_info):
        """Set up camera intrinsics from camera info message"""
        self.K = np.array(camera_info.K).reshape(3, 3)
        self.D = np.array(camera_info.D)
        
        # Store focal length and principal point
        self.fx = self.K[0, 0]
        self.fy = self.K[1, 1]
        self.cx = self.K[0, 2]
        self.cy = self.K[1, 2]

    def deproject_pixel_to_point(self, pixel_x, pixel_y, depth):
        """
        Convert pixel coordinates and depth to 3D point.
        
        Args:
            pixel_x (float): x coordinate in image
            pixel_y (float): y coordinate in image
            depth (float): depth value in meters
            
        Returns:
            tuple: (X, Y, Z) coordinates in camera frame
        """
        # Apply undistortion if needed (assuming points are already undistorted in this case)
        x = (pixel_x - self.cx) / self.fx
        y = (pixel_y - self.cy) / self.fy
        
        # Calculate 3D coordinates
        X = x * depth
        Y = y * depth
        Z = depth
        
        return (X, Y, Z)

    def callback(self, rgb_msg, depth_msg):
        """Callback for synchronized RGB and depth messages"""
        try:
            # Convert compressed image to CV2 format
            color_image = self.bridge.compressed_imgmsg_to_cv2(rgb_msg)
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg)

            # YOLO inference
            # s_det = time.process_time_ns()
            results = self.model.predict(source=color_image, conf=self.confidence, show=False, verbose=False)
            boxes = results[0].boxes
            # e_det = time.process_time_ns()
            # try:
            #     print(len(boxes), self.model.names[int(boxes[0].cls[0])])
            # except:
            #     print(len(boxes))
            # print("Det time", (e_det - s_det)/10**9)
            # s_seg = time.process_time_ns()
            # Process each detection
            for idx, box in enumerate(boxes):
                class_idx = int(box.cls[0])
                class_name = self.model.names[class_idx]

                print(class_name)

                if self.requested_object is None or class_name != self.requested_object:
                    continue

                # Extract bounding box coordinates
                bbox = box.xyxy[0]
                x1, y1, x2, y2 = map(int, bbox.tolist())

                # Generate mask using SAM2
                sam_results = self.sam_model(color_image, bboxes=bbox.tolist(), verbose=False)
                mask = sam_results[0].masks
                mask = np.asarray(mask.xy)[0]

                # print("Running processing")
                # Process mask and compute orientation
                self.process_mask(mask, depth_image, x1, y1, x2, y2, idx, rgb_msg.header.stamp, rgb_msg.header.frame_id)
            e_seg = time.process_time_ns()
            # print("seg", (e_seg - s_seg)/10**9)
            
        except Exception as e:
            rospy.logerr(f"Error in callback: {str(e)}")

    def process_mask(self, mask, depth_image, x1, y1, x2, y2, idx, timestamp, frame_id):
        """Process mask to compute orientation and position"""
        try:
            # Perform PCA on mask coordinates
            pca = PCA(n_components=2)
            pca.fit(mask)
            components = pca.components_
            
            # Compute orientation from principal component
            major_component = components[0]
            angle_rad = np.arctan2(major_component[1], major_component[0])
            quat = quaternion_about_axis(angle_rad, (0, 0, 1))

            # Compute 3D position
            center_x = (x1 + x2) // 2
            center_y = (y1 + y2) // 2
            
            # Get depth value in meters (assuming depth is in millimeters)
            depth = depth_image[center_y, center_x] / 1000.0
            
            # print(f"Center point: ({center_x}, {center_y}), Depth: {depth}")
            # Convert pixel coordinates to 3D point
            X, Y, Z = self.deproject_pixel_to_point(center_x, center_y, depth)

            # print(X, Y, Z)

            # Broadcast transform if position is valid
            if not np.isnan(X) and not np.isnan(Y) and not np.isnan(Z):
                self.broadcast_tf(X, Y, Z, quat, timestamp, frame_id)

        except Exception as e:
            rospy.logerr(f"Error processing mask: {str(e)}")

    def broadcast_tf(self, x, y, z, quat, timestamp, frame_id):
        """Broadcast object pose as tf transform"""
        self.tf_broadcaster.sendTransform(
            (x, y, z),
            (quat[0], quat[1], quat[2], quat[3]),
            timestamp,
            "detected_object",
            "camera_color_frame_calib"
        )

if __name__ == '__main__':
    try:
        detector = GPUObjectDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass