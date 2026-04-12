#!/usr/bin/env python3

import os

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from ultralytics import YOLO
import message_filters
import numpy as np
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, PointCloud2, PointField
from sloam_msgs.msg import SyncPcOdom
import yaml
from ament_index_python.packages import get_package_share_directory

bridge = CvBridge()

# observation 1: in the bag file, color and depth images can go out of sync
# observation 2: might be able to use aligned depth?


class SemDetection(Node):
    def __init__(self) -> None:
        super().__init__("sem_detection")

        self.prev_time = self.get_clock().now()
        self.model_path = os.path.join(
            get_package_share_directory("object_modeller"),
            "models", "yolov8x-seg.pt")
        self.yolo = YOLO(self.model_path)

        ##################################### PARAMS / CONFIG #####################################
        self.sim = False
        # RealSense D435i camera intrinsics
        self.declare_parameter("fx", 603.7166748046875)
        self.declare_parameter("fy", 603.9064331054688)
        self.declare_parameter("cx", 314.62518310546875)
        self.declare_parameter("cy", 244.9166717529297)
        self.declare_parameter("k_depth_scaling_factor", 1000.0)
        self.declare_parameter("desired_rate", 2.0)
        self.declare_parameter("confidence_threshold", 0.4)
        self.declare_parameter("rgb_topic", "/camera/color/image_raw/")
        self.declare_parameter("depth_topic", "/camera/depth/image_rect_raw/")
        self.declare_parameter("aligned_depth_topic",
                               "/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("odom_topic",
                               "/dragonfly67/quadrotor_ukf/control_odom")
        self.declare_parameter("sync_odom_measurements", True)
        self.declare_parameter("sync_pc_odom_topic",
                               "/sem_detection/sync_pc_odom")
        self.declare_parameter("pc_topic", "/sem_detection/pointcloud")

        self.color_fx = self.get_parameter("fx").value
        self.color_fy = self.get_parameter("fy").value
        self.color_cx = self.get_parameter("cx").value
        self.color_cy = self.get_parameter("cy").value
        self.k_depth_scaling_factor = self.get_parameter(
            "k_depth_scaling_factor").value

        if self.sim:
            # Note: Somehow the cv bridge change the raw depth value to meter
            # So we should not apply depth scale to the depth image again
            self.depth_scale = 1
        else:
            self.depth_scale = 1 / self.k_depth_scaling_factor

        self.desired_rate = self.get_parameter("desired_rate").value

        self.confidence_threshold = self.get_parameter(
            "confidence_threshold").value
        ##################################### PARAMS / CONFIG #####################################

        print("Depth scale: ", self.depth_scale)
        print("fx: ", self.color_fx)
        print("fy: ", self.color_fy)
        print("cx: ", self.color_cx)
        print("cy: ", self.color_cy)

        self.rgb_topic = self.get_parameter("rgb_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.aligned_depth_topic = self.get_parameter(
            "aligned_depth_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.sync_odom_measurements = self.get_parameter(
            "sync_odom_measurements").value
        self.sync_pc_odom_topic = self.get_parameter("sync_pc_odom_topic").value
        self.pc_topic = self.get_parameter("pc_topic").value

        # Subscriber and publisher
        self.rgb_sub = message_filters.Subscriber(self, Image, self.rgb_topic)

        # Subscribe to the depth image
        self.depth_sub = message_filters.Subscriber(
            self, Image, self.depth_topic)
        self.aligned_depth_sub = message_filters.Subscriber(
            self, Image, self.aligned_depth_topic)
        self.odom_sub = message_filters.Subscriber(
            self, Odometry, self.odom_topic)

        self.pc_pub_ = self.create_publisher(PointCloud2, self.pc_topic, 1)
        self.synced_pc_pub_ = self.create_publisher(
            SyncPcOdom, self.sync_pc_odom_topic, 1)

        # Synchronize the two image topics with a time delay of 0.1 seconds
        if (self.sync_odom_measurements):
            self.get_logger().info("Syncing rgb, aligned depth and odom")
            # ApproximateTimeSynchronizer to allow for 0.01s time difference
            ts = message_filters.ApproximateTimeSynchronizer(
                [self.rgb_sub, self.aligned_depth_sub, self.odom_sub], 10, 0.05)
            ts.registerCallback(self.rgb_aligned_depth_odom_callback)
        else:
            ts = message_filters.TimeSynchronizer(
                [self.rgb_sub, self.aligned_depth_sub], 5)
            ts.registerCallback(self.rgb_aligned_depth_callback)

        self.pc_fields_ = self.make_fields()

        try:
            scan2shape_share = get_package_share_directory("scan2shape_launch")
        except Exception:
            scan2shape_share = ""
        self.cls_config_path = os.path.join(
            scan2shape_share, "config",
            "process_cloud_node_indoor_cls_info.yaml")

        with open(self.cls_config_path, "r") as file:
            self.cls_data_all = yaml.load(file, Loader=yaml.FullLoader)

        # convert the "table" to "dining table" for yolov8
        self.cls = {}
        for key, value in self.cls_data_all.items():
            if key == "table":
                self.cls["dining table"] = value["id"]
            else:
                self.cls[key] = value["id"]

    def _elapsed_since(self, prev_time):
        now = self.get_clock().now()
        return (now - prev_time).nanoseconds * 1e-9

    def rgb_aligned_depth_odom_callback(self, rgb, aligned_depth, odom):
        if self._elapsed_since(self.prev_time) < 1.0 / self.desired_rate:
            self.get_logger().info(
                f"Time elapsed since last depth rgb callback is: {self._elapsed_since(self.prev_time)}",
                throttle_duration_sec=3)
            self.get_logger().info(
                f"Skipping current depth image to get desired rate of {self.desired_rate}",
                throttle_duration_sec=3)
            return
        else:
            self.prev_time = self.get_clock().now()

        try:
            # Convert ROS image message to OpenCV image
            # Note: Somehow the cv bridge change the raw depth value to meter
            # So we should not apply depth scale to the depth image again
            color_img = bridge.imgmsg_to_cv2(rgb, "bgr8")
            depth_img = bridge.imgmsg_to_cv2(
                aligned_depth, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(str(e))
            return

        # 1. detect semantics
        # Perform instance segmentation using YOLOv8
        detections = self.yolo.predict(color_img, show=False)

        # 2. open img_size * 2 array save class and id
        label = np.zeros([color_img.shape[0], color_img.shape[1]])
        id = np.zeros([color_img.shape[0], color_img.shape[1]])
        conf = np.zeros([color_img.shape[0], color_img.shape[1]])

        # 3. go though all masks, fill them in class and id
        if len(detections[0]) != 0:
            for detection in detections:
                num_obj = detection.masks.shape[0]
                for i in range(num_obj):
                    cls_int = int(detection.boxes.cls[i])
                    cls_str = self.yolo.names[cls_int]
                    cur_mask = detection.masks.masks[i, :, :].cpu().numpy()

                    mask_pos = np.where(cur_mask != 0)
                    label[mask_pos] = self.get_cls_label(cls_str)
                    id[mask_pos] = i + 1
                    conf[mask_pos] = float(detection.boxes.conf[i])

        # Create a grid of pixel coordinates
        u, v = np.meshgrid(np.arange(depth_img.shape[1]),
                           np.arange(depth_img.shape[0]))
        u = u.astype(np.float32)
        v = v.astype(np.float32)
        d = depth_img.flatten()
        d = d * self.depth_scale
        x = (u.flatten() - self.color_cx) * d / self.color_fx
        y = (v.flatten() - self.color_cy) * d / self.color_fy
        z = d
        x_pt = x.reshape(-1, depth_img.shape[1])
        y_pt = y.reshape(-1, depth_img.shape[1])
        z_pt = z.reshape(-1, depth_img.shape[1])
        x_pt = x_pt[..., None]
        y_pt = y_pt[..., None]
        z_pt = z_pt[..., None]
        points = np.concatenate((x_pt, y_pt, z_pt), axis=2)
        # 5. Stack labels, id, conf
        label = label[..., None]
        id = id[..., None]
        conf = conf[..., None]
        pc_data = np.concatenate(
            (points, label, id, conf), axis=2).astype(np.float32)

        self.make_fields()

        # 6. publish point cloud
        sync_pc_odom_msg = SyncPcOdom()
        sync_pc_odom_msg.header = Header()
        sync_pc_odom_msg.header.stamp = odom.header.stamp
        sync_pc_odom_msg.header.frame_id = "camera"

        pc_msg = PointCloud2()
        header = Header()
        pc_msg.header = header
        pc_msg.header.stamp = odom.header.stamp
        pc_msg.header.frame_id = "camera"
        self.get_logger().warn(
            "Hard coding segmented point cloud frame_id to \"camera\"",
            throttle_duration_sec=5)
        pc_msg.width = int(color_img.shape[1])
        pc_msg.height = int(color_img.shape[0])
        pc_msg.point_step = 24
        pc_msg.row_step = pc_msg.width * pc_msg.point_step
        pc_msg.fields = self.pc_fields_
        pc_msg.data = pc_data.tobytes()
        sync_pc_odom_msg.cloud = pc_msg
        sync_pc_odom_msg.odom = odom
        self.get_logger().info(
            "Published synced point cloud odom msg",
            throttle_duration_sec=3)
        self.synced_pc_pub_.publish(sync_pc_odom_msg)
        self.pc_pub_.publish(pc_msg)

    def rgb_aligned_depth_callback(self, rgb, aligned_depth):
        if self._elapsed_since(self.prev_time) < 1.0 / self.desired_rate:
            self.get_logger().info(
                f"Time elapsed since last depth rgb callback is: {self._elapsed_since(self.prev_time)}",
                throttle_duration_sec=3)
            self.get_logger().info(
                f"Skipping current depth image to get desired rate of {self.desired_rate}",
                throttle_duration_sec=3)
            return
        else:
            self.prev_time = self.get_clock().now()

        try:
            color_img = bridge.imgmsg_to_cv2(rgb, "bgr8")
            depth_img = bridge.imgmsg_to_cv2(
                aligned_depth, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(str(e))
            return

        # 1. detect semantics
        detections = self.yolo.predict(color_img, show=False)

        label = np.zeros([color_img.shape[0], color_img.shape[1]])
        id = np.zeros([color_img.shape[0], color_img.shape[1]])
        conf = np.zeros([color_img.shape[0], color_img.shape[1]])

        if len(detections[0]) != 0:
            for detection in detections:
                num_obj = detection.masks.shape[0]
                for i in range(num_obj):
                    cls_int = int(detection.boxes.cls[i])
                    cls_str = self.yolo.names[cls_int]
                    cur_mask = detection.masks.masks[i, :, :].cpu().numpy()

                    mask_pos = np.where(cur_mask != 0)
                    label[mask_pos] = self.get_cls_label(cls_str)
                    id[mask_pos] = i + 1
                    conf[mask_pos] = float(detection.boxes.conf[i])

        u, v = np.meshgrid(np.arange(depth_img.shape[1]),
                           np.arange(depth_img.shape[0]))
        u = u.astype(np.float32)
        v = v.astype(np.float32)
        d = depth_img.flatten()
        d = d * self.depth_scale
        x = (u.flatten() - self.color_cx) * d / self.color_fx
        y = (v.flatten() - self.color_cy) * d / self.color_fy
        z = d
        x_pt = x.reshape(-1, depth_img.shape[1])
        y_pt = y.reshape(-1, depth_img.shape[1])
        z_pt = z.reshape(-1, depth_img.shape[1])
        x_pt = x_pt[..., None]
        y_pt = y_pt[..., None]
        z_pt = z_pt[..., None]
        points = np.concatenate((x_pt, y_pt, z_pt), axis=2)

        label = label[..., None]
        id = id[..., None]
        conf = conf[..., None]
        pc_data = np.concatenate(
            (points, label, id, conf), axis=2).astype(np.float32)

        self.make_fields()

        pc_msg = PointCloud2()
        header = Header()
        pc_msg.header = header
        pc_msg.header.stamp = aligned_depth.header.stamp
        self.get_logger().warn(
            "Hard coding segmented point cloud frame_id to \"camera\"",
            throttle_duration_sec=5)
        pc_msg.header.frame_id = "camera"
        pc_msg.width = int(color_img.shape[1])
        pc_msg.height = int(color_img.shape[0])
        pc_msg.point_step = 24
        pc_msg.row_step = pc_msg.width * pc_msg.point_step
        pc_msg.fields = self.pc_fields_
        pc_msg.data = pc_data.tobytes()
        self.pc_pub_.publish(pc_msg)
        self.get_logger().info(
            "Published synced point cloud odom msg",
            throttle_duration_sec=3)

    def get_cls_label(self, cls_str):
        if cls_str in self.cls.keys():
            return self.cls[cls_str]
        else:
            return 0  # 4

    def make_fields(self):
        fields = []
        field = PointField()
        field.name = 'x'
        field.count = 1
        field.offset = 0
        field.datatype = PointField.FLOAT32
        fields.append(field)

        field = PointField()
        field.name = 'y'
        field.count = 1
        field.offset = 4
        field.datatype = PointField.FLOAT32
        fields.append(field)

        field = PointField()
        field.name = 'z'
        field.count = 1
        field.offset = 8
        field.datatype = PointField.FLOAT32
        fields.append(field)

        field = PointField()
        field.name = 'intensity'
        field.count = 1
        field.offset = 12
        field.datatype = PointField.FLOAT32
        fields.append(field)

        field = PointField()
        field.name = 'id'
        field.count = 1
        field.offset = 16
        field.datatype = PointField.FLOAT32
        fields.append(field)

        field = PointField()
        field.name = 'confidence'
        field.count = 1
        field.offset = 20
        field.datatype = PointField.FLOAT32
        fields.append(field)
        return fields


def main(args=None):
    rclpy.init(args=args)
    node = SemDetection()
    node.get_logger().info("Semantic detection node init.")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
