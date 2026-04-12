#!/usr/bin/env python3
# This file is a modified version of the original code from: https://github.com/PRBonn/lidar-bonnetal

from load_model import Load_Model
from laserscan import LaserScan
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import torch
import torch.backends.cudnn as cudnn

# ros_numpy is not available in ROS2 Jazzy. Use sensor_msgs_py.point_cloud2
# helpers to read PointCloud2 messages instead.
from sensor_msgs_py import point_cloud2 as pc2_py


class Inference(Node):
    def __init__(self):
        super().__init__("inference_node")
        self.node_name = self.get_name()
        ################################ IMPORTANT PARAMS ################################
        self.declare_parameter("desired_frequency", 2)
        self.declare_parameter("num_cpu_threads", 10)
        self.declare_parameter("pc_range_threshold", 20)
        self.declare_parameter("out_of_range_default_position", [0.0, 0.0, 0.0])
        self.declare_parameter("pc_point_step", 16)
        # REQUIRED: path to the RangeNet++ semantic-segmentation model
        # directory. There is no sensible default — pass it via launch
        # argument or via infer_node_params.yaml. Example:
        #     ros2 run scan2shape_launch infer_node.py --ros-args \
        #         -p model_dir:=/path/to/pennovation-darknet-smallest/
        self.declare_parameter("model_dir", "")
        self.declare_parameter("namespace", "/os_node")
        self.declare_parameter("gpu", True)

        self.desired_frequency = self.get_parameter("desired_frequency").value
        num_cpu_threads = self.get_parameter("num_cpu_threads").value
        self.range_threshold = self.get_parameter("pc_range_threshold").value
        self.out_of_range_pts_default_position = np.array(
            self.get_parameter("out_of_range_default_position").value)
        self.pc_point_step = self.get_parameter("pc_point_step").value

        torch.set_num_threads(num_cpu_threads)

        model_directory = self.get_parameter("model_dir").value
        namespace = self.get_parameter("namespace").value
        print("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
        print("[infer_node] desired frequency for inference is: ", self.desired_frequency,
              " Hz. If its a positive value, some point clouds will be skipped for inference. Set to 0 to run inference on every point cloud.")
        print("[infer_node] using namespace: ", namespace,
              " to publish segmented point cloud (should be /quadrotor if running on the robot, otherwise /os_node)")
        print("[infer_node] setting number of CPU threads to: ", num_cpu_threads,
              " for Pytorch CPU inference (should be small if you are running on the robot)")
        print("[infer_node] setting range threshold for final segmented point cloud (for cutting off all far-away points) to: ",
              self.range_threshold,
              " meters. Those cut-off points will be set to have coordinate: ",
              self.out_of_range_pts_default_position)
        print("[infer_node] model directory is: ", model_directory)
        print("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
        ################################ IMPORTANT PARAMS ENDS ################################

        self.last_called_stamp = None

        self.load_obj_ = Load_Model(model_directory)
        self.model_ = self.load_obj_.load_model()
        self.scan_obj_ = LaserScan(project=True,
                                   H=self.load_obj_.arch_configs["dataset"]["sensor"]["img_prop"]["height"],
                                   W=self.load_obj_.arch_configs["dataset"]["sensor"]["img_prop"]["width"],
                                   fov_up=self.load_obj_.arch_configs["dataset"]["sensor"]["fov_up"],
                                   fov_down=self.load_obj_.arch_configs["dataset"]["sensor"]["fov_down"],
                                   range_threshold=self.range_threshold)

        # Set model cuda parameters
        self.gpu_ = False
        if torch.cuda.is_available() and torch.cuda.device_count() > 0 and \
                self.get_parameter("gpu").value:
            cudnn.benchmark = True
            cudnn.fastest = True
            self.gpu_ = True
            self.model_.cuda()
        else:
            self.model_.cpu()

        # Push model into eval mode
        self.model_.eval()
        if self.gpu_:
            torch.cuda.empty_cache()

        self.info_ = None
        self.pc_fields_ = self.make_fields()

        # Subscriber for using point cloud from Faster-LIO
        self.scan_sub_ = self.create_subscription(
            PointCloud2, "/cloud_registered_body", self.pc_cb, 1)
        # Publisher for publishing segmented point cloud
        self.pc_pub_ = self.create_publisher(
            PointCloud2, namespace + "/segmented_point_cloud_no_destagger", 1)
        # Publisher for active SLAM to publish car probability. Currently not used
        # self.prob_pub = self.create_publisher(PointCloud2, namespace + "/os_node/segmented_point_cloud_no_destagger/car_prob", 1)

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
        return fields

    @staticmethod
    def _stamp_to_sec(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def pc_cb(self, msg):
        if self.last_called_stamp is not None:
            if self.desired_frequency > 0:
                if (self._stamp_to_sec(msg.header.stamp) - self._stamp_to_sec(self.last_called_stamp)) > (1.0 / self.desired_frequency):
                    self.last_called_stamp = msg.header.stamp
                else:
                    self.get_logger().warn(
                        "Skipping this point cloud for inference as desired frequency is {} Hz. Set desired frequency to 0 to run inference on every point cloud.".format(self.desired_frequency),
                        throttle_duration_sec=10)
                    return
            else:
                self.get_logger().warn(
                    "Desired frequency is set to 0. Running inference on every point cloud. Set desired frequency to a positive value to limit the inference rate.",
                    throttle_duration_sec=10)
        else:
            self.last_called_stamp = msg.header.stamp

        # store the header in case it get updated before the inference finishes
        header = msg.header

        # Read points from the PointCloud2 with sensor_msgs_py helpers.
        pts = list(pc2_py.read_points(
            msg, field_names=("x", "y", "z", "intensity"), skip_nans=False))
        if len(pts) == 0:
            return
        pts_arr = np.array(pts, dtype=np.float32)
        # pts_arr is a structured ndarray when read_points returns named fields;
        # convert to a plain (N, 4) float array.
        if pts_arr.dtype.names is not None:
            xs = pts_arr['x'].astype(np.float32)
            ys = pts_arr['y'].astype(np.float32)
            zs = pts_arr['z'].astype(np.float32)
            intens = pts_arr['intensity'].astype(np.float32)
            pts_arr = np.stack([xs, ys, zs, intens], axis=1)

        self.pc_width = pts_arr.shape[0]
        self.pc_height = 1
        points_xyz = np.zeros((pts_arr.shape[0], 3))
        points_xyz[:, 0] = pts_arr[:, 0]
        points_xyz[:, 1] = pts_arr[:, 1]
        points_xyz[:, 2] = pts_arr[:, 2]
        points_intensity = pts_arr[:, 3]

        # Load into the LaserScan class to do inference
        self.scan_obj_.open_scan(points_xyz=points_xyz.copy(),
                                 points_intensity=points_intensity)

        # make a tensor of the uncompressed data (with the max num points)
        unproj_n_points = self.scan_obj_.points.shape[0]
        unproj_xyz = torch.full(
            (self.load_obj_.arch_configs["dataset"]["max_points"], 3), -1.0, dtype=torch.float)

        unproj_xyz[:unproj_n_points] = torch.from_numpy(self.scan_obj_.points)
        unproj_range = torch.full(
            [self.load_obj_.arch_configs["dataset"]["max_points"]], -1.0, dtype=torch.float)
        unproj_range[:unproj_n_points] = torch.from_numpy(
            self.scan_obj_.unproj_range)
        unproj_remissions = torch.full(
            [self.load_obj_.arch_configs["dataset"]["max_points"]], -1.0, dtype=torch.float)
        unproj_remissions[:unproj_n_points] = torch.from_numpy(
            self.scan_obj_.remissions)

        # prepare final points
        proj_range = torch.from_numpy(self.scan_obj_.proj_range).clone()
        # projected point cloud xyz - [H,W,3] xyz coord (-1 is no data)
        proj_xyz = torch.from_numpy(self.scan_obj_.proj_xyz).clone()
        proj_remission = torch.from_numpy(
            self.scan_obj_.proj_remission).clone()
        proj_mask = torch.from_numpy(self.scan_obj_.proj_mask)

        proj_x = torch.full([self.load_obj_.arch_configs["dataset"]
                            ["max_points"]], -1, dtype=torch.long)
        proj_x[:unproj_n_points] = torch.from_numpy(self.scan_obj_.proj_x)
        proj_y = torch.full([self.load_obj_.arch_configs["dataset"]
                            ["max_points"]], -1, dtype=torch.long)
        proj_y[:unproj_n_points] = torch.from_numpy(self.scan_obj_.proj_y)

        proj = torch.cat([proj_range.unsqueeze(0).clone(),
                          proj_xyz.clone().permute(2, 0, 1),
                          proj_remission.unsqueeze(0).clone()])

        proj = (proj - torch.tensor(self.load_obj_.arch_configs["dataset"]["sensor"]["img_means"])[:, None, None]
                ) / torch.tensor(self.load_obj_.arch_configs["dataset"]["sensor"]["img_stds"])[:, None, None]

        proj = proj * proj_mask.float()

        with torch.no_grad():
            p_x = proj_x[:unproj_n_points]
            p_y = proj_y[:unproj_n_points]
            proj_range = proj_range[0, :unproj_n_points]
            unproj_range = unproj_range[:unproj_n_points]

            proj = torch.unsqueeze(proj, 0)
            proj_mask = torch.unsqueeze(proj_mask, 0)

            if self.gpu_:
                proj_in = proj.cuda()
                proj_mask = proj_mask.cuda()
                p_x = p_x.cuda()
                p_y = p_y.cuda()
            else:
                proj_in = proj
                proj_mask = proj_mask
                p_x = p_x
                p_y = p_y

            proj_output = self.model_(proj_in, proj_mask)
            if self.gpu_:
                torch.cuda.synchronize()

            # Output probability instead of hard classification
            proj_argmax = proj_output[0].argmax(dim=0)

            # threshold out of range points
            points_xyz[np.linalg.norm(
                points_xyz, axis=1) > self.range_threshold, :] = self.out_of_range_pts_default_position

            pred_np_range_image = proj_argmax.cpu().numpy()
            pred_np_range_image = pred_np_range_image.reshape((-1))
            proj_xyz_range_image = proj_xyz.cpu().numpy()
            proj_xyz_range_image = proj_xyz_range_image.reshape((-1, 3))
            # threshold out of points
            pred_np_range_image[np.linalg.norm(
                proj_xyz_range_image, axis=1) > self.range_threshold] = 0
            # make sure that points do not exist in the originial input PC are labeled as 0
            pred_np_range_image[proj_xyz_range_image[:, 0] == -1] = 0
            proj_xyz_range_image[proj_xyz_range_image[:, 0] == -1] = np.nan
            full_data_range_image = np.hstack(
                (proj_xyz_range_image, pred_np_range_image[:, None])).astype(np.float32)

            pc_msg = PointCloud2()
            pc_msg.header = header
            pc_msg.header.frame_id = "body"
            self.get_logger().warn(
                "Segmented point cloud is currently hardcoded to be published in \"body\" frame. Please change it to the correct frame if needed.",
                throttle_duration_sec=30)
            pc_msg.width = self.pc_width
            pc_msg.height = self.pc_height
            pc_msg.point_step = self.pc_point_step
            pc_msg.row_step = pc_msg.width * pc_msg.point_step
            pc_msg.fields = self.pc_fields_
            pc_msg.data = full_data_range_image.tobytes()
            self.pc_pub_.publish(pc_msg)

            self.get_logger().info(
                "Inference done. Segmented point cloud published.",
                throttle_duration_sec=5)


def main(args=None):
    rclpy.init(args=args)
    node = Inference()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.get_logger().info("Shutting down {}...".format(node.get_name()))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
