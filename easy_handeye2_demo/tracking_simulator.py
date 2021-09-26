# this script emulates the tf output of a tracking system in a hand-eye calibration setup, given the ground-truth
# calibration and arbitrary transforms. the output is consists of the transform between the camera and the marker,
# plus noise
#
# if we are not calibrating but publishing the calibration, but demoing the result, we compute the result according to
# the specified ground truth frame but publish it with respect to the actual result of the calibration (so that the
# outcome of a faulty calibration is faithfully reproduced)
#
# it is assumed that a MoveIt! instance is running, so that the tf frames for a (simulated) robot are present
#
# the calibration transform is the one which is the unknown for the calibration process; the arbitrary transformation
# is the collateral transformation which is ignored during the calibration process, but it is necessary to compute the
# tracking output here

# TODO: output tracking information only when marker within field of view of tracking system to make it more realistic
# (in prism for optical tracking, within radius for EM, ...)

# TODO: more noise models

import rclpy
import rclpy.time
import tf2_ros
import numpy as np
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3, Quaternion, Transform, TransformStamped


class TrackingSimulator(rclpy.node.Node):
    def __init__(self):
        super().__init__('tracking_simulator_node')

        # declare and read parameters
        self.declare_parameter('eye_in_hand')

        self.declare_parameter('robot_base_frame')
        self.declare_parameter('robot_effector_frame')
        self.declare_parameter('tracking_base_frame')
        self.declare_parameter('tracking_marker_frame')

        self.declare_parameter('frequency')
        self.declare_parameter('translation_noise_stdev')
        self.declare_parameter('rotation_noise_stdev')

        self.declare_parameter('base_to_tracking')
        self.declare_parameter('hand_to_tracking')

        self.is_eye_in_hand = self.get_parameter('eye_in_hand').get_parameter_value().bool_value

        self.robot_base_frame = self.get_parameter('robot_base_frame').get_parameter_value().string_value
        self.robot_effector_frame = self.get_parameter('robot_effector_frame').get_parameter_value().string_value
        self.tracking_base_frame = self.get_parameter('tracking_base_frame').get_parameter_value().string_value
        self.tracking_marker_frame = self.get_parameter('tracking_marker_frame').get_parameter_value().string_value
        self.CAMERA_DUMMY_FRAME = self.tracking_base_frame + '_gt'
        self.MARKER_DUMMY_FRAME = self.tracking_marker_frame + '_gt'

        self.frequency = self.get_parameter('frequency').get_parameter_value().integer_value
        self.transl_noise = self.get_parameter('translation_noise_stdev').get_parameter_value().double_value
        self.rot_noise = self.get_parameter('rotation_noise_stdev').get_parameter_value().double_value

        if self.is_eye_in_hand:
            ground_truth_calibration_transformation = self.parse_transformation(
                self.get_parameter('hand_to_tracking').get_parameter_value().string_value)
            arbitrary_marker_placement_transformation = self.parse_transformation(
                self.get_parameter('base_to_tracking').get_parameter_value().string_value)

            self.calibration_origin_frame = self.robot_effector_frame
            self.marker_placement_origin_frame = self.robot_base_frame
        else:
            ground_truth_calibration_transformation = self.parse_transformation(
                self.get_parameter('base_to_tracking').get_parameter_value().string_value)
            arbitrary_marker_placement_transformation = self.parse_transformation(
                self.get_parameter('hand_to_tracking').get_parameter_value().string_value)

            self.calibration_origin_frame = self.robot_base_frame
            self.marker_placement_origin_frame = self.robot_effector_frame

        # set up tf 

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.tfBroadcaster = tf2_ros.TransformBroadcaster(self)
        self.tfStaticBroadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # set the marker placement in the buffer, so that we can measure the tracking output
        # but the transform doesn't show up in tf (more realistic to the viewer, and no loops in the DAG)
        self.arbitrary_transform_msg_stmpd = TransformStamped(
            header=Header(frame_id=self.marker_placement_origin_frame, stamp=self.get_clock().now().to_msg()),
            child_frame_id=self.MARKER_DUMMY_FRAME,
            transform=arbitrary_marker_placement_transformation)
        self.tfBuffer.set_transform_static(self.arbitrary_transform_msg_stmpd, 'tracking_simulator')

        # set the ground truth calibration; during demo this allows us to compute the correct tracking output even if the calibration failed
        calib_gt_msg_stmpd = TransformStamped(header=Header(frame_id=self.calibration_origin_frame),
                                              child_frame_id=self.CAMERA_DUMMY_FRAME,
                                              transform=ground_truth_calibration_transformation)
        self.tfBuffer.set_transform_static(calib_gt_msg_stmpd, 'tracking_simulator')

        # in the loop:
        # - compute the tracking transform by closing the loop
        # - add noise
        # - publish the tracking transform

        self.tracking_transform_msg_stmpd = TransformStamped(header=Header(frame_id=self.tracking_base_frame),
                                                             child_frame_id=self.tracking_marker_frame,
                                                             transform=Transform(translation=Vector3(),
                                                                                 rotation=Quaternion()))

        self.timer = self.create_timer(1 / self.frequency, self.timer_callback)
        self.last_transform_check = self.get_clock().now()

    def timer_callback(self):

        if not self.tfBuffer.can_transform(self.CAMERA_DUMMY_FRAME, self.MARKER_DUMMY_FRAME, rclpy.time.Time()):
            if self.get_clock().now() - self.last_transform_check > rclpy.time.Duration(seconds=1):
                self.get_logger().info('Waiting for tf tree to be connected...')
                self.last_transform_check = self.get_clock().now()
            return

        # measure tracking transform
        measured_tracking_transformation_msg_stmpd = self.tfBuffer.lookup_transform(self.CAMERA_DUMMY_FRAME,
                                                                                    self.MARKER_DUMMY_FRAME,
                                                                                    rclpy.time.Time())

        fuzzy_transform = self.apply_noise(measured_tracking_transformation_msg_stmpd.transform)

        # publish tracking transform
        self.tracking_transform_msg_stmpd.header.stamp = (self.get_clock().now() - rclpy.time.Duration(seconds=0.1)).to_msg()
        self.tracking_transform_msg_stmpd.transform = fuzzy_transform

        self.tfBroadcaster.sendTransform(self.tracking_transform_msg_stmpd)

    @staticmethod
    def parse_transformation(t_str):
        tx, ty, tz, rx, ry, rz, rw = [float(t) for t in t_str.split(' ')[0:7]]
        return Transform(translation=Vector3(x=tx, y=ty, z=tz), rotation=Quaternion(x=rx, y=ry, z=rz, w=rw))

    def apply_noise(self, transformation):
        t = transformation.translation
        q = transformation.rotation
        nt = np.random.normal([t.x, t.y, t.z], self.transl_noise)
        nq = np.random.normal([q.x, q.y, q.z, q.w], self.rot_noise)  # TODO: better noise for rotation
        nq /= np.linalg.norm(nq)
        return Transform(translation=Vector3(x=nt[0], y=nt[1], z=nt[2]),
                         rotation=Quaternion(x=nq[0], y=nq[1], z=nq[2], w=nq[3]))


def main(args=None):
    rclpy.init(args=args)

    tracking_simulator = TrackingSimulator()

    rclpy.spin(tracking_simulator)

    tracking_simulator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
