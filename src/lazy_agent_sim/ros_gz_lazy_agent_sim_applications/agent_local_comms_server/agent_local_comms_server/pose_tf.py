import math
import select
import socket
import struct
from typing import override

import numpy as np
import rclpy
import rclpy.clock
import rclpy.time
import rclpy.duration
import rclpy.node
from agent_local_comms_server.packets import (
    EpuckHeartbeatPacket,
    EpuckHeartbeatResponsePacket,
    EpuckNeighbourPacket,
    EpuckKnowledgePacket,
)
from socketserver import BaseRequestHandler, ThreadingUDPServer
import threading
import tf2_py
from tf2_ros.buffer import Buffer
import tf2_ros.transform_listener
from lazy_agent_sim_interfaces.msg import (
    EpuckKnowledgePacket as EpuckKnowledgePacketMsg,
    EpuckKnowledgeRecord as EpuckKnowledgeRecordMsg,
    Boundary as BoundaryMsg,
    Centroid as CentroidMsg,
)
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import geometry_msgs.msg
from tf2_ros import TransformBroadcaster, TransformListener
from tf_transformations import translation_matrix, quaternion_matrix, inverse_matrix, translation_from_matrix, quaternion_from_matrix, euler_from_quaternion, compose_matrix, concatenate_matrices
import time


class FramePublisher(rclpy.node.Node):

    def __init__(self):
        super().__init__('pose_tf', parameter_overrides=[
            rclpy.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        # Topic publishing geometry_msgs/msg/PoseStamped msgs
        self.declare_parameter('source_topic_name',
                               rclpy.Parameter.Type.STRING)

        # Frame id of the source topic, must have existing tf to tf_frame_id
        self.declare_parameter('source_frame_id', rclpy.Parameter.Type.STRING)

        # Child frame id of the source topic, must have existing tf to tf_child_frame_id
        self.declare_parameter('source_child_frame_id',
                               rclpy.Parameter.Type.STRING)

        # Frame id of the tf to be published, must have existing tf to source_frame_id
        self.declare_parameter('tf_frame_id', rclpy.Parameter.Type.STRING)

        # Child frame id of the tf to be published, must have existing tf to source_child_frame_id
        self.declare_parameter('tf_child_frame_id',
                               rclpy.Parameter.Type.STRING)

        # Topic republishing geometry_msgs/msg/PoseStamped msgs, optional
        self.declare_parameter('pose_topic_name',
                               rclpy.Parameter.Type.STRING)

        # Frame id of the pose topic
        self.declare_parameter('pose_frame_id', rclpy.Parameter.Type.STRING)

        # Child frame id of the pose topic
        self.declare_parameter('pose_child_frame_id',
                               rclpy.Parameter.Type.STRING)

        # Initialize the transform broadcaster
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
            # qos=QoSProfile(
            #     reliability=ReliabilityPolicy.BEST_EFFORT,
            #     durability=DurabilityPolicy.VOLATILE,
            #     history=HistoryPolicy.KEEP_LAST,
            #     depth=10
            # ),
            # static_qos=QoSProfile(
            #     reliability=ReliabilityPolicy.BEST_EFFORT,
            #     durability=DurabilityPolicy.VOLATILE,
            #     history=HistoryPolicy.KEEP_LAST,
            #     depth=10
            # ),
        )
        self.tf_broadcaster = TransformBroadcaster(self)

        # Publish the pose to the pose topic if enabled
        if self.get_parameter('pose_topic_name').value != "":
            self.get_logger().info("CREATING PUBLISHER")
            self.publisher = self.create_publisher(
                geometry_msgs.msg.PoseStamped,
                self.get_parameter('pose_topic_name').value,
                QoSProfile(
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.VOLATILE,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=10
                ),
            )

        self.subscription = self.create_subscription(
            geometry_msgs.msg.PoseStamped,
            self.get_parameter('source_topic_name').value,
            self.handle_pose_callback,
            qos_profile=QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            ),
        )
        self.subscription  # prevent unused variable warning

    def handle_pose_callback(self, msg: geometry_msgs.msg.PoseStamped):
        # Ignore the frame_id field in the msg, instead use the source_frame_id param

        # source_topic_name - Topic publishing geometry_msgs/msg/PoseStamped msgs
        # source_frame_id - Frame id of the source topic, must have existing tf to tf_frame_id
        # source_child_frame_id - Child frame id of the source topic, must have existing tf from tf_child_frame_id
        # tf_frame_id - Frame id of the tf to be published, must have existing tf from source_frame_id
        # tf_child_frame_id - Child frame id of the tf to be published, must have existing tf to source_child_frame_id
        # pose_topic_name - Topic republishing geometry_msgs/msg/PoseStamped msgs, optional
        # pose_frame_id - Frame id of the pose topic

        # Msg is transform from source_frame_id to source_child_frame_id
        # msg = TF(source_frame_id, tf_frame_id) @ TF(tf_frame_id, tf_child_frame_id) @ TF(tf_child_frame_id, source_child_frame_id)
        # We want the middle one, which is the transform from tf_frame_id to tf_child_frame_id

        # msg = TF(source_frame_id, tf_frame_id) @ TF(tf_frame_id, tf_child_frame_id) @ TF(tf_child_frame_id, source_child_frame_id)
        # TF(source_frame_id, source_child_frame_id) = TF(source_frame_id, tf_frame_id) @ TF(tf_frame_id, tf_child_frame_id) @ TF(tf_child_frame_id, source_child_frame_id)
        # inv(TF(source_frame_id, tf_frame_id)) @ TF(source_frame_id, source_child_frame_id) = TF(tf_frame_id, tf_child_frame_id) @ TF(tf_child_frame_id, source_child_frame_id)
        # inv(TF(source_frame_id, tf_frame_id)) @ TF(source_frame_id, source_child_frame_id) @ inv(TF(tf_child_frame_id, source_child_frame_id)) = TF(tf_frame_id, tf_child_frame_id)
        # TF(tf_frame_id, source_frame_id) @ TF(source_frame_id, source_child_frame_id) @ TF(source_child_frame_id, tf_child_frame_id) = TF(tf_frame_id, tf_child_frame_id)
        # TF(tf_frame_id, tf_child_frame_id) = TF(tf_frame_id, source_frame_id) @ TF(source_frame_id, source_child_frame_id) @ TF(source_child_frame_id, tf_child_frame_id)

        def log_tf_msg(tf: geometry_msgs.msg.TransformStamped):
            self.get_logger().info(
                f"TF {tf.header.frame_id} -> {tf.child_frame_id}: {tf.transform.translation.x:.2f}, {tf.transform.translation.y:.2f}, {tf.transform.translation.z:.2f}",
                # throttle_duration_sec=2.0,
            )

        def log_tf_pose_msg(tf: geometry_msgs.msg.PoseStamped, child_frame_id: str):
            self.get_logger().info(
                f"Pose {tf.header.frame_id} -> {child_frame_id}: {tf.pose.position.x:.2f}, {tf.pose.position.y:.2f}, {tf.pose.position.z:.2f}",
                # throttle_duration_sec=2.0,
            )

        def log_tf_matrix(tf, frame_id, child_frame_id):
            translation = translation_from_matrix(tf)
            self.get_logger().info(
                f"TF {frame_id} -> {child_frame_id}: {translation[0]:.2f}, {translation[1]:.2f}, {translation[2]:.2f}",
                # throttle_duration_sec=2.0,
            )

        # Lookup the transform from the tf_frame_id to the source_frame_id
        try:
            t1 = self.tf_buffer.lookup_transform(
                self.get_parameter('tf_frame_id').value,
                self.get_parameter('source_frame_id').value,
                rclpy.time.Time(),
            )
        except tf2_py.LookupException as e:
            self.get_logger().error(
                f"LookupException: {e}", throttle_duration_sec=2.0)
            return
        except tf2_py.ConnectivityException as e:
            self.get_logger().error(
                f"ConnectivityException: {e}", throttle_duration_sec=2.0)
            return
        except tf2_py.ExtrapolationException as e:
            self.get_logger().error(
                f"ExtrapolationException: {e}", throttle_duration_sec=2.0)
            return

        # Lookup the transform from the source_child_frame_id to the tf_child_frame_id
        try:
            t2 = self.tf_buffer.lookup_transform(
                self.get_parameter('source_child_frame_id').value,
                self.get_parameter('tf_child_frame_id').value,
                rclpy.time.Time(),
            )
        except tf2_py.LookupException as e:
            self.get_logger().error(
                f"LookupException: {e}", throttle_duration_sec=2.0)
            return
        except tf2_py.ConnectivityException as e:
            self.get_logger().error(
                f"ConnectivityException: {e}", throttle_duration_sec=2.0)
            return
        except tf2_py.ExtrapolationException as e:
            self.get_logger().error(
                f"ExtrapolationException: {e}", throttle_duration_sec=2.0)
            return

        # Convert the first transform from tf_buffer into a transformation matrix
        tf_frame_id_to_source_frame_id_matrix = compose_matrix(
            translate=[
                t1.transform.translation.x,
                t1.transform.translation.y,
                t1.transform.translation.z
            ], angles=euler_from_quaternion([
                t1.transform.rotation.x,
                t1.transform.rotation.y,
                t1.transform.rotation.z,
                t1.transform.rotation.w
            ]),
        )

        # Convert the pose from the message into a transformation matrix
        source_frame_id_to_source_child_frame_id_matrix = compose_matrix(
            translate=[
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z
            ], angles=euler_from_quaternion([
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w
            ]),
        )

        # Convert the second transform from tf_buffer into a transformation matrix
        source_child_frame_id_to_tf_child_frame_id_matrix = compose_matrix(
            translate=[
                t2.transform.translation.x,
                t2.transform.translation.y,
                t2.transform.translation.z
            ], angles=euler_from_quaternion([
                t2.transform.rotation.x,
                t2.transform.rotation.y,
                t2.transform.rotation.z,
                t2.transform.rotation.w
            ]),
        )

        tf_frame_id_to_tf_child_frame_id_matrix = concatenate_matrices(
            tf_frame_id_to_source_frame_id_matrix,
            source_frame_id_to_source_child_frame_id_matrix,
            source_child_frame_id_to_tf_child_frame_id_matrix,
        )

        translation = translation_from_matrix(
            tf_frame_id_to_tf_child_frame_id_matrix)
        quaternion = quaternion_from_matrix(
            tf_frame_id_to_tf_child_frame_id_matrix)

        # Broadcast the transform from tf_frame_id to tf_child_frame_id
        timestamp = msg.header.stamp

        child_tf = geometry_msgs.msg.TransformStamped()
        child_tf.header.stamp = timestamp
        child_tf.header.frame_id = self.get_parameter('tf_frame_id').value
        child_tf.child_frame_id = self.get_parameter('tf_child_frame_id').value
        child_tf.transform.translation.x = translation[0]
        child_tf.transform.translation.y = translation[1]
        child_tf.transform.translation.z = translation[2]
        child_tf.transform.rotation.x = quaternion[0]
        child_tf.transform.rotation.y = quaternion[1]
        child_tf.transform.rotation.z = quaternion[2]
        child_tf.transform.rotation.w = quaternion[3]

        # Send the transformation
        self.tf_buffer.set_transform(child_tf, 'default_authority')
        self.tf_broadcaster.sendTransform(child_tf)

        # Publish the pose to the pose topic if enabled
        if self.get_parameter('pose_topic_name').value != "":
            try:
                pose_topic_tf = self.tf_buffer.lookup_transform(
                    self.get_parameter('pose_child_frame_id').value,
                    self.get_parameter('pose_frame_id').value,
                    timestamp,
                )
            except tf2_py.LookupException as e:
                self.get_logger().error(
                    f"LookupException: {e}", throttle_duration_sec=2.0)
                return
            except tf2_py.ConnectivityException as e:
                self.get_logger().error(
                    f"ConnectivityException: {e}", throttle_duration_sec=2.0)
                return
            except tf2_py.ExtrapolationException as e:
                self.get_logger().error(
                    f"ExtrapolationException: {e}", throttle_duration_sec=2.0)
                return

            pose_msg = geometry_msgs.msg.PoseStamped()
            pose_msg.header.stamp = timestamp.to_msg()
            pose_msg.header.frame_id = self.get_parameter(
                'pose_frame_id').value
            pose_msg.pose.position.x = pose_topic_tf.transform.translation.x
            pose_msg.pose.position.y = pose_topic_tf.transform.translation.y
            pose_msg.pose.position.z = pose_topic_tf.transform.translation.z
            pose_msg.pose.orientation.x = pose_topic_tf.transform.rotation.x
            pose_msg.pose.orientation.y = pose_topic_tf.transform.rotation.y
            pose_msg.pose.orientation.z = pose_topic_tf.transform.rotation.z
            pose_msg.pose.orientation.w = pose_topic_tf.transform.rotation.w

            # Publish the pose
            self.publisher.publish(pose_msg)


def main():
    rclpy.init()
    node = FramePublisher()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
