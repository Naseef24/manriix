#!/usr/bin/env python3

"""
Manriix Object Tracking -> MQTT Bridge

Consumes object tracking data produced by the direct ZED SDK perception node:

    /object_tracking/objects
    manriix_perception/msg/ObjectsList

and converts PERSON detections into the legacy AI photographer MQTT format:

    /photo_capture/objects_tracking

Coordinate conventions
----------------------
Input ObjectsList positions are already expressed in the robot base frame:

    X = forward
    Y = left
    Z = up

The AI photographer expects the legacy guidance convention:

    X = right
    Y = up
    Z = forward

Therefore:

    legacy_x = -base_y
    legacy_y =  base_z
    legacy_z =  base_x

No TF transformation is required here because the direct ZED node has already
transformed detections into the robot base coordinate convention.
"""

import json
import math
import threading
import time
from typing import Dict, List

import paho.mqtt.client as mqtt
import rclpy

from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from manriix_perception.msg import ObjectsList


class ZedMqttTrackingBridge(Node):
    """
    Bridge Manriix direct-ZED object detections to the AI photographer MQTT
    tracking format.
    """

    ROS_OBJECT_TOPIC = "/object_tracking/objects"

    def __init__(self) -> None:
        super().__init__("zed_mqtt_tracking_bridge")

        # --------------------------------------------------------------
        # Parameters
        # --------------------------------------------------------------
        self.declare_parameter("mqtt_host", "localhost")
        self.declare_parameter("mqtt_port", 1883)

        self.declare_parameter(
            "mqtt_topic",
            "/photo_capture/objects_tracking",
        )

        self.declare_parameter(
            "base_frame",
            "base_footprint",
        )

        # Retained for launch-file compatibility.
        #
        # The new ObjectsList message contains detections that have already
        # passed the direct ZED node's tracking-state filtering, therefore
        # SEARCHING/OK filtering is no longer performed here.
        self.declare_parameter(
            "publish_searching_tracks",
            True,
        )

        self.mqtt_host = str(
            self.get_parameter("mqtt_host").value
        )

        self.mqtt_port = int(
            self.get_parameter("mqtt_port").value
        )

        self.mqtt_topic = str(
            self.get_parameter("mqtt_topic").value
        )

        self.base_frame = str(
            self.get_parameter("base_frame").value
        )

        self.publish_searching_tracks = bool(
            self.get_parameter(
                "publish_searching_tracks"
            ).value
        )

        # --------------------------------------------------------------
        # Statistics
        # --------------------------------------------------------------
        self.mqtt_lock = threading.Lock()

        self.mqtt_connected = False

        self.messages_received = 0
        self.messages_published = 0
        self.detections_published = 0

        self.invalid_detections = 0
        self.non_person_detections = 0

        # --------------------------------------------------------------
        # MQTT
        # --------------------------------------------------------------
        self._setup_mqtt()

        # --------------------------------------------------------------
        # ROS QoS
        #
        # Must match the direct ZED node's RELIABLE publisher.
        # --------------------------------------------------------------
        object_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # --------------------------------------------------------------
        # ROS subscription
        # --------------------------------------------------------------
        self.object_subscription = self.create_subscription(
            ObjectsList,
            self.ROS_OBJECT_TOPIC,
            self._objects_callback,
            object_qos,
        )

        self.get_logger().info(
            "Subscribed to direct ZED tracking topic: "
            f"{self.ROS_OBJECT_TOPIC}"
        )

        # --------------------------------------------------------------
        # Status
        # --------------------------------------------------------------
        self.status_timer = self.create_timer(
            10.0,
            self._report_status,
        )

        self.get_logger().info(
            f"MQTT destination: "
            f"{self.mqtt_host}:{self.mqtt_port} "
            f"topic={self.mqtt_topic}"
        )

        self.get_logger().info(
            f"Input coordinate frame: {self.base_frame}"
        )

        self.get_logger().info(
            "Expected input convention: "
            "X=forward, Y=left, Z=up"
        )

        self.get_logger().info(
            "MQTT guidance convention: "
            "X=right, Y=up, Z=forward"
        )

    # ==================================================================
    # MQTT
    # ==================================================================

    def _setup_mqtt(self) -> None:
        try:
            try:
                self.mqtt_client = mqtt.Client(
                    callback_api_version=(
                        mqtt.CallbackAPIVersion.VERSION2
                    ),
                    client_id="zed_mqtt_tracking_bridge",
                )

            except (AttributeError, TypeError):
                self.mqtt_client = mqtt.Client(
                    client_id="zed_mqtt_tracking_bridge"
                )

            self.mqtt_client.on_connect = (
                self._on_mqtt_connect
            )

            self.mqtt_client.on_disconnect = (
                self._on_mqtt_disconnect
            )

            self.mqtt_client.connect(
                self.mqtt_host,
                self.mqtt_port,
                keepalive=60,
            )

            self.mqtt_client.loop_start()

        except Exception as exc:
            self.get_logger().error(
                f"Failed to initialise MQTT connection: {exc}"
            )
            raise

    def _on_mqtt_connect(
        self,
        client,
        userdata,
        flags,
        reason_code,
        properties=None,
    ) -> None:
        try:
            connected = int(reason_code) == 0

        except (TypeError, ValueError):
            connected = reason_code == 0

        self.mqtt_connected = connected

        if connected:
            self.get_logger().info(
                "Connected to MQTT broker"
            )

        else:
            self.get_logger().error(
                f"MQTT connection failed: {reason_code}"
            )

    def _on_mqtt_disconnect(
        self,
        client,
        userdata,
        disconnect_flags=None,
        reason_code=0,
        properties=None,
    ) -> None:
        self.mqtt_connected = False

        self.get_logger().warning(
            f"MQTT disconnected: {reason_code}"
        )

    # ==================================================================
    # Object tracking
    # ==================================================================

    def _objects_callback(
        self,
        msg: ObjectsList,
    ) -> None:

        self.messages_received += 1

        detections: List[Dict] = []

        # --------------------------------------------------------------
        # Input frame
        #
        # direct_zed_detection_node_modified publishes ObjectsList in
        # base_footprint.
        # --------------------------------------------------------------
        source_frame = (
            msg.header.frame_id
            if msg.header.frame_id
            else self.base_frame
        )

        for obj in msg.objects:

            # ----------------------------------------------------------
            # Photographer currently tracks people only.
            # ----------------------------------------------------------
            class_name = str(
                obj.class_name
            ).strip().lower()

            class_id = int(
                obj.class_id
            )

            if (
                class_id != 0
                and class_name != "person"
            ):
                self.non_person_detections += 1
                continue

            # ----------------------------------------------------------
            # Position
            #
            # Already in ROS robot convention:
            #
            # X forward
            # Y left
            # Z up
            # ----------------------------------------------------------
            base_x = float(
                obj.position.x
            )

            base_y = float(
                obj.position.y
            )

            base_z = float(
                obj.position.z
            )

            if not all(
                math.isfinite(value)
                for value in (
                    base_x,
                    base_y,
                    base_z,
                )
            ):
                self.invalid_detections += 1
                continue

            # ----------------------------------------------------------
            # Velocity
            # ----------------------------------------------------------
            velocity_x = float(
                obj.velocity.x
            )

            velocity_y = float(
                obj.velocity.y
            )

            velocity_z = float(
                obj.velocity.z
            )

            if not all(
                math.isfinite(value)
                for value in (
                    velocity_x,
                    velocity_y,
                    velocity_z,
                )
            ):
                velocity_x = 0.0
                velocity_y = 0.0
                velocity_z = 0.0

            # ----------------------------------------------------------
            # Convert ROS base convention -> legacy photographer
            #
            # ROS:
            #   X forward
            #   Y left
            #   Z up
            #
            # Photographer:
            #   X right
            #   Y up
            #   Z forward
            # ----------------------------------------------------------
            legacy_x = -base_y
            legacy_y = base_z
            legacy_z = base_x

            legacy_vx = -velocity_y
            legacy_vy = velocity_z
            legacy_vz = velocity_x

            # ----------------------------------------------------------
            # Distance
            # ----------------------------------------------------------
            calculated_distance = math.sqrt(
                base_x ** 2
                + base_y ** 2
                + base_z ** 2
            )

            message_distance = float(
                obj.distance
            )

            if (
                math.isfinite(message_distance)
                and message_distance >= 0.0
            ):
                distance = message_distance
            else:
                distance = calculated_distance

            # ----------------------------------------------------------
            # Velocity magnitude
            # ----------------------------------------------------------
            calculated_velocity_magnitude = math.sqrt(
                legacy_vx ** 2
                + legacy_vy ** 2
                + legacy_vz ** 2
            )

            message_velocity_magnitude = float(
                obj.velocity_magnitude
            )

            if (
                math.isfinite(
                    message_velocity_magnitude
                )
                and message_velocity_magnitude >= 0.0
            ):
                velocity_magnitude = (
                    message_velocity_magnitude
                )
            else:
                velocity_magnitude = (
                    calculated_velocity_magnitude
                )

            # ----------------------------------------------------------
            # Tracking ID
            #
            # The direct node already provides the tracking ID.
            # Do NOT apply the old per-camera offsets again.
            # ----------------------------------------------------------
            tracking_id = int(
                obj.tracking_id
            )

            camera_name = str(
                obj.camera_name
            )

            confidence = float(
                obj.confidence
            )

            # ----------------------------------------------------------
            # Preserve the previous MQTT schema as closely as possible.
            #
            # Fields that existed only in zed_msgs/ObjectsStamped cannot
            # be recovered from ObjectTrackingData, so sensible
            # compatibility values are supplied.
            # ----------------------------------------------------------
            detection = {
                "tracking_id": tracking_id,

                "source_tracking_id": tracking_id,

                "class_id": class_id,

                "class_name": (
                    class_name
                    if class_name
                    else "person"
                ),

                "label": (
                    class_name.upper()
                    if class_name
                    else "PERSON"
                ),

                # ObjectTrackingData does not contain a sublabel.
                "sublabel": "",

                "confidence": confidence,

                # Legacy photographer convention.
                "position": {
                    "x": legacy_x,
                    "y": legacy_y,
                    "z": legacy_z,
                },

                # ROS robot convention.
                "base_position": {
                    "x": base_x,
                    "y": base_y,
                    "z": base_z,
                },

                "distance": distance,

                # Preserve old compatibility field.
                "depth_mean": distance,

                "velocity": {
                    "x": legacy_vx,
                    "y": legacy_vy,
                    "z": legacy_vz,
                },

                "velocity_magnitude": (
                    velocity_magnitude
                ),

                "camera_name": camera_name,

                "source_frame": source_frame,

                # The direct node already filters invalid ZED tracking
                # states before publishing ObjectTrackingData.
                "tracking_available": (
                    tracking_id >= 0
                ),

                # Compatibility value:
                # 1 represented TRACKING_STATE.OK in the old bridge.
                "tracking_state": (
                    1
                    if tracking_id >= 0
                    else 0
                ),

                # ObjectTrackingData does not expose action_state.
                "action_state": 0,

                # These fields are not currently present in
                # ObjectTrackingData.
                "bbox_2d": [],

                "dimensions_3d": [],
            }

            detections.append(
                detection
            )

        # --------------------------------------------------------------
        # Preserve previous behaviour:
        # don't send empty MQTT tracking messages.
        # --------------------------------------------------------------
        if not detections:
            return

        # --------------------------------------------------------------
        # Timestamp
        # --------------------------------------------------------------
        timestamp = (
            float(msg.header.stamp.sec)
            + float(msg.header.stamp.nanosec)
            / 1e9
        )

        # --------------------------------------------------------------
        # Group detections by camera.
        #
        # The old bridge published one message per physical camera.
        # ObjectsList can contain detections from several cameras, so
        # preserve the "cameras" dictionary while allowing all active
        # cameras to be represented in one MQTT message.
        # --------------------------------------------------------------
        cameras: Dict[str, List[Dict]] = {}

        for detection in detections:
            camera_name = (
                detection["camera_name"]
                or "unknown"
            )

            cameras.setdefault(
                camera_name,
                [],
            ).append(
                detection
            )

        # --------------------------------------------------------------
        # MQTT payload
        # --------------------------------------------------------------
        payload = {
            "timestamp": timestamp,

            "frame_id": self.base_frame,

            "cameras": cameras,

            "total_detections": len(
                detections
            ),
        }

        try:
            encoded = json.dumps(
                payload,
                allow_nan=False,
                separators=(",", ":"),
            )

            with self.mqtt_lock:
                result = self.mqtt_client.publish(
                    self.mqtt_topic,
                    encoded,
                    qos=0,
                )

            if (
                result.rc
                == mqtt.MQTT_ERR_SUCCESS
            ):
                self.messages_published += 1

                self.detections_published += (
                    len(detections)
                )

            else:
                self.get_logger().warning(
                    "MQTT publish failed with "
                    f"code {result.rc}"
                )

        except Exception as exc:
            self.get_logger().error(
                "Failed to publish tracking "
                f"payload: {exc}"
            )

    # ==================================================================
    # Status
    # ==================================================================

    def _report_status(
        self,
    ) -> None:

        self.get_logger().info(
            "ZED MQTT bridge status | "
            f"mqtt="
            f"{'connected' if self.mqtt_connected else 'disconnected'}"
            " | "
            f"ROS messages={self.messages_received}"
            " | "
            f"MQTT messages={self.messages_published}"
            " | "
            f"detections={self.detections_published}"
            " | "
            f"invalid={self.invalid_detections}"
            " | "
            f"non_person={self.non_person_detections}"
        )

    # ==================================================================
    # Shutdown
    # ==================================================================

    def destroy_node(
        self,
    ) -> bool:

        try:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()

        except Exception:
            pass

        return super().destroy_node()


def main(
    args=None,
) -> None:

    rclpy.init(
        args=args
    )

    node = ZedMqttTrackingBridge()

    try:
        rclpy.spin(
            node
        )

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()





# #!/usr/bin/env python3

# import json
# import math
# import threading
# import time
# from typing import Dict, List

# import paho.mqtt.client as mqtt
# import rclpy
# from geometry_msgs.msg import PointStamped
# from rclpy.duration import Duration
# from rclpy.node import Node
# from rclpy.qos import (
#     DurabilityPolicy,
#     HistoryPolicy,
#     QoSProfile,
#     ReliabilityPolicy,
# )
# from rclpy.time import Time
# import tf2_ros
# from tf2_geometry_msgs import do_transform_point
# from zed_msgs.msg import ObjectsStamped


# class ZedMqttTrackingBridge(Node):
#     """Bridge native ZED object detections to the AI photographer MQTT format."""

#     CAMERA_TOPICS = {
#         "front": "/zedx_front/zed_node/obj_det/objects",
#         "left": "/zedx_left/zed_node/obj_det/objects",
#         "right": "/zedx_right/zed_node/obj_det/objects",
#     }

#     # Keep tracking IDs unique across independent ZED tracking instances.
#     CAMERA_ID_OFFSETS = {
#         "front": 0,
#         "left": 100000,
#         "right": 200000,
#     }

#     def __init__(self) -> None:
#         super().__init__("zed_mqtt_tracking_bridge")

#         self.declare_parameter("mqtt_host", "localhost")
#         self.declare_parameter("mqtt_port", 1883)
#         self.declare_parameter(
#             "mqtt_topic",
#             "/photo_capture/objects_tracking",
#         )
#         self.declare_parameter("base_frame", "base_link")
#         self.declare_parameter("publish_searching_tracks", True)

#         self.mqtt_host = str(self.get_parameter("mqtt_host").value)
#         self.mqtt_port = int(self.get_parameter("mqtt_port").value)
#         self.mqtt_topic = str(self.get_parameter("mqtt_topic").value)
#         self.base_frame = str(self.get_parameter("base_frame").value)
#         self.publish_searching_tracks = bool(
#             self.get_parameter("publish_searching_tracks").value
#         )

#         self.mqtt_lock = threading.Lock()
#         self.mqtt_connected = False
#         self.messages_received = 0
#         self.messages_published = 0
#         self.detections_published = 0
#         self.tf_failures = 0

#         self._setup_mqtt()

#         self.tf_buffer = tf2_ros.Buffer(
#             cache_time=Duration(seconds=10.0)
#         )
#         self.tf_listener = tf2_ros.TransformListener(
#             self.tf_buffer,
#             self,
#             spin_thread=True,
#         )

#         object_qos = QoSProfile(
#             reliability=ReliabilityPolicy.RELIABLE,
#             durability=DurabilityPolicy.VOLATILE,
#             history=HistoryPolicy.KEEP_LAST,
#             depth=10,
#         )

#         self.object_subscriptions = []
#         for camera_name, topic in self.CAMERA_TOPICS.items():
#             subscription = self.create_subscription(
#                 ObjectsStamped,
#                 topic,
#                 lambda msg, camera=camera_name: self._objects_callback(
#                     msg,
#                     camera,
#                 ),
#                 object_qos,
#             )
#             self.object_subscriptions.append(subscription)
#             self.get_logger().info(
#                 f"Subscribed: {topic} [{camera_name}]"
#             )

#         self.status_timer = self.create_timer(10.0, self._report_status)

#         self.get_logger().info(
#             f"MQTT destination: {self.mqtt_host}:{self.mqtt_port}"
#             f" topic={self.mqtt_topic}"
#         )
#         self.get_logger().info(
#             f"Target coordinate frame: {self.base_frame}"
#         )

#     def _setup_mqtt(self) -> None:
#         try:
#             try:
#                 self.mqtt_client = mqtt.Client(
#                     callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
#                     client_id="zed_mqtt_tracking_bridge",
#                 )
#             except (AttributeError, TypeError):
#                 self.mqtt_client = mqtt.Client(
#                     client_id="zed_mqtt_tracking_bridge"
#                 )

#             self.mqtt_client.on_connect = self._on_mqtt_connect
#             self.mqtt_client.on_disconnect = self._on_mqtt_disconnect
#             self.mqtt_client.connect(
#                 self.mqtt_host,
#                 self.mqtt_port,
#                 keepalive=60,
#             )
#             self.mqtt_client.loop_start()

#         except Exception as exc:
#             self.get_logger().error(
#                 f"Failed to initialise MQTT connection: {exc}"
#             )
#             raise

#     def _on_mqtt_connect(
#         self,
#         client,
#         userdata,
#         flags,
#         reason_code,
#         properties=None,
#     ) -> None:
#         try:
#             connected = int(reason_code) == 0
#         except (TypeError, ValueError):
#             connected = reason_code == 0

#         self.mqtt_connected = connected

#         if connected:
#             self.get_logger().info("Connected to MQTT broker")
#         else:
#             self.get_logger().error(
#                 f"MQTT connection failed: {reason_code}"
#             )

#     def _on_mqtt_disconnect(
#         self,
#         client,
#         userdata,
#         disconnect_flags=None,
#         reason_code=0,
#         properties=None,
#     ) -> None:
#         self.mqtt_connected = False
#         self.get_logger().warning(
#             f"MQTT disconnected: {reason_code}"
#         )

#     def _objects_callback(
#         self,
#         msg: ObjectsStamped,
#         camera_name: str,
#     ) -> None:
#         self.messages_received += 1

#         if not msg.header.frame_id:
#             self.get_logger().warning(
#                 f"Skipping {camera_name} message with empty frame_id"
#             )
#             return

#         try:
#             stamp = Time.from_msg(msg.header.stamp)
#             transform = self.tf_buffer.lookup_transform(
#                 self.base_frame,
#                 msg.header.frame_id,
#                 Time(),
#                 timeout=Duration(seconds=0.1),
#             )
#         except Exception as exc:
#             self.tf_failures += 1
#             self.get_logger().warning(
#                 f"TF unavailable: {msg.header.frame_id} -> "
#                 f"{self.base_frame}: {exc}"
#             )
#             return

#         detections: List[Dict] = []

#         for obj in msg.objects:
#             # 0=OFF, 1=OK, 2=SEARCHING, 3=TERMINATE
#             valid_tracking_states = {1}
#             if self.publish_searching_tracks:
#                 valid_tracking_states.add(2)

#             if obj.tracking_available:
#                 if int(obj.tracking_state) not in valid_tracking_states:
#                     continue

#             if str(obj.label).upper() != "PERSON":
#                 continue

#             if len(obj.position) != 3 or len(obj.velocity) != 3:
#                 continue

#             camera_position = [float(value) for value in obj.position]
#             camera_velocity = [float(value) for value in obj.velocity]

#             if not all(math.isfinite(value) for value in camera_position):
#                 continue

#             if not all(math.isfinite(value) for value in camera_velocity):
#                 camera_velocity = [0.0, 0.0, 0.0]

#             base_position = self._transform_point(
#                 camera_position,
#                 msg.header.frame_id,
#                 msg.header.stamp,
#                 transform,
#             )

#             if base_position is None:
#                 continue

#             base_velocity = self._transform_vector(
#                 camera_velocity,
#                 msg.header.frame_id,
#                 msg.header.stamp,
#                 transform,
#             )

#             # base_link convention:
#             #   X = forward, Y = left, Z = up
#             #
#             # Legacy ZED-X guidance convention:
#             #   X = right, Y = up, Z = forward
#             legacy_x = -base_position[1]
#             legacy_y = base_position[2]
#             legacy_z = base_position[0]

#             legacy_vx = -base_velocity[1]
#             legacy_vy = base_velocity[2]
#             legacy_vz = base_velocity[0]

#             distance = math.sqrt(
#                 base_position[0] ** 2
#                 + base_position[1] ** 2
#                 + base_position[2] ** 2
#             )

#             velocity_magnitude = math.sqrt(
#                 legacy_vx ** 2
#                 + legacy_vy ** 2
#                 + legacy_vz ** 2
#             )

#             raw_tracking_id = int(obj.label_id)
#             if raw_tracking_id >= 0:
#                 tracking_id = (
#                     self.CAMERA_ID_OFFSETS[camera_name]
#                     + raw_tracking_id
#                 )
#             else:
#                 tracking_id = -1

#             bbox_2d = [
#                 [int(corner.kp[0]), int(corner.kp[1])]
#                 for corner in obj.bounding_box_2d.corners
#             ]

#             detection = {
#                 "tracking_id": tracking_id,
#                 "source_tracking_id": raw_tracking_id,
#                 "class_id": 0,
#                 "class_name": "person",
#                 "label": str(obj.label),
#                 "sublabel": str(obj.sublabel),
#                 "confidence": float(obj.confidence),
#                 "position": {
#                     "x": legacy_x,
#                     "y": legacy_y,
#                     "z": legacy_z,
#                 },
#                 "base_position": {
#                     "x": base_position[0],
#                     "y": base_position[1],
#                     "z": base_position[2],
#                 },
#                 "distance": distance,
#                 "depth_mean": distance,
#                 "velocity": {
#                     "x": legacy_vx,
#                     "y": legacy_vy,
#                     "z": legacy_vz,
#                 },
#                 "velocity_magnitude": velocity_magnitude,
#                 "camera_name": camera_name,
#                 "source_frame": msg.header.frame_id,
#                 "tracking_available": bool(obj.tracking_available),
#                 "tracking_state": int(obj.tracking_state),
#                 "action_state": int(obj.action_state),
#                 "bbox_2d": bbox_2d,
#                 "dimensions_3d": [
#                     float(value) for value in obj.dimensions_3d
#                 ],
#             }

#             detections.append(detection)

#         if not detections:
#             return

#         timestamp = (
#             float(msg.header.stamp.sec)
#             + float(msg.header.stamp.nanosec) / 1e9
#         )

#         payload = {
#             "timestamp": timestamp,
#             "frame_id": self.base_frame,
#             "cameras": {
#                 camera_name: detections,
#             },
#             "total_detections": len(detections),
#         }

#         try:
#             encoded = json.dumps(
#                 payload,
#                 allow_nan=False,
#                 separators=(",", ":"),
#             )

#             with self.mqtt_lock:
#                 result = self.mqtt_client.publish(
#                     self.mqtt_topic,
#                     encoded,
#                     qos=0,
#                 )

#             if result.rc == mqtt.MQTT_ERR_SUCCESS:
#                 self.messages_published += 1
#                 self.detections_published += len(detections)
#             else:
#                 self.get_logger().warning(
#                     f"MQTT publish failed with code {result.rc}"
#                 )

#         except Exception as exc:
#             self.get_logger().error(
#                 f"Failed to publish tracking payload: {exc}"
#             )

#     @staticmethod
#     def _transform_point(
#         xyz: List[float],
#         source_frame: str,
#         stamp,
#         transform,
#     ):
#         point = PointStamped()
#         point.header.frame_id = source_frame
#         point.header.stamp = stamp
#         point.point.x = xyz[0]
#         point.point.y = xyz[1]
#         point.point.z = xyz[2]

#         transformed = do_transform_point(point, transform)

#         return (
#             float(transformed.point.x),
#             float(transformed.point.y),
#             float(transformed.point.z),
#         )

#     @staticmethod
#     def _transform_vector(
#         xyz: List[float],
#         source_frame: str,
#         stamp,
#         transform,
#     ):
#         # Transform the vector as two points and subtract them.
#         # This applies rotation while cancelling translation.
#         origin = PointStamped()
#         origin.header.frame_id = source_frame
#         origin.header.stamp = stamp

#         endpoint = PointStamped()
#         endpoint.header.frame_id = source_frame
#         endpoint.header.stamp = stamp
#         endpoint.point.x = xyz[0]
#         endpoint.point.y = xyz[1]
#         endpoint.point.z = xyz[2]

#         transformed_origin = do_transform_point(origin, transform)
#         transformed_endpoint = do_transform_point(endpoint, transform)

#         return (
#             float(
#                 transformed_endpoint.point.x
#                 - transformed_origin.point.x
#             ),
#             float(
#                 transformed_endpoint.point.y
#                 - transformed_origin.point.y
#             ),
#             float(
#                 transformed_endpoint.point.z
#                 - transformed_origin.point.z
#             ),
#         )

#     def _report_status(self) -> None:
#         self.get_logger().info(
#             "ZED MQTT bridge status | "
#             f"mqtt={'connected' if self.mqtt_connected else 'disconnected'} | "
#             f"ROS messages={self.messages_received} | "
#             f"MQTT messages={self.messages_published} | "
#             f"detections={self.detections_published} | "
#             f"TF failures={self.tf_failures}"
#         )

#     def destroy_node(self) -> bool:
#         try:
#             self.mqtt_client.loop_stop()
#             self.mqtt_client.disconnect()
#         except Exception:
#             pass

#         return super().destroy_node()


# def main(args=None) -> None:
#     rclpy.init(args=args)
#     node = ZedMqttTrackingBridge()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         if rclpy.ok():
#             rclpy.shutdown()


# if __name__ == "__main__":
#     main()
