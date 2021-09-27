from rclpy.node import Node
from rclpy import init, spin, shutdown
from interactive_markers import InteractiveMarkerServer, MenuHandler
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Vector3, Quaternion, TransformStamped, Transform, Point, Pose
from tf2_ros import StaticTransformBroadcaster
from interactive_transform_publisher_msgs.srv import FramesTransform


class InteractiveTransformPublisher(Node):
    def __init__(self):
        super().__init__(node_name='interactive_transform_publisher')
        self.interactive_marker_server = None
        self.menu_handler = None
        self.static_tf_broadcaster = None

    def on_configure(self):
        self.interactive_marker_server = InteractiveMarkerServer(node=self, namespace='interactive_markers')
        self.menu_handler = MenuHandler()
        self.menu_handler.insert(
            title='Print Transform',
            callback=self._print_transform_callback
        )

        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        self.add_interactive_static_transform_service = self.create_service(
            srv_type=FramesTransform,
            srv_name='interactive_transform_publisher/add_interactive_static_transform',
            callback=self._add_interactive_static_transform_callback
        )

        self.get_logger().info('CONFIGURED')

    def on_activate(self):
        self.get_logger().info('ACTIVATED')

    def on_shutdown(self):
        self.interactive_marker_server.shutdown()
        self.destroy_service(self.add_interactive_static_transform_service)

        self.get_logger().info('SHUT DOWN')

    def on_error(self):
        self.get_logger().error('ERROR')

    def _add_interactive_static_transform_callback(self, request, response):
        self.add_interactive_static_transform(
            frame_id=request.frame_id,
            child_frame_id=request.child_frame_id,
            position=Point(x=request.x, y=request.y, z=request.z),
            orientation=Quaternion(x=request.qx, y=request.qy, z=request.qz, w=request.qw)
        )

        response.accomplished = True
        return response

    def add_interactive_static_transform(self, frame_id, child_frame_id, position=Point(x=0., y=0., z=0.), orientation=Quaternion(x=0., y=0., z=0., w=1.)):
        self.interactive_marker_server.insert(
            marker=self._create_interactive_marker(
                position=position,
                orientation=orientation,
                frame_id=frame_id,
                child_frame_id=child_frame_id
            ),
            feedback_callback=self._update_static_transform_callback
        )
        self.menu_handler.apply(server=self.interactive_marker_server, marker_name=child_frame_id)

        self.static_tf_broadcaster.sendTransform([self._create_transform(
            stamp=self.get_clock().now().to_msg(),
            position=Vector3(x=position.x, y=position.y, z=position.z),
            orientation=orientation,
            frame_id=frame_id,
            child_frame_id=child_frame_id
        )])

        self.interactive_marker_server.applyChanges()

    @staticmethod
    def _create_interactive_marker(position, orientation, frame_id, child_frame_id):
        return InteractiveMarker(
            header=Header(
                frame_id=frame_id
            ),
            pose=Pose(
                position=position,
                orientation=orientation
            ),
            scale=1.,
            name=child_frame_id,
            description=child_frame_id,
            controls=[
                # Cube
                InteractiveMarkerControl(
                    always_visible=True,
                    interaction_mode=InteractiveMarkerControl.BUTTON,
                    markers=[Marker(
                        type=Marker.CUBE,
                        color=ColorRGBA(
                            r=0.5,
                            g=0.5,
                            b=0.5,
                            a=1.
                        ),
                        scale=Vector3(
                            x=0.5,
                            y=0.5,
                            z=0.5
                        )
                    )]
                ),
                # X axis
                InteractiveMarkerControl(
                    name='x',
                    interaction_mode=InteractiveMarkerControl.MOVE_AXIS,
                    orientation_mode=InteractiveMarkerControl.FIXED,
                    orientation=Quaternion(
                        x=0.7071067811865476,
                        y=0.,
                        z=0.,
                        w=0.7071067811865476
                    )
                ),
                # Y axis
                InteractiveMarkerControl(
                    name='y',
                    interaction_mode=InteractiveMarkerControl.MOVE_AXIS,
                    orientation_mode=InteractiveMarkerControl.FIXED,
                    orientation=Quaternion(
                        x=0.,
                        y=0.,
                        z=0.7071067811865476,
                        w=0.7071067811865476
                    )
                ),
                # Z axis
                InteractiveMarkerControl(
                    name='z',
                    interaction_mode=InteractiveMarkerControl.MOVE_AXIS,
                    orientation_mode=InteractiveMarkerControl.FIXED,
                    orientation=Quaternion(
                        x=0.,
                        y=0.7071067811865476,
                        z=0.,
                        w=0.7071067811865476
                    )
                ),
                # Roll
                InteractiveMarkerControl(
                    name='roll',
                    interaction_mode=InteractiveMarkerControl.ROTATE_AXIS,
                    orientation_mode=InteractiveMarkerControl.FIXED,
                    orientation=Quaternion(
                        x=0.7071067811865476,
                        y=0.,
                        z=0.,
                        w=0.7071067811865476
                    )
                ),
                # Pitch
                InteractiveMarkerControl(
                    name='pitch',
                    interaction_mode=InteractiveMarkerControl.ROTATE_AXIS,
                    orientation_mode=InteractiveMarkerControl.FIXED,
                    orientation=Quaternion(
                        x=0.,
                        y=0.,
                        z=0.7071067811865476,
                        w=0.7071067811865476
                    )
                ),
                # Yaw
                InteractiveMarkerControl(
                    name='yaw',
                    interaction_mode=InteractiveMarkerControl.ROTATE_AXIS,
                    orientation_mode=InteractiveMarkerControl.FIXED,
                    orientation=Quaternion(
                        x=0.,
                        y=0.7071067811865476,
                        z=0.,
                        w=0.7071067811865476
                    )
                ),
            ]
        )

    @staticmethod
    def _create_transform(stamp, frame_id, child_frame_id, position=Vector3(x=0., y=0., z=0.), orientation=Quaternion(x=0., y=0., z=0., w=1.)):
        return TransformStamped(
            header=Header(
                stamp=stamp,
                frame_id=frame_id,
            ),
            child_frame_id=child_frame_id,
            transform=Transform(
                translation=position,
                rotation=orientation
            )
        )

    def _update_static_transform_callback(self, feedback):
        self.static_tf_broadcaster.sendTransform([self._create_transform(
            stamp=self.get_clock().now().to_msg(),
            position=Vector3(
                    x=feedback.pose.position.x,
                    y=feedback.pose.position.y,
                    z=feedback.pose.position.z
                ),
            orientation=Quaternion(
                    x=feedback.pose.orientation.x,
                    y=feedback.pose.orientation.y,
                    z=feedback.pose.orientation.z,
                    w=feedback.pose.orientation.w
                ),
            frame_id=feedback.header.frame_id,
            child_frame_id=feedback.marker_name
        )])

    def _print_transform_callback(self, feedback):
        self.get_logger().info('Transform: {} {} {} {} {} {} {} {} {}'.format(
            feedback.pose.position.x,
            feedback.pose.position.y,
            feedback.pose.position.z,
            feedback.pose.orientation.x,
            feedback.pose.orientation.y,
            feedback.pose.orientation.z,
            feedback.pose.orientation.w,
            feedback.header.frame_id,
            feedback.marker_name
        ))


def main(args=None):
    init(args=args)
    node = InteractiveTransformPublisher()

    try:
        node.on_configure()
        node.on_activate()
        spin(node)
    except KeyboardInterrupt:
        shutdown()
    except Exception as e:  # TODO: Identify all possible exceptions
        shutdown()
        node.on_error()
        node.get_logger().error(str(e))
    finally:
        node.on_shutdown()


if __name__ == '__main__':
    main()
