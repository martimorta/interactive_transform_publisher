from rclpy.node import Node
from rclpy import init, spin, shutdown
from interactive_markers import InteractiveMarkerServer, MenuHandler
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Vector3, Quaternion, TransformStamped, Transform, Point, Pose
from tf2_ros import StaticTransformBroadcaster
from interactive_transform_publisher_msgs.srv import FramesTransform
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from math import sqrt
from copy import deepcopy

class InteractiveTransformPublisher(Node):
    def __init__(self):
        super().__init__(node_name='interactive_transform_publisher')
        self.interactive_marker_server = None
        self.menu_handler = None
        self.static_tf_broadcaster = None
        self.shadows_ = []

         # Parameters
        self.add_on_set_parameters_callback(self.on_parameter)

        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('child_frame_id', 'marker')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.0)
        self.declare_parameter('qx', 0.0)
        self.declare_parameter('qy', 0.0)
        self.declare_parameter('qz', 0.0)
        self.declare_parameter('qw', 1.0)
        
        self.declare_parameter('scale', 1.0)

    def on_parameter(self, params):  
        for param in params:
            if param.name == 'scale' and param.type_ == Parameter.Type.DOUBLE:
                self.scale_ = param.value

            self.get_logger().info(f'{param.name}: {param.value}')

        return SetParametersResult(successful=True)

    def on_configure(self):
        self.interactive_marker_server = InteractiveMarkerServer(node=self, namespace='interactive_markers')
        self.menu_handler = MenuHandler()
        self.menu_handler.insert(
            title='Print Transform',
            callback=self._print_transform_callback
        )
        self.menu_handler.setCheckState(self.menu_handler.insert( "Shadow", callback=self._enable_shadow_callback ), MenuHandler.UNCHECKED )

        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        self.add_interactive_static_transform_service = self.create_service(
            srv_type=FramesTransform,
            srv_name='interactive_transform_publisher/add_interactive_static_transform',
            callback=self._add_interactive_static_transform_callback
        )

        self.add_interactive_static_transform(
            frame_id=self.get_parameter('frame_id').get_parameter_value().string_value,
            child_frame_id=self.get_parameter('child_frame_id').get_parameter_value().string_value,
            position=Point(
              x=self.get_parameter('x').get_parameter_value().double_value, 
              y=self.get_parameter('y').get_parameter_value().double_value, 
              z=self.get_parameter('z').get_parameter_value().double_value
            ),
            orientation=Quaternion(
              x=self.get_parameter('qx').get_parameter_value().double_value, 
              y=self.get_parameter('qy').get_parameter_value().double_value, 
              z=self.get_parameter('qz').get_parameter_value().double_value, 
              w=self.get_parameter('qw').get_parameter_value().double_value
              )
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
                child_frame_id=child_frame_id,
                scale=self.scale_
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

    def _update_static_transform_callback(self, feedback):
        transforms = [self._create_transform(
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
        )]
        if feedback.marker_name in self.shadows_:
            shadow_pose = deepcopy(feedback.pose)
            shadow_pose.position.z = 0.0
            shadow_pose.orientation = Quaternion()
            transforms.append(
                self._create_transform(
                    stamp=self.get_clock().now().to_msg(),
                    position=Vector3(
                            x=shadow_pose.position.x,
                            y=shadow_pose.position.y,
                            z=0.0
                        ),
                    orientation=Quaternion(),
                    frame_id=feedback.header.frame_id,
                    child_frame_id=feedback.marker_name+"-shadow"
                )
            )
            # Move marker
            self.interactive_marker_server.setPose(feedback.marker_name+"-shadow",shadow_pose)
        self.static_tf_broadcaster.sendTransform(transforms)
        


    def _enable_shadow_callback(self, feedback):
        handle = feedback.menu_entry_id
        
        if self.menu_handler.getCheckState( handle ) == MenuHandler.CHECKED:
            self.menu_handler.setCheckState( handle, MenuHandler.UNCHECKED )
            self.get_logger().info(f'Hiding shadow for {feedback.marker_name}')
        else:
            self.menu_handler.setCheckState( handle, MenuHandler.CHECKED )

            if not feedback.marker_name in self.shadows_:
                self.shadows_.append(feedback.marker_name)

            self.get_logger().info(f'Showing shadow for {feedback.marker_name}')
            self.interactive_marker_server.insert(
                marker=self._create_shadow_marker(
                    feedback.marker_name, 
                    feedback.pose, 
                    feedback.header.frame_id, 
                    self.scale_
                ),
                feedback_callback=self._update_static_transform_callback
            )
            self.static_tf_broadcaster.sendTransform([self._create_transform(
                stamp=self.get_clock().now().to_msg(),
                position=Vector3(x=feedback.pose.position.x, y=feedback.pose.position.y, z=feedback.pose.position.z),
                orientation=feedback.pose.orientation,
                frame_id=self.get_parameter('frame_id').get_parameter_value().string_value,
                child_frame_id=feedback.marker_name+"-shadow"
            )])


            self.menu_handler.reApply( self.interactive_marker_server )
            self.interactive_marker_server.applyChanges()

    @staticmethod
    def _create_shadow_marker(marker_name, marker_pose, frame_id, scale):
        marker_pose.position.z = 0.0
        return InteractiveMarker(
            header = Header(
                frame_id=frame_id
            ),
            pose = marker_pose,
            scale = scale,
            name = marker_name+"-shadow",
            controls = [
                InteractiveMarkerControl(
                    always_visible=True,
                    interaction_mode=InteractiveMarkerControl.NONE,
                    markers=[Marker(
                        type=Marker.CYLINDER,
                        color=ColorRGBA(
                            r=0.0,
                            g=0.0,
                            b=0.0,
                            a=0.5
                        ),
                        scale=Vector3(
                            x=0.5,
                            y=0.5,
                            z=0.01
                        )
                    )]
                )
            ]
        )

    @staticmethod
    def _create_interactive_marker(position, orientation, frame_id, child_frame_id, scale):
    
        def make_control_gizmo(name, type, qx, qy, qz, qw, fixed=False) :
            control = InteractiveMarkerControl()
            control.name = name
            n = 1/sqrt(qx**2 + qy**2 + qz**2 + qw**2) # Normalise quaternion
            control.orientation.w = qw*n
            control.orientation.x = qx*n
            control.orientation.y = qy*n
            control.orientation.z = qz*n       
            control.interaction_mode = type
            if fixed:
                control.orientation_mode = InteractiveMarkerControl.FIXED
            return control
        
        return InteractiveMarker(
            header=Header(
                frame_id=frame_id
            ),
            pose=Pose(
                position=position,
                orientation=orientation
            ),
            scale=scale,
            name=child_frame_id,
            description=child_frame_id,
            controls=[
                InteractiveMarkerControl(
                    always_visible=True,
                    interaction_mode=InteractiveMarkerControl.MOVE_ROTATE_3D,
                    markers=[Marker(
                        type=Marker.SPHERE,
                        color=ColorRGBA(
                            r=0.0,
                            g=1.0,
                            b=1.0,
                            a=1.
                        ),
                        scale=Vector3(
                            x=scale*0.5,
                            y=scale*0.5,
                            z=scale*0.5
                        )
                    )]
                ),
                make_control_gizmo("x", InteractiveMarkerControl.ROTATE_AXIS, 1, 1, 0, 0, False),
                make_control_gizmo("x", InteractiveMarkerControl.MOVE_AXIS,   1, 1, 0, 0, False),
                make_control_gizmo("y", InteractiveMarkerControl.ROTATE_AXIS, 1, 0, 0, 1, False),
                make_control_gizmo("y", InteractiveMarkerControl.MOVE_AXIS,   1, 0, 0, 1, False),
                make_control_gizmo("z", InteractiveMarkerControl.ROTATE_AXIS, 1, 0, 1, 0, False),
                make_control_gizmo("z", InteractiveMarkerControl.MOVE_AXIS,   1, 0, 1, 0, False),
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
