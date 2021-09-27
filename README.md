# interactive_transform_publisher
ROS2 package for creating interactive markers and their associated transformations frames.

Run server:
ros2 run interactive_transform_publisher interactive_transform_publisher

Add interactive transforms at origin:
ros2 service call /interactive_transform_publisher/add_interactive_static_transform interactive_transform_publisher_msgs/srv/FramesTransform "{frame_id: FRAME_ID, child_frame_id: CHILD_ID}"
