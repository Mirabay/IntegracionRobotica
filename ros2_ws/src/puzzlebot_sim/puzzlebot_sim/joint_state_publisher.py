import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
import transforms3d
import numpy as np

class JointStatePublisher(Node):

    def __init__(self):
        super().__init__('joint_state_publisher')
        
        # Configuration parameters
        self.wheel_radius = 0.05  # Same as localization node
        self.base_height = 0.05   # Height from base_link to ground
        
        # Setup publishers and timers
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        
        self.create_timer(0.1, self.timer_callback)
        self.publish_static_transforms()
        
        # Joint state initialization
        self.joint_state = JointState()
        self.joint_state.name = ['wheel_left_joint', 'wheel_right_joint']
        self.joint_state.position = [0.0, 0.0]
        self.joint_state.velocity = [0.0, 0.0]
        self.joint_state.effort = [0.0, 0.0]
        
        self.start_time = self.get_clock().now().nanoseconds / 1e9

    def publish_static_transforms(self):
        static_transforms = [
            self.create_transform(
                parent_frame="base_link",
                child_frame="base_footprint",
                x=0.0, 
                y=0.0, 
                z=self.base_height,
                roll=0.0,
                pitch=0.0,
                yaw=0.0
            )
        ]
        self.tf_static_broadcaster.sendTransform(static_transforms)

    def timer_callback(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed_time = current_time - self.start_time
        
        # Update joint positions (example using sinusoidal motion)
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.joint_state.position[0] = np.sin(elapsed_time)  # Left wheel
        self.joint_state.position[1] = np.cos(elapsed_time)  # Right wheel
        
        self.joint_pub.publish(self.joint_state)

    def create_transform(self, parent_frame, child_frame, 
                        x, y, z, roll, pitch, yaw):
        transform = TransformStamped()
        
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame
        
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = z
        
        q = transforms3d.euler.euler2quat(roll, pitch, yaw)
        transform.transform.rotation.x = q[1]
        transform.transform.rotation.y = q[2]
        transform.transform.rotation.z = q[3]
        transform.transform.rotation.w = q[0]
        
        return transform

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()