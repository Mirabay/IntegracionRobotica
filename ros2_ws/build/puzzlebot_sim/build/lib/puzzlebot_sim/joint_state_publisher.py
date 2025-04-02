
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
import transforms3d
import numpy as np

class join_state_publisher(Node):

    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher= self.create_publisher(JointState, '/joint_states', 10)

        #Create a Timer
        timer_period = 0.1 #seconds
        self.timer = self.create_timer(timer_period, self.timer_cb)

        # Initialize the TransformBroadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        # Intitialize static TransformBroadcaster
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        
        # Initialize Messages to be published
        self.ctrJoints = JointState()
        self.ctrJoints.header.stamp = self.get_clock().now().to_msg()
        self.ctrJoints.name = ['wheel_left_joint', 'wheel_right_joint']
        self.ctrJoints.position= [0.0] * 2
        self.ctrJoints.velocity= [1.0] * 2
        self.ctrJoints.effort= [0.0] * 2

        self.publishStaticTransforms()
    #Timer Callback
    def timer_cb(self):
        time = self.get_clock().now().nanoseconds/1e9
        
        
        self.ctrJoints.header.stamp = self.get_clock().now().to_msg()
        
        #Update the Joint States
        
        self.ctrJoints.position[0] = -np.sin(time)
        self.ctrJoints.position[1] = np.sin(time)
        
        self.tf_broadcaster.sendTransform(self.create_transform(
            parent_frame="odom",
            child_frame="base_footprint",
            x=0.5, y=0.0, z=0.0,
            roll=0.0, pitch= 0.0, yaw=np.sin(time),
            is_dynamic=True
        ))
        
        self.publisher.publish(self.ctrJoints)
        
    def publishStaticTransforms(self):
        static_transforms = [
            self.create_transform(
                parent_frame="map",
                child_frame="odom",
                x=0.5, y=0.0, z=0.0,
                roll=0.0, pitch=0.0, yaw=0.0
            )]
        self.tf_static_broadcaster.sendTransform(static_transforms)
    def create_transform(self, parent_frame, child_frame, 
                        x, y, z, roll, pitch, yaw, 
                        is_dynamic=False):
        tf = TransformStamped()
        
        # Cabecera con timestamp (importante para TF dinámicas)
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = parent_frame
        tf.child_frame_id = child_frame
        
        # Transformación de traslación
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = z
        
        # Convertir ángulos Euler a cuaternión
        q = transforms3d.euler.euler2quat(roll, pitch, yaw)
        tf.transform.rotation.x = q[1]
        tf.transform.rotation.y = q[2]
        tf.transform.rotation.z = q[3]
        tf.transform.rotation.w = q[0]
        
        return tf

def main(args=None):
    rclpy.init(args=args)

    node = join_state_publisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():  # Ensure shutdown is only called once
            rclpy.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()