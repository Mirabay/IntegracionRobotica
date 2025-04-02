
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
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

        # Initialize Messages to be published
        self.ctrJoints = JointState()
        self.ctrJoints.header.stamp = self.get_clock().now().to_msg()
        self.ctrJoints.name = ['odom_to_base_footprint','wheel_left_joint', 'wheel_right_joint']
        self.ctrJoints.position= [0.0] * 3
        self.ctrJoints.velocity= [1.0] * 3
        self.ctrJoints.effort= [0.0] * 3

    #Timer Callback
    def timer_cb(self):
        time = self.get_clock().now().nanoseconds/1e9
        
        
        self.ctrJoints.header.stamp = self.get_clock().now().to_msg()
        
        #Update the Joint States
        self.ctrJoints.position[0] = np.sin(time)
        self.ctrJoints.position[1] = time* 0.5
        self.ctrJoints.position[2] = time* 0.5
        
        self.publisher.publish(self.ctrJoints)



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