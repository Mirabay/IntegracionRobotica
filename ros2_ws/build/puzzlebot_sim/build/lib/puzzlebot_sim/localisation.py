import rclpy 
from rclpy.node import Node 
from nav_msgs.msg import Odometry 
from std_msgs.msg import Float32 
from rclpy import qos 
import numpy as np 
import transforms3d 

 

class Localisation(Node): 

    def __init__(self): 

        super().__init__('localisation') 

        # Create subscribers to the /wr and /wl topics 
        self.wr_sub= self.create_subscription(Float32, 'wr', self.wr_callback, qos.qos_profile_sensor_data) 
        self.wl_sub= self.create_subscription(Float32, 'wl', self.wl_callback, qos.qos_profile_sensor_data) 

        # Create a publisher for the robot's pose 
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10) 

        ############ ROBOT CONSTANTS ################  
        self.r=0.05 #puzzlebot wheel radius [m] 
        self.L = 0.19 #puzzlebot wheel separation [m] 

        ############ Variables ############### 
        self.w = 0.0 # robot's angular speed [rad/s] 
        self.v = 0.0 #robot's linear speed [m/s] 

        self.x = 0.0 # robot's x position [m] 
        self.y = 0.0 # robot's y position [m] 
        self.theta = 0.0 # robot's yaw angle [rad] 
        self.wr = 0.0 # right wheel speed [rad/s] 
        self.wl = 0.0 # left wheel speed [rad/s] 
        self.odom = Odometry() # ROS message to publish the robot's pose 
        self.prev_time = 0.0 # previous time [s]

        
        self.prev_time = self.get_clock().now().nanoseconds # Get the current time [ns]
        self.get_logger().info("Localisation node started")
 

 

        # Create a timer to update the robot's pose 

        timer_period = 0.02 # Desired time to update the robot's pose [s] 

        self.timer = self.create_timer(timer_period, self.timer_callback) 

        # Initialize the robot's pose 

    

        #WRITE YOUR CODE HERE 

     

    def timer_callback(self): 
        
        # Get the robot's linear and angular speeds from the wheel speeds
        v, w = self.get_robot_vel(self.wr, self.wl)
        
        self.update_pose(v,w) # Update the robot's pose
        
        # Fill the odom message with the robot's pose 
        self.odom = self.fill_odom_message(self.x, self.y, self.theta) 
        # Publish the odometry message 
        self.odom_pub.publish(self.odom) 

     

    def wr_callback(self, msg): 

        # Get the right wheel speed from the message 
        self.wr = msg.data 

 

    def wl_callback(self, msg): 
        
        # Get the left wheel speed from the message 
        self.wl = msg.data 

     

    def get_robot_vel(self, wr, wl): 

        # Calculate the linear and angular velocities based on the wheel speeds 
        v = self.r * (wr + wl) / 2.0 # linear speed [m/s]
        w = self.r * (wr - wl) / self.L # angular speed [rad/s]
        print('v:', v, 'w:', w)
        return v, w 

     

    def update_pose(self, v, w): 

        #This functions receives the robot speed v [m/s] and w [rad/s] 
        # and gets the robot's pose (x, y, theta) where (x,y) are in [m] and (theta) in [rad] 
        # is the orientation,     
        ############ MODIFY THIS CODE   ################ 
        
        
        dt = (self.get_clock().now().nanoseconds - self.prev_time)*10e-9 # Get the time since the last update [s]
        
        self.x = self.x + v * np.cos(self.theta) * dt # Update x position [m]
        self.y = self.y + v * np.sin(self.theta) * dt
        self.theta = self.theta + w * dt
        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta)) # Limit the angle to [-pi, pi]
        self.prev_time = self.get_clock().now().nanoseconds # Update the previous time [s]

        # Prints
        print(f'Pose: x={self.x:.2f}, y={self.y:.2f}, theta={self.theta:.2f}')
     

    def fill_odom_message(self, x, y, yaw): 

        # Create a new Odometry message 
        odom_msg = Odometry() 

        # Fill the message with the robot's pose 

        odom_msg.header.stamp = self.get_clock().now().to_msg() # Get the current time
        odom_msg.header.frame_id = 'odom' # Set the frame id 
        odom_msg.child_frame_id = 'base_link' # Set the child frame id 
        odom_msg.pose.pose.position.x = x # x position [m] 
        odom_msg.pose.pose.position.y = y # y position [m] 
        odom_msg.pose.pose.position.z = 0.05 # z position [m]
         
        # Set the orientation using quaternion 
        # Convert the yaw angle to a quaternion 

        quat = transforms3d.euler.euler2quat(0, 0, yaw)  
        odom_msg.pose.pose.orientation.w = quat[0] 
        odom_msg.pose.pose.orientation.x = quat[1] 
        odom_msg.pose.pose.orientation.y = quat[2] 
        odom_msg.pose.pose.orientation.z = quat[3] 
        return odom_msg 

 

def main(args=None): 

    rclpy.init(args=args) 
    node = Localisation() 

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