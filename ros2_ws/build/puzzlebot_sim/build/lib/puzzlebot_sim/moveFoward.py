import rclpy 
from rclpy.node import Node 
from geometry_msgs.msg import Twist  
from std_msgs.msg import Float32 


class MoveForwardClass(Node): 

    def __init__(self): 

        super().__init__('move_forward') 
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10) 
        self.sub = self.create_subscription(Float32, "distance", self.distance_callback, 10) 
        timer_period = 0.05 
        self.timer = self.create_timer(timer_period, self.timer_callback) 
        self.get_logger().info("Node initialized!!!") 
        self.vel = Twist() 
        self.distance = 0.0 
        self.state = "stop" 
        self.distance_received = False 
        self.v = 0.3 #[m/s] 

    def timer_callback(self): 

        if self.state == "stop": 
            self.vel.linear.x = 0.0 
            self.vel.angular.z = 0.0 

            if self.distance_received == True: 
                self.state = "move_linear" 
                print("changed to move linear") 
                self.t = int((self.distance/self.v)*10**9) #time in [nanoseconds] 
                self.start_time = self.get_clock().now().nanoseconds #Get the current time in nanoseconds 

        elif self.state == "move_linear": 
            self.vel.linear.x  = self.v 
            self.vel.angular.z = 0.0 
            current_time=self.get_clock().now().nanoseconds  

            if current_time - self.start_time >= self.t: 
                print("changed to stop") 
                self.state = "stop" 
                self.distance_received = False 

        self.cmd_vel_pub.publish(self.vel) #publish the message 
     

    def distance_callback(self, msg): 

        ## This function receives the ROS message as the msg variable.  
        self.distance =  msg.data #msg.data is the string contained inside the ROS message  
        self.distance_received = True 
        print("I received this message in the callback: " + str(self.distance)) 

 

 

def main(args=None): 

    rclpy.init(args=args) 
    m_p=MoveForwardClass() 
    rclpy.spin(m_p) 
    m_p.destroy_node() 
    rclpy.shutdown() 

     

if __name__ == '__main__': 

    main() 