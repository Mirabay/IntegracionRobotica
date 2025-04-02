import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import transforms3d
import numpy as np

class PuzzlebotSim(Node):
    def __init__(self):
        super().__init__('puzzlebot_sim')
        
        # Inicializar broadcasters para TF estáticas y dinámicas
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.tf_dynamic_broadcaster = TransformBroadcaster(self)
        
        # Variable para controlar la rotación de las ruedas
        self.wheel_angle = 0.0
        self.rotation_speed = 0.5  # radianes por segundo
        self.base_footprint_angle = 0.0
        # Publicar transformaciones estáticas al inicio
        self.publish_static_transforms()
        
        # Timer para transformaciones dinámicas (10 Hz)
        self.timer = self.create_timer(0.1, self.publish_dynamic_transforms)

    def publish_static_transforms(self):
        # Transformaciones estáticas (solo se publican una vez)
        static_transforms = [
            self.create_transform(
                parent_frame="map",
                child_frame="odom",
                x=0.5, y=0.0, z=0.0,
                roll=0.0, pitch=0.0, yaw=0.0
            ),
            self.create_transform(
                parent_frame="base_footprint",
                child_frame="base_link",
                x=0.0, y=0.0, z=0.05,
                roll=0.0, pitch=0.0, yaw=0.0
            ),
            self.create_transform(
                parent_frame="base_link",
                child_frame="caster_link",
                x=-.095, y=0.0, z=-0.03,
                roll=0.0, pitch=0.0, yaw=0.0
            )
        ]
        self.tf_static_broadcaster.sendTransform(static_transforms)

    def publish_dynamic_transforms(self):
        # Actualizar ángulo de las ruedas
        self.wheel_angle += self.rotation_speed * 0.1  # delta_time = 0.1s (10Hz)
        self.base_footprint_angle += self.rotation_speed * 0.5
        # Transformaciones dinámicas (se actualizan periódicamente)
        dynamic_transforms = [
            self.create_transform(
                parent_frame="odom",
                child_frame="base_footprint",
                x=0.5, y=0.0, z=0.0,
                roll=0.0, pitch= 0.0, yaw=0.0,
                is_dynamic=True
            ),
            self.create_transform(
                parent_frame="base_link",
                child_frame="wheel_left_link",
                x=0.052, y=0.095, z=-0.0025,
                roll=0.0, pitch=self.wheel_angle, yaw=0.0,
                is_dynamic=True
            ),
            self.create_transform(
                parent_frame="base_link",
                child_frame="wheel_right_link",
                x=0.052, y=-0.095, z=-0.0025,
                roll=0.0, pitch=self.wheel_angle, yaw=0.0,
                is_dynamic=True
            )
        ]
        self.tf_dynamic_broadcaster.sendTransform(dynamic_transforms)

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

def main():
    rclpy.init()
    node = PuzzlebotSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()