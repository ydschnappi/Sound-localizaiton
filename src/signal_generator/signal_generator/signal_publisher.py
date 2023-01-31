import math

from std_msgs.msg import Float32MultiArray,Float32
import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class signal_pub(Node):
    def __init__(self):
        super().__init__('signal_publisher_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer,self)
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'signal_pose',
            self.listener_callback,
            10)

        self.timer=self.create_timer(0.3,self.on_time)

        self.bot_pose_x=None
        self.bot_pose_y=None
        self.bot_pose_z=None
        
        self.signal_pose_x= None
        self.signal_pose_y= None
        self.signal_pose_z= None

        self.bot_rotation=None

        self.d=0.2 #the defined distance between the vitual sensors and the TAS car
        self.sound_speed=343.0

        self.orientation=self.create_publisher(Float32,'yaw',10)
        # self.dist_publisher_r=self.create_publisher(Float32,'signal_dist_r',10)
        
        self.signal_info_pub = self.create_publisher(Float32MultiArray,'signal_info',10)# publish the signal info for the planner


    def listener_callback(self, msg):
        pose=msg.data

        self.signal_pose_x=pose[0]
        self.signal_pose_y=pose[1]
        self.signal_pose_z=pose[2] # save the True signal position from the topic into local variable
    def on_time(self):
        try:
            # t=self.tf_buffer.lookup_transform('map','base_footprint',rclpy.time.Time(),timeout=rclpy.duration.Duration(seconds=15.0))
            t=self.tf_buffer.lookup_transform('map','base_footprint',rclpy.time.Time())
            
            self.bot_pose_x=t.transform.translation.x
            self.bot_pose_y=t.transform.translation.y
            self.bot_pose_z=t.transform.translation.z #TAS car position

            w=t.transform.rotation.w
            x=t.transform.rotation.x
            y=t.transform.rotation.y
            z=t.transform.rotation.z
            self.bot_rotation=math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y)) #TAS car rotation (Yaw)
            

            msg3=Float32MultiArray()
            msg3.data= [t.transform.translation.x, t.transform.translation.y,self.bot_rotation]
            # self.get_logger().info('Robot pose: %s' % msg3.data)
            orientation=Float32()
            orientation.data=self.bot_rotation
            self.orientation.publish(orientation)
            
            # msg1=Float32()
            if self.signal_pose_x is not None:
                right_sensor_pos_x=self.bot_pose_x+self.d*math.sin(self.bot_rotation)
                right_sensor_pos_y=self.bot_pose_x-self.d*math.cos(self.bot_rotation) 
            
                signal_strength = 10.0/(math.sqrt((self.bot_pose_x-self.signal_pose_x)**2+(self.bot_pose_y-self.signal_pose_y)**2)+0.01)

                left_sensor_pos_x=self.bot_pose_x-self.d*math.sin(self.bot_rotation)
                left_sensor_pos_y=self.bot_pose_x+self.d*math.cos(self.bot_rotation)
                distance_r=math.sqrt((self.signal_pose_x-right_sensor_pos_x)**2+(self.signal_pose_y-right_sensor_pos_y)**2) #calculate the signal info based on the distance between sensors and the signal sources
            # msg1.data=distance_r
            # self.dist_publisher_r.publish(msg1)
            # self.get_logger().info('Distance between signal and right sensor: "%s"' % msg1.data)

            # msg2=Float32()
                distance_l=math.sqrt((self.signal_pose_x-left_sensor_pos_x)**2+(self.signal_pose_y-left_sensor_pos_y)**2)
            # msg2.data=distance_l
            # self.dist_publisher_l.publish(msg2)
            # self.get_logger().info('Distance between signal and left sensor: "%s"' % msg2.data)
            
                delta_t = (distance_l-distance_r)/self.sound_speed
            
                msg=Float32MultiArray()
                msg.data= [signal_strength, delta_t]
                # self.get_logger().info('msg: %s' % msg.data)
                self.signal_info_pub.publish(msg) #publish the signal info



        except TransformException as ex:
            self.get_logger().info(f'cannot listen to tf: {ex}')
            return

def main():
    rclpy.init()
    node = signal_pub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
