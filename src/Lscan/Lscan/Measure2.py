import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist



###Reads out laser scan data
##test node used for checking whether the explorer works properly
class ReadingLaser(Node):

    def __init__(self):
        super().__init__('reading_laser')



        self.subscription = self.create_subscription(LaserScan, 'scan', self.listener_callback, 10)


    def listener_callback(self,msg):
        self.get_logger().info('I heard : back range "%f" left range "%f" front range "%f" right range "%f"' %(msg.ranges[355] ,msg.ranges[270], msg.ranges[180], msg.ranges[90]))


def main(args=None):
    rclpy.init()
    reading_laser = ReadingLaser()                  
    reading_laser.get_logger().info("message recieved")
    rclpy.spin(reading_laser)


    reading_laser.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
