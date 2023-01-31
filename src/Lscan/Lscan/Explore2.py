from random import random

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

distance_from_wall = 1.2
collision_distance = 0.45

class Subscriber(Node):

    def __init__(self):
        super().__init__('subscriber')
        self.subscription = self.create_subscription(LaserScan, 'scan', self.listener_callback, 10)
        self.subscription
        self.ranges = [0.0] * 180
        self.forward_distance = 1000.0
        self.left_forward_distance = 1000.0
        self.right_forward_distance = 1000.0
        self.left_distance = 1000.0
        self.back_distance = 1000.0
        self.right_distance = 1000.0

    def listener_callback(self, msg):
        # print("Forward distance: " + str(msg.ranges[0]))
        # print("Left distance: " + str(msg.ranges[90]))
        # print("Back distance: " + str(msg.ranges[180]))
        # print("Right distance: " + str(msg.ranges[270]))

        self.forward_distance = msg.ranges[180]
        self.left_distance = msg.ranges[270]
        self.back_distance = msg.ranges[357]
        self.right_distance = msg.ranges[90]
        self.left_forward_distance = msg.ranges[210]
        self.right_forward_distance = msg.ranges[150]


class Publisher(Node):
    def __init__(self):
        super().__init__('publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)


def reset_commands(command):
    """
    :param command: input Twist message
    :return: command: returns reset Twist message
    """
    command.linear.x = 0.0
    command.linear.y = 0.0
    command.linear.z = 0.0
    command.angular.x = 0.0
    command.angular.y = 0.0
    command.angular.z = 0.0

    return command

def check_collision(subscriber, publisher, command):

    """
    checks wheter the car is too close to an obstacle and moves it away
    """
    command = reset_commands(command)
    rclpy.spin_once(subscriber)
    if subscriber.forward_distance < collision_distance:
    	while subscriber.forward_distance < collision_distance:
    	    rclpy.spin_once(subscriber)
    	    speed=-1
    	    command.linear.x = speed
    	    publisher.publisher_.publish(command)
    	    publisher.get_logger().info("Moving backwards...")
    	
    elif subscriber.left_distance < collision_distance-0.07 or subscriber.left_forward_distance < collision_distance-0.07:
    	while subscriber.left_distance < collision_distance-0.07:
    	    rclpy.spin_once(subscriber)
    	    command.angular.z = -0.4
    	    publisher.publisher_.publish(command)
    	    publisher.get_logger().info("Rotating slightly right...")	
    	    
    elif subscriber.right_distance < collision_distance-0.07 or subscriber.right_forward_distance < collision_distance-0.07:
    	while subscriber.right_distance < collision_distance-0.07:
    	    rclpy.spin_once(subscriber)
    	    command.angular.z = 0.4
    	    publisher.publisher_.publish(command)
    	    publisher.get_logger().info("Rotating slightly left...")

def check_ranges(subscriber):
    """
    returns boolean value of whether all sensors are above distance from wall.
    :param subscriber:
    :return: boolean value of whether the bot is clear to go forward
    :return: float value of the smallest sensor reading
    """
    
    rclpy.spin_once(subscriber)
    readings = [subscriber.forward_distance, subscriber.left_forward_distance, subscriber.right_forward_distance]
    min_value = min(readings)
    if subscriber.forward_distance < distance_from_wall:  # if there is a wall in front, dont go forward
        subscriber.get_logger().info("Obstacle detected")
        return False, min_value
    else:
        if (subscriber.right_forward_distance > distance_from_wall and
                subscriber.left_forward_distance > distance_from_wall):  # if theres no wall close to the sides,
            # go forward
            return True, min_value
        else:
            if (subscriber.left_forward_distance > subscriber.left_distance or  # if the robot is aiming away from
                    # the wall, go forward
                    subscriber.right_forward_distance > subscriber.right_distance):
                return True, min_value
            else:
                subscriber.get_logger().info("Obstacle detected")
                return False, min_value


def go_forward_until_obstacle(subscriber, publisher, command):
    """
    :param subscriber: subscriber object that is subscribed to /scan
    :param publisher: publisher object that that is publishing in /cmd_vel
    :param command: /Twist message object
    :return: nothing
    """
    command = reset_commands(command)

    while check_ranges(subscriber)[0]:  # while obstacles are not present or robot is aiming away from wall, go forward
        rclpy.spin_once(subscriber)
        speed = check_ranges(subscriber)[1] / 2.5  # robot speed depends on distance to closest obstacle
        if speed > 2.2:  # max speed
            speed = 2.2
        command.linear.x = speed
        publisher.get_logger().info('Going forward at %s m/s.' % round(speed, 2))
        publisher.publisher_.publish(command)
        check_collision(subscriber, publisher, command)
    command = reset_commands(command)
    publisher.publisher_.publish(command)


def rotate_until_clear(subscriber, publisher, command):
    """
    :param subscriber: subscriber object that is subscribed to /scan
    :param publisher: publisher object that that is publishing in /cmd_vel
    :param command: /Twist message object
    :return: nothing
    """

    command = reset_commands(command)
    rclpy.spin_once(subscriber)
    check_collision(subscriber, publisher, command)
    	
    if subscriber.left_forward_distance < subscriber.right_forward_distance:  # if the robot has the wall to his left
        while subscriber.left_forward_distance < subscriber.left_distance or subscriber.forward_distance < distance_from_wall:
            rclpy.spin_once(subscriber)
            command.angular.z = -0.9 #+ (random()*0.3)
            publisher.publisher_.publish(command)
            publisher.get_logger().info("Rotating right...")
    else:
        while subscriber.right_forward_distance < subscriber.right_distance or subscriber.forward_distance < distance_from_wall:
            rclpy.spin_once(subscriber)
            command.angular.z = 0.9 #- (random()*0.3)
            publisher.publisher_.publish(command)
            publisher.get_logger().info("Rotating left...")

    subscriber.get_logger().info("Clear.")

    command = reset_commands(command)
    publisher.publisher_.publish(command)


def main(args=None):
    rclpy.init(args=args)

    subscriber = Subscriber()
    publisher = Publisher()
    command = Twist()

    while 1:  # main loop. The robot goes forward until obstacle, and then turns until its free to advance, repeatedly.
        go_forward_until_obstacle(subscriber, publisher, command)
        rotate_until_clear(subscriber, publisher, command)

    rclpy.spin(subscriber)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    subscriber.destroy_node()
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
