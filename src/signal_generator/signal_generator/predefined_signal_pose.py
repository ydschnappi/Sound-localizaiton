"""
create by Dian Yuan based on signal_pose.py to generate predefined signal position.
Please refer to that file for more information about each function.
"""

import rclpy
import rclpy.node
from std_msgs.msg import Float32MultiArray,Bool
import matplotlib.pyplot as plt
import numpy as np
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped

class signalpose(rclpy.node.Node):
    def __init__(self):
        super().__init__('predefind_signal_node')

        self.declare_parameter('signal_x', 4.5)
        self.declare_parameter('signal_y', 3.0)
        self.declare_parameter('signal_z', 0)

        self.publishers_pose=self.create_publisher(Float32MultiArray,'signal_pose',10)
        self.publisher_isinside=self.create_publisher(Bool,'/check_goal',10)
        self.publisher_corner=self.create_publisher(Float32MultiArray,'/exploring_point',10)

        self.subscription = self.create_subscription(
            Bool,
            'source_found',
            self.listener_callback,
            1)

        self.timer = self.create_timer(1.0, self.timer_callback)

        self.map_sub=self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            1
        )
        
        self.goal_sub=self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            1
        )
        self.estimated_x=None
        self.estimated_y=None
        
        self.last_goal_x=None
        self.last_goal_y=None
        
        self.sourseGoalList = [[4.5,3.0],[8.2,2.0],[8.5,-1.0],[6.0,6.0]] #predefined signal position
        self.goalCount = 0
        
    def goal_callback(self,msg):

        self.estimated_x=msg.pose.position.x
        self.estimated_y=msg.pose.position.y

    def map_callback(self,msg):
        
        isinside=False

        resolution=msg.info.resolution
        lowerbound_x=msg.info.origin.position.x+0.1
        upperbound_x=msg.info.origin.position.x+msg.info.width*resolution-0.1
        lowerbound_y=msg.info.origin.position.y+0.1
        upperbound_y=msg.info.origin.position.y+msg.info.height*resolution-0.1
        
        if self.estimated_x is not None and upperbound_x is not None:
            if self.estimated_x<upperbound_x and self.estimated_x>lowerbound_x and self.estimated_y<upperbound_y and self.estimated_y>lowerbound_y:
                isinside=True
                
            if self.last_goal_x is not None:
                if self.last_goal_x<upperbound_x and self.last_goal_x>lowerbound_x and self.last_goal_y<upperbound_y and self.last_goal_y>lowerbound_y:
                    msg=Bool()
                    msg.data=True
                    self.publisher_isinside.publish(msg)

            if isinside==False:
                msg=Float32MultiArray()
                self.last_goal_x = self.estimated_x
                self.last_goal_y = self.estimated_y
                if self.estimated_x>=(upperbound_x-lowerbound_x)/2 and self.estimated_y>=(upperbound_y-lowerbound_y)/2:
                    msg.data=[upperbound_x,upperbound_y]

                if self.estimated_x<(upperbound_x-lowerbound_x)/2 and self.estimated_y>=(upperbound_y-lowerbound_y)/2:
                    msg.data=[lowerbound_x,upperbound_y]

                if self.estimated_x<(upperbound_x-lowerbound_x)/2 and self.estimated_y<(upperbound_y-lowerbound_y)/2:
                    msg.data=[lowerbound_x,lowerbound_y]

                if self.estimated_x>=(upperbound_x-lowerbound_x)/2 and self.estimated_y<(upperbound_y-lowerbound_y)/2:
                    msg.data=[upperbound_x,lowerbound_y]

                # msg1=Bool()
                # msg1.data=False
                # self.publisher_isinside.publish(msg1)
                self.publisher_corner.publish(msg)    
            
    def timer_callback(self):
        signal_x = self.get_parameter('signal_x').get_parameter_value().double_value
        signal_y = self.get_parameter('signal_y').get_parameter_value().double_value
        signal_z = self.get_parameter('signal_z').get_parameter_value().double_value

        msg=Float32MultiArray()
        msg.data= [signal_x, signal_y, signal_z]
        self.publishers_pose.publish(msg)
        

        x_new = rclpy.parameter.Parameter(
            'signal_x',
            rclpy.Parameter.Type.DOUBLE,
            signal_x
        )
        y_new = rclpy.parameter.Parameter(
            'signal_y',
            rclpy.Parameter.Type.DOUBLE,
            signal_y
        )
        z_new = rclpy.parameter.Parameter(
            'signal_z',
            rclpy.Parameter.Type.DOUBLE,
            signal_z
        )
        all_new_parameters = [x_new,y_new,z_new]
        self.set_parameters(all_new_parameters)

    def listener_callback(self, msg):
        reach=msg.data

        if reach == True:
          
            new_x=rclpy.parameter.Parameter(
            'signal_x',
            rclpy.Parameter.Type.DOUBLE,
            self.sourseGoalList[self.goalCount][0]
            )
            new_y=rclpy.parameter.Parameter(
            'signal_y',
            rclpy.Parameter.Type.DOUBLE,
            self.sourseGoalList[self.goalCount][1]
            )
            all_new_coordinate= [new_x,new_y]
            self.set_parameters(all_new_coordinate)
            if self.goalCount < len(self.sourseGoalList):
                self.goalCount = self.goalCount + 1
            else:
                self.goalCount = 0
                


def main(args=None):
    rclpy.init(args=args)
    pose_node = signalpose()
    rclpy.spin(pose_node)


    pose_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
