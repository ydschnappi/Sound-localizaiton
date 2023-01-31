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
        super().__init__('signal_pose_node')

        self.declare_parameter('signal_x', 3.5)
        self.declare_parameter('signal_y', 1.0)
        self.declare_parameter('signal_z', 0)

        self.publishers_pose=self.create_publisher(Float32MultiArray,'signal_pose',10) #publish the true signal position
        self.publisher_isinside=self.create_publisher(Bool,'/check_goal',10) #publish if the current signal source is inside the map range
        self.publisher_corner=self.create_publisher(Float32MultiArray,'/exploring_point',10) #publish the intermediate way point for the exploration

        self.subscription = self.create_subscription(
            Bool,
            'source_found',
            self.listener_callback,
            1) #subscribe to the topic to check if the TAS car reached the signal source

        self.timer = self.create_timer(1.0, self.timer_callback)

        self.map_sub=self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            1
        ) #subscribe to the map 
        self.goal_sub=self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.goal_callback,
            1
        ) #subscribe to the estimated signal position 
        self.estimated_x=None
        self.estimated_y=None #estimated coordinate of the signal
        
        self.last_goal_x=None
        self.last_goal_y=None
        
    def goal_callback(self,msg):

        self.estimated_x=msg.pose.position.x
        self.estimated_y=msg.pose.position.y #Save the estimated signal position into variable

    def map_callback(self,msg):
            
            isinside=False

            resolution=msg.info.resolution
            lowerbound_x=msg.info.origin.position.x+0.1
            upperbound_x=msg.info.origin.position.x+msg.info.width*resolution-0.1
            lowerbound_y=msg.info.origin.position.y+0.1
            upperbound_y=msg.info.origin.position.y+msg.info.height*resolution-0.1
            
            if self.estimated_x is not None and upperbound_x is not None:
                if self.estimated_x<upperbound_x and self.estimated_x>lowerbound_x and self.estimated_y<upperbound_y and self.estimated_y>lowerbound_y:
                    # msg=Bool()
                    # msg.data=True
                    # self.publisher_isinside.publish(msg)
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
        self.publishers_pose.publish(msg)#publish the true signal position into signal pose topic
        # self.get_logger().info('Signal pose: %s' % msg.data) 
        

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
          
            
            info_rel_path = get_package_share_directory('signal_generator')+'/LSR_N5_basement.yaml'
            with open(info_rel_path, 'r') as file:
                map_info = yaml.safe_load(file)
            resolution=map_info['resolution']
            origin_info=map_info['origin']
            origin_x=origin_info[0]
            origin_y=origin_info[1]

            
            map_rel_path = get_package_share_directory('signal_generator')+'/LSR_N5_basement.pgm'
            with open(map_rel_path, 'rb') as pgmf:
                im = plt.imread(pgmf)
            #read the map information from the pgm file

            [row,col]=np.shape(im) #pixel size of the map
            origin_pixel=[-origin_x/resolution,(row+origin_y/resolution)] #the pixel position of origin of the map

            valid_pose=np.where(im==254) #valid pixel should have a value of 254
            valid_num=len(valid_pose[0])
            rand_index=np.random.randint(1,valid_num)-1
            selected_pose_pixel=[valid_pose[1][rand_index]+1,valid_pose[0][rand_index]+1] #randomly select a pixel with value 254 as the new signal position

            selected_coordinate=[(selected_pose_pixel[0]-origin_pixel[0])*resolution,(origin_pixel[1]-selected_pose_pixel[1])*resolution] #transform the pixel coordinate to the map coordinate

            new_x=rclpy.parameter.Parameter(
            'signal_x',
            rclpy.Parameter.Type.DOUBLE,
            selected_coordinate[0]
        )
            new_y=rclpy.parameter.Parameter(
            'signal_y',
            rclpy.Parameter.Type.DOUBLE,
            selected_coordinate[1]
        )
            all_new_coordinate= [new_x,new_y]
            self.set_parameters(all_new_coordinate) #change the signal position into new selected signal source point
            # if self.goalCount < len(self.sourseGoalList):
            #     self.goalCount = self.goalCount + 1
            


def main(args=None):
    rclpy.init(args=args)
    pose_node = signalpose()
    rclpy.spin(pose_node)


    pose_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
