//planner node
//author: Dian Yuan created at 15.12.2022

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "nav_msgs/msg/path.hpp"

using namespace std::chrono_literals;

#define SOUND_SPEED 343.0
#define SENSOR_DISTANCE 0.2
#define STRENGTH_LIMIT 11.0
#define GOALREACHED 0.19
#define TIME_LIMIT 90.0

//state machine
#define INITIAL 0
#define LOCATING 1
#define SEARCHING 2
#define STOP 3
#define EXPLORING 4

class Planner : public rclcpp::Node{
    private:
    //define the subscriber and publisher
        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr soundInfoSub;
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr currentPosSub;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr yawSub;
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr pathSub;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr exploringSub;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr checkGoalSub;

        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr sourceFound;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr waypointPub;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velPub;
        rclcpp::Time start_time, current_time;

        
        
        double soundStrength, last_strength, delta_t, delta_distance, yaw, max_strength;
        int state, loop_count;
        bool goal_valid;
        Eigen::Vector2d currentPos_est, source_est;
        geometry_msgs::msg::PoseStamped waypointPos; //to publish the waypoint
        geometry_msgs::msg::Quaternion quat; //to get yaw from quaternion
        geometry_msgs::msg::Twist stopMsg, forwardMsg; //to control the car

        
        
    public:
        Planner():Node("planner"),last_strength(0.0),yaw(0.0),max_strength(0.0), state(INITIAL),loop_count(0){
            soundInfoSub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
                "signal_info", 10, std::bind(&Planner::getSoundStrength, this, std::placeholders::_1));

            currentPosSub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/pose", 20, std::bind(&Planner::getPos, this,std::placeholders::_1));

            pathSub = this->create_subscription<nav_msgs::msg::Path>("/plan", 100, std::bind(&Planner::checkPath, this,std::placeholders::_1));

            yawSub = this->create_subscription<std_msgs::msg::Float32>("/yaw", 10, std::bind(&Planner::getYaw, this,std::placeholders::_1));

            exploringSub = this->create_subscription<std_msgs::msg::Float32MultiArray>("/exploring_point", 10, std::bind(&Planner::getExploringPoint, this,std::placeholders::_1));

            checkGoalSub = this->create_subscription<std_msgs::msg::Bool>("/check_goal", 10, std::bind(&Planner::checkGoal, this,std::placeholders::_1));

            waypointPub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose",10);

            velPub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",1);

            sourceFound = this->create_publisher<std_msgs::msg::Bool>("source_found",1);

            timer = this->create_wall_timer(
                500ms, std::bind(&Planner::planner_loop, this));

            waypointPos.header.frame_id = "map";
            currentPos_est << 0.0,0.0;

            stopMsg.linear.x = 0.0;
            forwardMsg.linear.x = 0.2;
            stopMsg.linear.y = forwardMsg.linear.y = 0.0;
            stopMsg.linear.z = forwardMsg.linear.z = 0.0;
            stopMsg.angular.x = forwardMsg.angular.x = 0.0;
            stopMsg.angular.y = forwardMsg.angular.y =0.0;
            stopMsg.angular.z = forwardMsg.angular.z = 0.0;

            start_time = this->now();
        }

      
      //subcribe to sound strength and calculate the source position
        void getSoundStrength(const std_msgs::msg::Float32MultiArray::SharedPtr msg){
            // RCLCPP_INFO(this->get_logger(), "state: %d", state);
            soundStrength = msg->data[0];
            if(soundStrength > max_strength){
                max_strength = soundStrength;
            }
            if(max_strength - soundStrength > 5.0){
                state = STOP;
                RCLCPP_INFO(this->get_logger(), "source lost, try relocating");
                max_strength = 0.0;
                return;
            }
            if(soundStrength >= STRENGTH_LIMIT && state != STOP){
                auto flag = std_msgs::msg::Bool();
                flag.data = true;
                sourceFound->publish(flag);
                state = STOP;
                RCLCPP_INFO(this->get_logger(), "source found");
                max_strength = 0.0;
                return;
            }
            if(state == LOCATING){
                delta_t = msg->data[1];
                delta_distance = SOUND_SPEED * delta_t;
                // RCLCPP_INFO(this->get_logger(), "delta distance: %f", delta_distance);
                calSourcePos(soundStrength,delta_distance);
                max_strength = 0.0;
            }

        }
        
        //get the current pose from slam tool box
        void getPos(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){
            currentPos_est(0) = msg->pose.pose.position.x;
            currentPos_est(1) = msg->pose.pose.position.y;
            // RCLCPP_INFO(this->get_logger(), "current position: %f, %f", currentPos_est(0), currentPos_est(1));
            quat = msg->pose.pose.orientation;
            tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
            tf2::Matrix3x3 m(q);
            double r, p, y;
            m.getRPY(r, p, y);
            // RCLCPP_INFO(this->get_logger(), "current yaw: %f", y);
        }


        //check if the car has reached the goal pose (there should be a better way to do this with some nav2 msg but we failed to find it) 
        void checkPath(const nav_msgs::msg::Path::SharedPtr msg){
            Eigen::Vector2d pos1, pos_last;
            pos1<< msg->poses[0].pose.position.x, msg->poses[0].pose.position.y;
            pos_last<< msg->poses[msg->poses.size()-1].pose.position.x, msg->poses[msg->poses.size()-1].pose.position.y;
            if(checkDistance(pos1, pos_last, GOALREACHED) && soundStrength < STRENGTH_LIMIT && state == SEARCHING){
                state = STOP;
                RCLCPP_INFO(this->get_logger(), "Try Relocating");
            }
        }

        //check if the goal can be navigated with nav2
        void checkGoal(const std_msgs::msg::Bool::SharedPtr msg){
            goal_valid = msg->data;
        }

        //get the waypoint to explore the map
        void getExploringPoint(const std_msgs::msg::Float32MultiArray::SharedPtr msg){
            state = EXPLORING;
            source_est << msg->data[0], msg->data[1];
            RCLCPP_INFO(this->get_logger(), "Goal not reachable, try exploring the map");
        }

        //state machine and publish the goal pose
        void planner_loop(){
            RCLCPP_INFO_ONCE(this->get_logger(), "planner is running");

            //record the time
            current_time = this->now();
            if(start_time != current_time){
                auto duration = current_time - start_time;
                if(duration.seconds() > TIME_LIMIT){
                    state = STOP;
                    RCLCPP_INFO(this->get_logger(), "Time out, try new task");
                    auto flag = std_msgs::msg::Bool();
                    flag.data = true;
                    start_time = this->now();
                    sourceFound->publish(flag);
                }
            }

            switch (state)
            {
                case INITIAL:
                    if(source_est(0) == 0.0 && source_est(1) == 0.0){
                        source_est(0) = currentPos_est(0) + 0.7;
                    }
                    velPub->publish(forwardMsg);
                    state = LOCATING;
                    break;
                
                case LOCATING:
                    loop_count++;
                    return;
                    break;

                case SEARCHING:
                    break;
                
                case STOP:
                    // RCLCPP_INFO(this->get_logger(), "stop at current position: %f, %f, %f", currentPos_est(0), currentPos_est(1), yaw);
                    velPub->publish(stopMsg);
                    loop_count++;
                    if(loop_count >= 4){
                        state = INITIAL;
                        loop_count = 0;
                    }
                    waypointPub->publish(waypointPos);
                    // return;
                    source_est(0) = currentPos_est(0);
                    source_est(1) = currentPos_est(1);
                    break;
                
                case EXPLORING:
                    if(goal_valid){
                        state = LOCATING;
                        RCLCPP_INFO(this->get_logger(), "Goal reachable, try searching the source");
                    }
                    break;
            }
            
            waypointPos.pose.position.x = source_est(0);
            waypointPos.pose.position.y = source_est(1);
            waypointPos.pose.orientation = quat;
            waypointPos.header.stamp = this->now();
            waypointPub->publish(waypointPos);
            // RCLCPP_INFO(this->get_logger(), "source position: %f, %f, %d, %f", source_est(0), source_est(1),state, soundStrength);
            // RCLCPP_INFO(this->get_logger(), "current position and yaw: %f, %f, %f", currentPos_est(0), currentPos_est(1), yaw);

        }

      
        //calculate the source position
        void calSourcePos(double strength, double delta_distance){

            if(loop_count < 6){
                if(loop_count == 0){
                    last_strength = strength;
                }
                velPub->publish(forwardMsg);
                source_est(0) = currentPos_est(0) + cos(yaw);
                source_est(1) = currentPos_est(1) + sin(yaw);
                return;
            }

            double distance_est = 10.0/strength-0.01;
            // RCLCPP_INFO(this->get_logger(), "distance_est: %f", distance_est);
            double x2_est = (pow(distance_est,2)-pow(delta_distance,2)/4)/(1+pow(delta_distance,2)/(pow(SENSOR_DISTANCE*2,2)-pow(delta_distance,2)));
            double x_car_est,y_car_est;
            y_car_est = delta_distance >= 0 ? -sqrt(pow(distance_est,2)-x2_est) : sqrt(pow(distance_est,2)-x2_est);
            x_car_est = strength >= last_strength ? sqrt(x2_est) : -sqrt(x2_est);
            source_est(0) = x_car_est*cos(yaw) - y_car_est*sin(yaw) + currentPos_est(0);
            source_est(1) = x_car_est*sin(yaw) + y_car_est*cos(yaw) + currentPos_est(1);
            state = SEARCHING;
            start_time = this->now();
            loop_count = 0;
        }

        //check if the two positins are close with in a certain distance
        bool checkDistance(Eigen::Vector2d pos1, Eigen::Vector2d pos2, double distance){
            // RCLCPP_INFO(this->get_logger(), "distance: %f", sqrt(pow(pos1(0)-pos2(0),2)+pow(pos1(1)-pos2(1),2)));
            if(sqrt(pow(pos1(0)-pos2(0),2)+pow(pos1(1)-pos2(1),2)) < distance){ 
                return true;
            }
            return false;
        }

        void getYaw(const std_msgs::msg::Float32::SharedPtr msg){
            yaw = msg->data;
        }
};

int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<Planner>());
    rclcpp::shutdown();
    return 0;
}

