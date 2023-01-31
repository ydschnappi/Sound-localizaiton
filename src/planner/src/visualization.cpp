#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

class VisualizationNode : public rclcpp::Node
{
public:
  VisualizationNode() : Node("subscriber_node")
  {
    marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
    signal_pose_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("/signal_pose", 10, std::bind(&VisualizationNode::poseCallback, this, std::placeholders::_1));
  }

private:
  void poseCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.ns = "visualization";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = msg->data[0];
    marker.pose.position.y = msg->data[1];
    marker.pose.position.z = msg->data[2];
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    marker.lifetime = rclcpp::Duration(0, 1000000000);
    marker_publisher_->publish(marker);
  }

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr signal_pose_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisualizationNode>());
  rclcpp::shutdown();
  return 0;
}
