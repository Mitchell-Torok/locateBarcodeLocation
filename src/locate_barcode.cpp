#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "zbar_ros_interfaces/msg/symbol.hpp"
#include "cv_bridge/cv_bridge.h"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:

  MinimalSubscriber()
  : Node("minimal_subscriber")
  {  

    subscription_ = this->create_subscription<zbar_ros_interfaces::msg::Symbol>("barcode", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    
    publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("topic", 10);
    }

private:
  void topic_callback(const zbar_ros_interfaces::msg::Symbol::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Points: '%f'", msg->points[0].x);
    RCLCPP_INFO(this->get_logger(), "Data: '%s'", msg->data);
    
    
    //top left point:
    RCLCPP_INFO(this->get_logger(), "Points: '%f'", msg->points[0].x);
    RCLCPP_INFO(this->get_logger(), "Points: '%f'", msg->points[0].y);
    
    //top right point?:
    RCLCPP_INFO(this->get_logger(), "Points: '%f'", msg->points[1].x);
    RCLCPP_INFO(this->get_logger(), "Points: '%f'", msg->points[1].y);
    
    
  }
  rclcpp::Subscription<zbar_ros_interfaces::msg::Symbol>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}

