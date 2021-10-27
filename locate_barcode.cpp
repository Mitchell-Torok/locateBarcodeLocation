#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "zbar_ros_interfaces/msg/symbol.hpp"
#include "cv_bridge/cv_bridge.h"
#include <sensor_msgs/msg/laser_scan.hpp>

#include <string>

#include<cmath>

using std::placeholders::_1;
using namespace std;



std::string markers[10] = {"","","","","","","","","",""};



class MinimalSubscriber : public rclcpp::Node {
	

public:
  
  MinimalSubscriber() : Node("minimal_subscriber") {  
    subscription_ = this->create_subscription<zbar_ros_interfaces::msg::Symbol>("barcode", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    
    scanSub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 1, std::bind(&MinimalSubscriber::callbackScan, this, std::placeholders::_1));
    
    
    publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("topic", 10);
    
    xpos = 0;
    publishPoint = 0;
    }

private:
  int xpos;
  int publishPoint;
  
  void callbackScan(const sensor_msgs::msg::LaserScan::SharedPtr scan) {

      if (publishPoint == 1) {
          RCLCPP_INFO(this->get_logger(), "min data: '%d' ", xpos);
          RCLCPP_INFO(this->get_logger(), "laser data: '%f' ", scan->ranges[0]);
          // command this line out after curve fitting 
          for (int i = 0; i <= 360; i++) cout << i << ": " << scan->ranges[i] << " \n";
          
          // xpos to range_idx curve fitting result
          int range_idx = round(-0.07636 * xpos + 24.98);
          if (range_idx < 0) range_idx += 360;
          RCLCPP_INFO(this->get_logger(), "laser angle: '%d' ", range_idx);
          float distance = scan->ranges[range_idx];
          RCLCPP_INFO(this->get_logger(), "laser distance: '%f' ", distance);
          

          const float THRESHOLD = 1.8
          cout << "test1" << endl;
          if (distance<THRESHOLD){  
              cout << "test2" << endl;
              float radian_angle = 3.14159/180 * range_idx //range_idx is degree
              geometry_msgs::msg::PointStamped pointStamped;
              pointStamped.header.frame_id = "base_link";
              
              pointStamped.point.x = distance * cos(radian_angle);
              pointStamped.point.x = distance * sin(radian_angle);
              pointStamped.point.z = 0.0;
              
              publishPoint = 0;
              publisher_->publish(pointStamped);
              RCLCPP_INFO(this->get_logger(), "point published: '%f', '%f', '%f' ", pointStamped.point.x, pointStamped.point.y, pointStamped.point.z);
          } else{
               RCLCPP_INFO(this->get_logger(), "still too far away. dropped");
          }
      }
  }
  
  void topic_callback(const zbar_ros_interfaces::msg::Symbol::SharedPtr msg)  {

     int valueOne = msg->points[0].x ;
     int ValueTwo = msg->points[1].x;
        
     xpos = (valueOne+ValueTwo)/2;
    //for (int i = 0; i < 10; i ++ ) {
    //	RCLCPP_INFO(this->get_logger(), "i: '%d' string: '%s'", i,markers[i].c_str());
    //}
    
    
    for ( int i = 0; i < 10; i ++) {
    	//If marker is already in list we don't need to publish again
        if (msg->data.compare(markers[i]) == 0) { break; }
    	
    	//If reached end of list add marker then find and broadcast point
    	if (markers[i].compare("") == 0) {
    		markers[i] = msg->data.c_str();
    		publishPoint = 1;
    		break;
    	}
    }
    
  }
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scanSub_;
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
