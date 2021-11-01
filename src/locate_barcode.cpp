#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "zbar_ros_interfaces/msg/symbol.hpp"
#include "cv_bridge/cv_bridge.h"
#include <sensor_msgs/msg/laser_scan.hpp>
#include "qr_custom_message/msg/qr_point_stamped.hpp"

#include <string>
#include<cmath>

#define PI 3.14159265
#define THRESHOLD 1.8

using std::placeholders::_1;
using namespace std;



std::string markers[10] = {"","","","","","","","","",""};



class MinimalSubscriber : public rclcpp::Node {
	

public:
  
  MinimalSubscriber() : Node("minimal_subscriber") {  
    subscription_ = this->create_subscription<zbar_ros_interfaces::msg::Symbol>("barcode", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    
    scanSub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 1, std::bind(&MinimalSubscriber::callbackScan, this, std::placeholders::_1));
    
    
    publisher_ = this->create_publisher<qr_custom_message::msg::QrPointStamped>("topic", 10);
    
    xpos = 0;
    publishPoint = 0;
    id = "";
    }

private:
  int xpos;
  int publishPoint;
  std::string id;
  void callbackScan(const sensor_msgs::msg::LaserScan::SharedPtr scan) {

      if (publishPoint == 1) {
          RCLCPP_INFO(this->get_logger(), "min data: '%d' ", xpos);
          RCLCPP_INFO(this->get_logger(), "laser data: '%f' ", scan->ranges[0]);
          RCLCPP_INFO(this->get_logger(), "laser data: '%s' ", id.c_str());
          // command this line out after curve fitting				
          for (int i = 0; i <= 360; i++) cout << i << ": " << scan->ranges[i] << " \n ";
          
           // xpos to range_idx curve fitting result
          int range_idx = round(-0.093942 * xpos + 29.32);
          if (range_idx < 0) range_idx += 360;
          RCLCPP_INFO(this->get_logger(), "laser angle: '%d' ", range_idx);
          float distance = scan->ranges[range_idx];
          RCLCPP_INFO(this->get_logger(), "laser distance: '%f' ", distance);
          
          if (distance<THRESHOLD){  
              float radian_angle = PI/180 * range_idx; //range_idx is degree
              

              qr_custom_message::msg::QrPointStamped qrPointStamped;
              qrPointStamped.header.frame_id = "base_scan";
              qrPointStamped.data = id;
              
              qrPointStamped.point.x = distance * cos(radian_angle);
              qrPointStamped.point.y = distance * sin(radian_angle);
              qrPointStamped.point.z = 0.1;
              
              publishPoint = 0;
              publisher_->publish(qrPointStamped);
             
              RCLCPP_INFO(this->get_logger(), "point published: '%f', '%f', '%f' ", qrPointStamped.point.x, qrPointStamped.point.y, qrPointStamped.point.z);
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
    		id = msg->data.c_str();
    		publishPoint = 1;
    		break;
    	}
    }
    
  }
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scanSub_;
  rclcpp::Subscription<zbar_ros_interfaces::msg::Symbol>::SharedPtr subscription_;
  rclcpp::Publisher<qr_custom_message::msg::QrPointStamped>::SharedPtr publisher_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}

