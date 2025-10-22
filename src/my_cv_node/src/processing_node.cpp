#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

// Will receive data from a node (like a camera driver) that publishes (broadcasts) data onto the /camera/image_raw topic
// This code then publishes its own data onto the /camera/image_processed topic
class OpenCVNode : public rclcpp::Node
{
public:
  OpenCVNode() : Node("opencv_node")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10, std::bind(&OpenCVNode::image_callback, this,
      std::placeholders::_1));
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
  {
    try
    {
      cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
      cv::imshow("Camera", frame);
      cv::waitKey(10);
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OpenCVNode>());
  rclcpp::shutdown();
  return 0;
}