

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "drivetrain_interfaces/msg/joystick.hpp"

//joystick imports
#include <iostream>
#include <unistd.h>
#include <joystick.h>


//min and max value of inputs
#define MAX_VALUE 32767
#define MIN_VALUE -32767

//constant for each trigger
#define LEFT_JOYSTICK_HORIZONTAL 0
#define LEFT_JOYSTICK_VERTICAL 1
#define LEFT_TRIGGER 2
#define RIGHT_JOYSTICK_HORIZONTAL 3
#define RIGHT_JOYSTICK_VERTICAL 4
#define RIGHT_TRIGGER 5
#define DPAD_HORIZONTAL 6
#define DPAD_VERTICAL 7

using namespace std::chrono_literals;


/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<drivetrain_interfaces::msg::Joystick>("drivetrain_joystick_topic", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    float left_joystick_y_value;
    float right_joystick_x_value;

      if (event.isAxis())
      {
          int axisInput = (int)event.number;
          int axisValue = (int)event.value;

          if(axisInput == LEFT_JOYSTICK_VERTICAL)
          {
            left_joystick_y_value = (float)value/MAX_VALUE;
            right_joystick_x_value = 0.0f;
          }else if(axisInput == RIGHT_JOYSTICK_HORIZONTAL)
          {
            left_joystick_y_value =0.0f;
            right_joystick_x_value = (float)value/MAX_VALUE;
          }
        }
      }
      auto message = drivetrain_interfaces::msg::Joystick();
      message.left_joystick_y_axis = left_joystick_y_value;
      message.right_joystick_x_axis = right_joystick_x_value;
      publisher_->publish(message);
      rclcpp::TimerBase::SharedPtr timer_;
      rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
      size_t count_;
  };

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}

