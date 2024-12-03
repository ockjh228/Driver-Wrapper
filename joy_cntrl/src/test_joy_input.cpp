#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <cstring>
#include <cmath>

class JoystickToTwist
{
public:
    JoystickToTwist(std::shared_ptr<rclcpp::Node> node)
    : node_(node), joy_fd_(-1)
    {
        // QoS Profile
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

        // Create publisher for Twist messages
        twist_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>("joy_input", 10);

        // Open joystick device
        joy_fd_ = open("/dev/input/js0", O_RDONLY | O_NONBLOCK); // Open in non-blocking mode
        if (joy_fd_ < 0) {
            RCLCPP_ERROR(node_->get_logger(), "Error opening joystick device: %s", strerror(errno));
            return;
        }

        RCLCPP_INFO(node_->get_logger(), "Joystick device opened successfully.");

        // Set up a timer to read joystick data periodically
        timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(10), // Reduce the timer period to 10ms
            std::bind(&JoystickToTwist::timer_callback, this));
    }

    ~JoystickToTwist()
    {
        if (joy_fd_ >= 0) {
            close(joy_fd_);
        }
    }

private:
    void timer_callback()
    {
        if (joy_fd_ < 0) {
            return;
        }

        struct js_event js;
        while (read(joy_fd_, &js, sizeof(js)) > 0) {
            auto twist_msg = geometry_msgs::msg::Twist();
            bool data_available = false;

            // Initialize axes and buttons arrays
            std::vector<double> axes(8, 0.0); // 8 axes
            std::vector<int> buttons(11, 0); // 11 buttons

            if (js.type == JS_EVENT_AXIS) {
                // Normalize axis value
                
                //pseudo code
                // axes[2] = 0; // if not pushed
                // if (buttons[1] == on) nothing
                // else axes[2] = -axes[2];

                axes[js.number] = js.value / 32767.0;
                RCLCPP_INFO(node_->get_logger(), "Axis %d value: %f", js.number, axes[js.number]);
                data_available = true;
            } 
            else if (js.type == JS_EVENT_BUTTON) {
                buttons[js.number] = js.value;
                RCLCPP_INFO(node_->get_logger(), "Button %d value: %d", js.number, buttons[js.number]);
                data_available = true;
            }


            if (data_available) {
                // Apply button logic to axes[2] and axes[5]
                if (buttons[0] == 1) {
                    axes[2] = std::abs(axes[2]); // Always positive when button[0] is on
                } else {
                    axes[2] = -std::abs(axes[2]); // Always negative when button[0] is off
                }

                if (buttons[1] == 1) {
                    axes[5] = std::abs(axes[5]); // Always positive when button[1] is on
                } else {
                    axes[5] = -std::abs(axes[5]); // Always negative when button[1] is off
                }

                // Map axes to Twist message
                if (axes.size() >= 6) {
                    twist_msg.linear.x = axes[0]; // x
                    twist_msg.linear.y = axes[1]; // y
                    twist_msg.linear.z = axes[2]; // z

                    // Angular values directly mapped from joystick axes
                    twist_msg.angular.x = axes[3]; // roll
                    twist_msg.angular.y = axes[4]; // pitch
                    twist_msg.angular.z = axes[5]; // yaw
                } 


            // // If joystick data is available
            // if (data_available) {
            //     // Map axes to Twist message
            //     if (axes.size() >= 6) {
            //         twist_msg.linear.x = axes[0]; // x
            //         twist_msg.linear.y = axes[1]; // y
            //         twist_msg.linear.z = axes[2]; // z

            //         // Angular values directly mapped from joystick axes
            //         twist_msg.angular.x = axes[3]; // roll
            //         twist_msg.angular.y = axes[4]; // pitch
            //         twist_msg.angular.z = axes[5]; // yaw
            //     } 
                else {
                    RCLCPP_WARN(node_->get_logger(), "Not enough axes data available from joystick.");
                }

                RCLCPP_INFO(node_->get_logger(), "Publishing Twist message: linear[%f, %f, %f], angular[%f, %f, %f]",
                            twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z,
                            twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z);

                // Publish Twist message
                twist_publisher_->publish(twist_msg);
            }
        }

        if (errno != EAGAIN) {
            RCLCPP_ERROR(node_->get_logger(), "Error reading joystick data: %s", strerror(errno));
        }
    }

    std::shared_ptr<rclcpp::Node> node_;
    int joy_fd_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("joystick_to_twist");
    auto joystick_to_twist = std::make_shared<JoystickToTwist>(node);

    RCLCPP_INFO(node->get_logger(), "Node started.");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
