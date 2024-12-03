// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/joy.hpp>
// #include <geometry_msgs/msg/twist.hpp>
// #include <std_msgs/msg/empty.hpp>
// #include <std_msgs/msg/string.hpp>
// #include <control_msgs/msg/gripper_command.hpp>
// #include <fcntl.h>
// #include <unistd.h>
// #include <linux/joystick.h>
// #include <iostream>
// #include <cstring>
// #include <cmath>

// class JoystickToTwist
// {
// public:
//     JoystickToTwist(std::shared_ptr<rclcpp::Node> node)
//     : node_(node), joy_fd_(-1), cnt_2_(0), cnt_5_(0), gripper_close_active_(false), gripper_open_active_(false),
//     emergency_stop_active_(false), fault_clear_active_(false), twist_zero_published_(false), scale_factor_(1)
//     {
//         // Create publisher to Control
//         twist_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>("twist_remote_cmd", 10);

//         error_clear_publisher_ = node_->create_publisher<std_msgs::msg::Empty>("clear_faults_cmd", 10);

//         emergency_stop_publisher_ = node_->create_publisher<std_msgs::msg::Empty>("emergency_stop_cmd", 10);

//         gripper_command_publisher_ = node_->create_publisher<control_msgs::msg::GripperCommand>("gripper_command_cmd",10);

//         action_publisher_ = node_->create_publisher<std_msgs::msg::String>("action", 10);


//         // Open joystick device
//         joy_fd_ = open("/dev/input/js0", O_RDONLY | O_NONBLOCK); // Open in non-blocking mode
//         if (joy_fd_ < 0) {
//             RCLCPP_ERROR(node_->get_logger(), "Error opening joystick device: %s", strerror(errno));
//             rclcpp::shutdown(); // Kill the node when there is no Joystick connected.
//             return;
//         }

//         RCLCPP_INFO(node_->get_logger(), "Joystick device opened successfully.");

//         // Set up a timer to read joystick data periodically
//         timer_ = node_->create_wall_timer(
//             std::chrono::milliseconds(10), // Reduce the timer period to 10ms from 100ms
//             std::bind(&JoystickToTwist::timer_callback, this));
//     }

//     ~JoystickToTwist()
//     {
//         if (joy_fd_ >= 0) {
//             close(joy_fd_);
//         }
//     }

// private:
//     void timer_callback()
//     {
//         if (joy_fd_ < 0) {
//             return;
//         }

//             struct js_event js;

//             // Initialize static axes and buttons arrays
//             static std::vector<double> axes(8, 0.0); // 8 axes
//             static std::vector<int> buttons(11, 0); // 11 buttons
//             static auto twist_msg = geometry_msgs::msg::Twist(); // Maintain the last non-zero state
//             static bool twist_zero_published = false;

//             try
//             {
//                 // Read joystick events
//                 // Read stick movement or button pressed
//                 while (read(joy_fd_, &js, sizeof(js)) > 0) {

//                     twist_zero_published_ = false;                
                    
//                     //When Stick moved
//                     if (js.type == JS_EVENT_AXIS) {
//                         // Apply deadzone
//                         if (js.value >= -1500 && js.value <= 1500) {
//                             js.value = 0;
//                         }

//                         // Normalize axis value
//                         double normalized_value;
//                         if (js.number == 2 || js.number == 5) {
//                             normalized_value = (js.value + 32767.0) / (32767.0 * 2);
//                         } else {
//                             normalized_value = js.value / 32767.0;
//                         }
                        
//                         //Handle scale change
//                         if(js.number == 7){
//                             if(normalized_value == -1.0 && scale_factor_ < 4){
//                                 scale_factor_++;
//                             }
//                             else if(normalized_value == 1.0 && scale_factor_ > 1){
//                                 scale_factor_--;
//                             }
//                         }

//                         //Initiallizing
//                         double twist_value = 0;

//                         if(normalized_value > 0.5){
//                             twist_value = 0.1f;
//                         }
//                         else if(normalized_value < -0.5){
//                             twist_value = -0.1f;
//                         }

//                         axes[js.number] = twist_value;

//                         // Update the corresponding twist message values
//                         if (js.number == 0) {
//                             twist_msg.linear.x =  -1 * axes[js.number] * std::abs(scale_factor_);
//                         } 
//                         else if (js.number == 1) {
//                             twist_msg.linear.z =  -1 * axes[js.number] * std::abs(scale_factor_); 
//                         } 
//                         else if (js.number == 2) {
//                             twist_msg.linear.y = (cnt_2_ % 2 == 0) ? axes[js.number] * std::abs(scale_factor_) : -axes[js.number] * std::abs(scale_factor_);
//                         } 
//                         else if (js.number == 3) {
//                             twist_msg.angular.y = axes[js.number] * std::abs(scale_factor_) * 40; 
//                         } 
//                         else if (js.number == 4) {
//                             twist_msg.angular.x = axes[js.number] * std::abs(scale_factor_) * 40; 
//                         } 
//                         else if (js.number == 5) {
//                             twist_msg.angular.z = (cnt_5_ % 2 == 0) ? axes[js.number]* 60 * std::abs(scale_factor_) : -axes[js.number] * 60 * std::abs(scale_factor_);
//                         }
//                     }

//                     //When Button Pressed 
//                     else if (js.type == JS_EVENT_BUTTON) {
//                         buttons[js.number] = js.value;
//                         RCLCPP_INFO(node_->get_logger(), "Button %d value: %d", js.number, buttons[js.number]);

//                         // Increment counters when buttons are pressed
//                         // Buttons for Z-axis & yaw cntrl
//                         if (js.number == 0 && js.value == 1) {
//                             cnt_2_++;
//                         } 
//                         else if (js.number == 1 && js.value == 1) {
//                             cnt_5_++;
//                         }

//                         //Buttons for other Options
//                         if(js.number == 2){
//                             if(js.value == 1){
//                                 gripper_close_active_ = true;
//                                 std::cout<<"Gripper Close Activated!"<<std::endl;
//                             }
//                             else{
//                                 gripper_close_active_ = false;
//                                 std::cout<<"Gripper Close Deactivated!"<<std::endl;
//                             }
//                         }

//                         if(js.number == 3){
//                             if(js.value == 1){
//                                 gripper_open_active_ = true;
//                                 std::cout<<"Gripper Open Activated!"<<std::endl;
//                             }
//                             else{
//                                 gripper_open_active_ = false;
//                                 std::cout<<"Gripper Open Deactivated!"<<std::endl;
//                             }
//                         }
//                         //Publish error clearing signal when button 4 is pressed
//                         if(js.number == 4 && js.value == 1){
//                             std_msgs::msg::Empty msg;
//                             error_clear_publisher_ -> publish(msg);
//                             std::cout<<"Error clearing siganl published"<<std::endl;
//                             fault_clear_active_ = true;

//                             std::this_thread::sleep_for(std::chrono::milliseconds(500));  // 500ms 대기
//                             fault_clear_active_ = false;
//                             emergency_stop_active_ = false;
//                             std::cout<<"Fault clear deactivated"<<std::endl;
//                         }
//                         //Publsih e-stop signal when button 5 is pressed
//                         else if(js.number == 5 && js.value == 1){
//                             std_msgs::msg::Empty msg;
//                             emergency_stop_publisher_ -> publish(msg);
//                             std::cout<<"Emergency stop signal published"<<std::endl;
//                             emergency_stop_active_ = true;
//                         }

//                         //Publish HomePosition Action
//                         if (js.number == 8 && js.value == 1) {
//                             std_msgs::msg::String action_msg;
//                             action_msg.data = "Home";
//                             action_publisher_->publish(action_msg);
//                             std::cout << "Published action: Home" << std::endl;
//                         }

//                         if (js.number == 7 && js.value == 1) {
//                             std_msgs::msg::String action_msg;
//                             action_msg.data = "Retract";
//                             action_publisher_->publish(action_msg);
//                             std::cout << "Published action: Retract" << std::endl;
//                         }
//                     }
//                 } // End of while (Finish reading Joystick event)

//                 //Publishing Action
//                 if(gripper_close_active_){
//                     publish_gripper_command(1.0, 10.0);
//                     std::cout<<"Publishing Gripper Close Signal"<<std::endl;
//                 }

//                 if(gripper_open_active_){
//                     publish_gripper_command(0.0, 10.0);
//                     std::cout<<"Publishing Gripper Open Signal"<<std::endl;
//                 }

//                 // Publish the twist message only if there was joystick activity or a non-zero value is present
//                 if (!emergency_stop_active_ && !fault_clear_active_ &&
//                     (twist_msg.linear.x != 0.0 || twist_msg.linear.y != 0.0 || 
//                     twist_msg.linear.z != 0.0 || twist_msg.angular.x != 0.0 || 
//                     twist_msg.angular.y != 0.0 || twist_msg.angular.z != 0.0))  
//                 {
//                     RCLCPP_INFO(node_->get_logger(), "Publishing Twist message: linear[%f, %f, %f], angular[%f, %f, %f]",
//                                 twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z,
//                                 twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z);
//                     twist_publisher_->publish(twist_msg);            
//                 }

//                 if (twist_msg.linear.x == 0.0 && twist_msg.linear.y == 0.0 && 
//                 twist_msg.linear.z == 0.0 && twist_msg.angular.x == 0.0 && 
//                 twist_msg.angular.y == 0.0 && twist_msg.angular.z == 0.0) 
//                 {
//                     if (!twist_zero_published_){
//                         twist_publisher_->publish(twist_msg);
//                         RCLCPP_INFO(node_->get_logger(), "Publishing Twist message with all zero values.");
//                         twist_zero_published_ = true; 
//                     }
//                     twist_msg = geometry_msgs::msg::Twist();
//                 }
//             }
//             catch(const std::exception& e)
//             {
//                 std::cerr << e.what() << '\n';
//             }            
//         // if (errno != EAGAIN) {
//         //     RCLCPP_ERROR(node_->get_logger(), "Error reading joystick data: %s", strerror(errno));
//         // }
//     }   

//     void publish_gripper_command(float position, float max_effort){
//         control_msgs::msg::GripperCommand msg;
//         msg.position = position;
//         msg.max_effort = max_effort;
//         gripper_command_publisher_ -> publish(msg);
//     }

//     std::shared_ptr<rclcpp::Node> node_;

//     rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
//     rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr error_clear_publisher_;
//     rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr emergency_stop_publisher_;
//     rclcpp::Publisher<control_msgs::msg::GripperCommand>::SharedPtr gripper_command_publisher_;
//     rclcpp::Publisher<std_msgs::msg::String>::SharedPtr action_publisher_;
//     rclcpp::TimerBase::SharedPtr timer_;

//     int joy_fd_;
//     int cnt_2_;
//     int cnt_5_;
//     int scale_factor_;
//     bool gripper_close_active_;  
//     bool gripper_open_active_;       
//     bool emergency_stop_active_;  
//     bool fault_clear_active_;    
//     bool twist_zero_published_;
// };

// int main(int argc, char * argv[])
// {
//     rclcpp::init(argc, argv);

//     auto node = rclcpp::Node::make_shared("joystick_to_twist");
//     auto joystick_to_twist = std::make_shared<JoystickToTwist>(node);

//     RCLCPP_INFO(node->get_logger(), "Node started.");

//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }


/****************/
/****************/
/****************/
/****************/

#include <rclcpp/rclcpp.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <iostream>
#include <cstring>
#include <cmath>

// Custom message headers
#include "add_sugv_msgs/msg/twist_cmd.hpp"
#include "add_sugv_msgs/msg/clear_fault_cmd.hpp"
#include "add_sugv_msgs/msg/emergency_stop_cmd.hpp"
#include "add_sugv_msgs/msg/gripper_cmd.hpp"
#include "add_sugv_msgs/msg/preset_cmd.hpp"
#include <std_msgs/msg/string.hpp>

class JoystickToTwist
{
public:
    JoystickToTwist(std::shared_ptr<rclcpp::Node> node)
    : node_(node), joy_fd_(-1), cnt_2_(0), cnt_5_(0), gripper_close_active_(false), gripper_open_active_(false),
    emergency_stop_active_(false), fault_clear_active_(false), twist_zero_published_(false), scale_factor_(1)
    {
        // Create publishers with the correct message types
        twist_publisher_ = node_->create_publisher<add_sugv_msgs::msg::TwistCmd>("command/twist", 10);
        error_clear_publisher_ = node_->create_publisher<add_sugv_msgs::msg::ClearFaultCmd>("command/clear", 10);
        emergency_stop_publisher_ = node_->create_publisher<add_sugv_msgs::msg::EmergencyStopCmd>("command/e_stop", 10);
        gripper_command_publisher_ = node_->create_publisher<add_sugv_msgs::msg::GripperCmd>("command/gripper", 10);
        action_publisher_ = node_->create_publisher<add_sugv_msgs::msg::PresetCmd>("command/preset", 10);

        // Open joystick device
        joy_fd_ = open("/dev/input/js0", O_RDONLY | O_NONBLOCK); // Open in non-blocking mode
        if (joy_fd_ < 0) {
            RCLCPP_ERROR(node_->get_logger(), "Error opening joystick device: %s", strerror(errno));
            rclcpp::shutdown(); // Shutdown if joystick not connected
            return;
        }

        RCLCPP_INFO(node_->get_logger(), "Joystick device opened successfully.");

        // Set up a timer to read joystick data periodically
        timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(10), // Timer period 10ms
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

        // Initialize static axes and buttons arrays
        static std::vector<double> axes(8, 0.0); // 8 axes
        static std::vector<int> buttons(11, 0); // 11 buttons
        static auto twist_cmd_msg = add_sugv_msgs::msg::TwistCmd(); // Use custom TwistCmd message
        static bool twist_zero_published = false;

        try
        {
            // Read joystick events
            while (read(joy_fd_, &js, sizeof(js)) > 0) {
                twist_zero_published_ = false;

                // When Stick moved
                if (js.type == JS_EVENT_AXIS) {
                    // Apply deadzone
                    if (js.value >= -1500 && js.value <= 1500) {
                        js.value = 0;
                    }

                    // Normalize axis value
                    double normalized_value;
                    if (js.number == 2 || js.number == 5) {
                        normalized_value = (js.value + 32767.0) / (32767.0 * 2);
                    } else {
                        normalized_value = js.value / 32767.0;
                    }

                    // Handle scale change
                    if (js.number == 7) {
                        if (normalized_value == -1.0 && scale_factor_ < 4) {
                            scale_factor_++;
                        } else if (normalized_value == 1.0 && scale_factor_ > 1) {
                            scale_factor_--;
                        }
                    }

                    // Initializing
                    double twist_value = 0;

                    if (normalized_value > 0.5) {
                        twist_value = 0.1f;
                    } else if (normalized_value < -0.5) {
                        twist_value = -0.1f;
                    }

                    axes[js.number] = twist_value;

                    // Update the corresponding twist message values
                    if (js.number == 0) {
                        twist_cmd_msg.linear.x = -1 * axes[js.number] * std::abs(scale_factor_);
                    }
                    else if (js.number == 1) {
                        twist_cmd_msg.linear.z = -1 * axes[js.number] * std::abs(scale_factor_);
                    }
                    else if (js.number == 2) {
                        twist_cmd_msg.linear.y = (cnt_2_ % 2 == 0) ? axes[js.number] * std::abs(scale_factor_) : -axes[js.number] * std::abs(scale_factor_);
                    }
                    else if (js.number == 3) {
                        twist_cmd_msg.angular.y = axes[js.number] * std::abs(scale_factor_) * 40;
                    }
                    else if (js.number == 4) {
                        twist_cmd_msg.angular.x = axes[js.number] * std::abs(scale_factor_) * 40;
                    }
                    else if (js.number == 5) {
                        twist_cmd_msg.angular.z = (cnt_5_ % 2 == 0) ? axes[js.number] * 60 * std::abs(scale_factor_) : -axes[js.number] * 60 * std::abs(scale_factor_);
                    }
                }

                // When Button Pressed
                else if (js.type == JS_EVENT_BUTTON) {
                    buttons[js.number] = js.value;

                    // Increment counters when buttons are pressed
                    if (js.number == 0 && js.value == 1) {
                        cnt_2_++;
                    }
                    else if (js.number == 1 && js.value == 1) {
                        cnt_5_++;
                    }

                    // Gripper control logic
                    if (js.number == 2) {
                        if (js.value == 1) {
                            gripper_close_active_ = true;
                        } else {
                            gripper_close_active_ = false;
                        }
                    }

                    if (js.number == 3) {
                        if (js.value == 1) {
                            gripper_open_active_ = true;
                        } else {
                            gripper_open_active_ = false;
                        }
                    }

                    // Publish error clearing signal when button 4 is pressed
                    if (js.number == 4 && js.value == 1) {
                        add_sugv_msgs::msg::ClearFaultCmd msg;
                        // Assuming ClearFaultCmd has a field 'clear_faults' of type std_msgs::msg::Bool
                        msg.clear_faults = true;
                        error_clear_publisher_->publish(msg);
                    }

                    // Publish e-stop signal when button 5 is pressed
                    else if (js.number == 5 && js.value == 1) {
                        add_sugv_msgs::msg::EmergencyStopCmd msg;
                        // Assuming EmergencyStopCmd has a field 'emergency_stop' of type std_msgs::msg::Bool
                        msg.emergency_stop = true;
                        emergency_stop_publisher_->publish(msg);
                    }

                    // Publish action signals (home and retract)
                    if (js.number == 8 && js.value == 1) {
                        add_sugv_msgs::msg::PresetCmd action_msg;
                        action_msg.preset_name = "Home";
                        action_publisher_->publish(action_msg);
                    }

                    if (js.number == 7 && js.value == 1) {
                        add_sugv_msgs::msg::PresetCmd action_msg;
                        action_msg.preset_name = "Retract";
                        action_publisher_->publish(action_msg);
                    }
                }
            }

            // Publish Gripper Actions
            if (gripper_close_active_) {
                publish_gripper_command(1.0, 10.0);
            }
            if (gripper_open_active_) {
                publish_gripper_command(0.0, 10.0);
            }

            // Publish twist message only if there's activity
            if (!emergency_stop_active_ && !fault_clear_active_ &&
                (twist_cmd_msg.linear.x != 0.0 || twist_cmd_msg.linear.y != 0.0 ||
                 twist_cmd_msg.linear.z != 0.0 || twist_cmd_msg.angular.x != 0.0 ||
                 twist_cmd_msg.angular.y != 0.0 || twist_cmd_msg.angular.z != 0.0)) {
                twist_publisher_->publish(twist_cmd_msg);
            }

            // If all values are zero, publish zero twist message
            if (twist_cmd_msg.linear.x == 0.0 && twist_cmd_msg.linear.y == 0.0 &&
                twist_cmd_msg.linear.z == 0.0 && twist_cmd_msg.angular.x == 0.0 &&
                twist_cmd_msg.angular.y == 0.0 && twist_cmd_msg.angular.z == 0.0) {
                if (!twist_zero_published_) {
                    twist_publisher_->publish(twist_cmd_msg);
                    twist_zero_published_ = true;
                }
                twist_cmd_msg = add_sugv_msgs::msg::TwistCmd();
            }
        } catch (const std::exception& e) {
            std::cerr << e.what() << '\n';
        }
    }

    void publish_gripper_command(float position, float max_effort) {
        add_sugv_msgs::msg::GripperCmd msg;
        msg.position = position;
        msg.max_effort = max_effort;
        gripper_command_publisher_->publish(msg);
    }

    std::shared_ptr<rclcpp::Node> node_;

    rclcpp::Publisher<add_sugv_msgs::msg::TwistCmd>::SharedPtr twist_publisher_;
    rclcpp::Publisher<add_sugv_msgs::msg::ClearFaultCmd>::SharedPtr error_clear_publisher_;
    rclcpp::Publisher<add_sugv_msgs::msg::EmergencyStopCmd>::SharedPtr emergency_stop_publisher_;
    rclcpp::Publisher<add_sugv_msgs::msg::GripperCmd>::SharedPtr gripper_command_publisher_;
    rclcpp::Publisher<add_sugv_msgs::msg::PresetCmd>::SharedPtr action_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    int joy_fd_;
    int cnt_2_;
    int cnt_5_;
    int scale_factor_;
    bool gripper_close_active_;
    bool gripper_open_active_;
    bool emergency_stop_active_;
    bool fault_clear_active_;
    bool twist_zero_published_;
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
