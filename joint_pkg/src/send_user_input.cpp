// // // #include <moveit/robot_model/robot_model.h>
// // // #include <moveit/robot_model_loader/robot_model_loader.h>
// // // #include <moveit/robot_state/robot_state.h>
// // // #include <geometry_msgs/msg/pose.hpp>
// // #include "rclcpp/rclcpp.hpp"
// // #include "std_msgs/msg/string.hpp"
// // // #include "trajectory_msgs/msg/joint_trajectory.hpp"
// // // #include "trajectory_msgs/msg/joint_trajectory_point.hpp"
// // #include <chrono>
// // #include <functional>
// // #include <memory>
// // #include <random>
// // #include <string>
// // #include <map>
// // #include <keyboard_msgs/msg/key.hpp>
// // #include <SDL.h>


// // using namespace std::chrono_literals;

// // class UserInput
// // {
// // public:
// //   UserInput(const rclcpp::Node::SharedPtr& node)
// //   : key_code_(0), key_pressed_(false)
// //   {
// //     node_ = node;

// //     rclcpp::QoS qos_profile(10);
// //     qos_profile.reliable(); 
// //     qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL); 

// //     pub_down_ = node_->create_publisher<keyboard_msgs::msg::Key>("Keydown", qos_profile);
// //     pub_up_ = node_->create_publisher<keyboard_msgs::msg::Key>("Keyup_event",qos_profile);
// //     timer_ = node_->create_wall_timer(20ms, std::bind(&UserInput::publish_input, this));

// //     // std::cout<<"1"<<std::endl;

// //     node_->declare_parameter("allow_repeat", false);
// //     bool allow_repeat = node_->get_parameter("allow_repeat").as_bool();

// //     node_->declare_parameter("repeat_delay", 500);
// //     int repeat_delay = node_->get_parameter("repeat_delay").as_int();

// //     node_->declare_parameter("repeat_interval", 30);  
// //     int repeat_interval = node_->get_parameter("repeat_interval").as_int();
// //     // std::cout<<"2"<<std::endl;
// //     if (!allow_repeat) repeat_delay = 0; // disable
// //     // std::cout<<"3"<<std::endl;


// //     if (SDL_Init(SDL_INIT_VIDEO) < 0){
// //       std::cout<<SDL_INIT_VIDEO<<std::endl;
// //       std::cout<<SDL_Init(SDL_INIT_VIDEO)<<std::endl;
// //       std::cerr<<SDL_GetError() <<std::endl;
// //       throw std::runtime_error("Could not init SDL");  // trouble issue starts from here.
// //     } 

// //     window_ = SDL_CreateWindow("ROS keyboard input", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 100, 100, SDL_WINDOW_SHOWN);
// //     if (!window_) throw std::runtime_error("Could not create SDL window");
// //     renderer_ = SDL_CreateRenderer(window_, -1, SDL_RENDERER_ACCELERATED);
// //     if (!renderer_) throw std::runtime_error("Could not create SDL renderer");
// //   }

// //   ~UserInput()
// //   {
// //     SDL_DestroyRenderer(renderer_);
// //     SDL_DestroyWindow(window_);
// //     SDL_Quit();
// //   }

// // private:
// //   void publish_input()
// //   {  
// //     SDL_Event event;
// //     while (SDL_PollEvent(&event)) {
// //       if (event.type == SDL_KEYDOWN) {
// //         handle_key_event(event.key, true);  // Key down event
// //       }
// //       else if (event.type == SDL_KEYUP) {
// //         handle_key_event(event.key, false); // Key up event
// //       }
// //       else if (event.type == SDL_QUIT) {
// //         SDL_DestroyRenderer(renderer_);
// //         SDL_DestroyWindow(window_);
// //         SDL_Quit();
// //         timer_->cancel();
// //         return;
// //       }      
// //     }
// //   }

// //   void handle_key_event(const SDL_KeyboardEvent& key_event, bool is_key_down) 
// //   {
// //     static std::map<SDL_Keycode, bool> key_state; // stores key_state

// //     if (is_key_down) {
// //       // Only publish if the key is not already considered "pressed"
// //       if (key_state.find(key_event.keysym.sym) == key_state.end() || !key_state[key_event.keysym.sym]) {
// //         keyboard_msgs::msg::Key k;
// //         k.code = key_event.keysym.sym;
// //         k.modifiers = key_event.keysym.mod;

// //         RCLCPP_INFO(node_->get_logger(), "Publishing key down: code=%d", k.code);
// //         pub_down_->publish(k);

// //         key_state[key_event.keysym.sym] = true; // Mark the key as pressed
// //         SDL_SetRenderDrawColor(renderer_, k.code % 256, 0, 255, SDL_ALPHA_OPAQUE);
// //         SDL_RenderClear(renderer_);
// //         SDL_RenderPresent(renderer_);
// //       }
// //     } 
// //     else {
// //       if (key_state.find(key_event.keysym.sym) != key_state.end() && key_state[key_event.keysym.sym]) {
// //         keyboard_msgs::msg::Key k;
// //         k.code = key_event.keysym.sym;
// //         k.modifiers = key_event.keysym.mod;

// //         RCLCPP_INFO(node_->get_logger(), "Publishing key up: code=%d", k.code);
// //         pub_up_->publish(k);

// //         key_state[key_event.keysym.sym] = false; // Update to false if key states changed to key_up
// //       }
// //       // // Key is released, update state
// //       // keyboard_msgs::msg::Key k;
// //       // k.code = key_event.keysym.sym;
// //       // k.modifiers = key_event.keysym.sym;

// //       // RCLCPP_INFO(node_->get_logger(), "Publishing key up: code= %d", k.code);
// //       // key_state[key_event.keysym.sym] = false; // update to false if key states changed to key_up
// //     }
// //   }
// //     // keyboard_msgs::msg::Key k;
// //     // bool new_event = false;

// //     // SDL_Event event;
// //     // if(SDL_PollEvent(&event)){
// //     //   if(event.type == SDL_KEYDOWN){
// //     //     k.code = event.key.keysym.sym;
// //     //     k.modifiers = event.key.keysym.mod;
// //     //     new_event = true;

// //     //     SDL_SetRenderDrawColor(renderer_, k.code % 256, 0, 255, SDL_ALPHA_OPAQUE);
// //     //     SDL_RenderClear(renderer_);
// //     //     SDL_RenderPresent(renderer_);
// //     //   }
// //     //   else if(event.type == SDL_QUIT){
// //     //     SDL_DestroyRenderer(renderer_);
// //     //     SDL_DestroyWindow(window_);
// //     //     SDL_Quit();
// //     //     timer_->cancel();
// //     //   }      
// //     // }

// //     // if(new_event){
// //     //   RCLCPP_INFO(node_->get_logger(), "Publishing key down: code=%d", k.code);
// //     //   pub_down_->publish(k);
// //     // }
// //   rclcpp::Node::SharedPtr node_;
// //   rclcpp::TimerBase::SharedPtr timer_;
// //   rclcpp::Publisher<keyboard_msgs::msg::Key>::SharedPtr pub_down_;
// //   rclcpp::Publisher<keyboard_msgs::msg::Key>::SharedPtr pub_up_;
// //   SDL_Window* window_;
// //   SDL_Renderer* renderer_;
// //   int key_code_;
// //   bool key_pressed_;
// // };

// // int main(int argc, char * argv[])
// // {
// //   rclcpp::init(argc, argv);  
// //   rclcpp::NodeOptions node_options;
// //   node_options.automatically_declare_parameters_from_overrides(true);
// //   auto node =
// //     rclcpp::Node::make_shared("user_input", node_options);
// //   auto user_input = std::make_shared<UserInput>(node);
// //   rclcpp::spin(node);
// //   rclcpp::shutdown();
// //   return 0;
// // }

// /***************************** */
// #include <chrono>
// #include <functional>
// #include <memory>
// #include <string>

// #include "rclcpp/rclcpp.hpp"
// #include <keyboard_msgs/msg/key.hpp>
// #include <SDL.h>

// using namespace std::chrono_literals;

// class UserInput
// {
// public:
//   UserInput(const rclcpp::Node::SharedPtr& node)
//   : key_code_(0), key_pressed_(false) 
//   {
//     node_ = node;

//     rclcpp::QoS qos_profile(10);
//     // qos_profile.reliable(); 
//     qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL); 

//     // pub_down_ = node_->create_publisher<keyboard_msgs::msg::Key>("Keydown", 10);
//     // pub_up_ = node_->create_publisher<keyboard_msgs::msg::Key>("Keyup", 10);

//     pub_down_ = node_->create_publisher<keyboard_msgs::msg::Key>("Keydown", qos_profile);
//     pub_up_ = node_->create_publisher<keyboard_msgs::msg::Key>("Keyup", qos_profile);

//     timer_ = node_->create_wall_timer(20ms, std::bind(&UserInput::publish_input, this));

//     node_->declare_parameter("allow_repeat", false);
//     bool allow_repeat = node_->get_parameter("allow_repeat").as_bool();

//     node_->declare_parameter("repeat_delay", 500);
//     int repeat_delay = node_->get_parameter("repeat_delay").as_int();

//     node_->declare_parameter("repeat_interval", 30);  
//     int repeat_interval = node_->get_parameter("repeat_interval").as_int();
    
//     if (!allow_repeat) repeat_delay = 0; // disable

//     if (SDL_Init(SDL_INIT_VIDEO) < 0) {
//       throw std::runtime_error("Could not init SDL: " + std::string(SDL_GetError()));
//     }

//     window_ = SDL_CreateWindow("ROS keyboard input", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 100, 100, SDL_WINDOW_SHOWN);
//     if (!window_) {
//       SDL_Quit();
//       throw std::runtime_error("Could not create SDL window: " + std::string(SDL_GetError()));
//     }

//     renderer_ = SDL_CreateRenderer(window_, -1, SDL_RENDERER_ACCELERATED);
//     if (!renderer_) {
//       SDL_DestroyWindow(window_);
//       SDL_Quit();
//       throw std::runtime_error("Could not create SDL renderer: " + std::string(SDL_GetError()));
//     }

//     SDL_SetRenderDrawColor(renderer_, 0, 0, 0, SDL_ALPHA_OPAQUE);
//     SDL_RenderClear(renderer_);
//     SDL_RenderPresent(renderer_);
//   }

//   ~UserInput()
//   {
//     SDL_DestroyRenderer(renderer_);
//     SDL_DestroyWindow(window_);
//     SDL_Quit();
//   }

// private:
//   void publish_input()
//   {  
//     SDL_Event event;
//     while (SDL_PollEvent(&event)) {
//       if (event.type == SDL_KEYDOWN) {
//         handle_key_event(event.key, true);  // Key down event
//       }
//       else if (event.type == SDL_KEYUP) {
//         handle_key_event(event.key, false); // Key up event
//       }
//       else if (event.type == SDL_QUIT) {
//         SDL_DestroyRenderer(renderer_);
//         SDL_DestroyWindow(window_);
//         SDL_Quit();
//         timer_->cancel();
//         return;
//       }
//     }
//   }

//   void handle_key_event(const SDL_KeyboardEvent& key_event, bool is_key_down) 
//   {
//     static std::map<SDL_Keycode, bool> key_state; // stores key_state

//     if (is_key_down) {
//       // Only publish if the key is not already considered "pressed"
//       if (key_state.find(key_event.keysym.sym) == key_state.end() || !key_state[key_event.keysym.sym]) {
//         keyboard_msgs::msg::Key k;
//         k.code = key_event.keysym.sym;
//         k.modifiers = key_event.keysym.mod;

//         RCLCPP_INFO(node_->get_logger(), "Publishing key down: code=%d", k.code);
//         pub_down_->publish(k);

//         key_state[key_event.keysym.sym] = true; // Mark the key as pressed
//         SDL_SetRenderDrawColor(renderer_, k.code % 256, 0, 255, SDL_ALPHA_OPAQUE);
//         SDL_RenderClear(renderer_);
//         SDL_RenderPresent(renderer_);
//       }
//     } 
//     else {
//       // Key is released, update state
//       if (key_state.find(key_event.keysym.sym) != key_state.end() && key_state[key_event.keysym.sym]) {
//         keyboard_msgs::msg::Key k;
//         k.code = key_event.keysym.sym;
//         k.modifiers = key_event.keysym.mod;

//         RCLCPP_INFO(node_->get_logger(), "Publishing key up: code=%d", k.code);
//         pub_up_->publish(k);

//         key_state[key_event.keysym.sym] = false; // update to false if key states changed to key_up
//         SDL_SetRenderDrawColor(renderer_, 0, 0, 0, SDL_ALPHA_OPAQUE);
//         SDL_RenderClear(renderer_);
//         SDL_RenderPresent(renderer_);
//       }
//     }
//   }

//   rclcpp::Node::SharedPtr node_;
//   rclcpp::TimerBase::SharedPtr timer_;
//   rclcpp::Publisher<keyboard_msgs::msg::Key>::SharedPtr pub_down_;
//   rclcpp::Publisher<keyboard_msgs::msg::Key>::SharedPtr pub_up_;
//   SDL_Window* window_;
//   SDL_Renderer* renderer_;
//   int key_code_;
//   bool key_pressed_;
// };

// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);  
//   rclcpp::NodeOptions node_options;
//   node_options.automatically_declare_parameters_from_overrides(true);
//   auto node = rclcpp::Node::make_shared("user_input", node_options);
//   auto user_input = std::make_shared<UserInput>(node);
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }
