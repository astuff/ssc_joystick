// Copyright (c) 2020 AutonomouStuff, LLC
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <stdio.h>
#include <unistd.h>
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/string.hpp"

#include "automotive_platform_msgs/msg/gear_command.hpp"
#include "automotive_platform_msgs/msg/gear_feedback.hpp"
#include "automotive_platform_msgs/msg/turn_signal_command.hpp"
#include "automotive_platform_msgs/msg/user_input_adas.hpp"
#include "automotive_platform_msgs/msg/speed_mode.hpp"
#include "automotive_platform_msgs/msg/steer_mode.hpp"
#include "automotive_platform_msgs/msg/velocity_accel_cov.hpp"
#include "automotive_navigation_msgs/msg/module_state.hpp"

#include "Json.hpp"
#include "GeneralUtils.hpp"

namespace apm = automotive_platform_msgs;
namespace anm = automotive_navigation_msgs;

double joy_fault_timeout = 0.0;

bool engage_speed_module = 0;
bool engage_steering_module = 0;

uint32_t engage1_button = 0;
uint32_t engage2_button = 0;
uint32_t park_button = 0;
uint32_t neutral_button = 0;
uint32_t drive_button = 0;
uint32_t reverse_button = 0;

uint32_t right_turn_button = 0;
uint32_t left_turn_button = 0;

uint32_t speed_axes = 0;
float speed_up_sign = 0.0;
float speed_step = 0.0;
float speed_max = 0.0;
float acceleration_limit = 0.0;
float deceleration_limit = 0.0;
uint32_t brake_axes = 0;
float brake_sign = 0.0;
float max_deceleration_limit = 0.0;

uint32_t steer_btn_axes = 0;
int32_t steer_btn_sign = 0;
float steer_btn_step = 0;
uint32_t steering_axes = 0;
float steering_sign = 0.0;
float steering_gain = 0.0;
float steering_exponent = 0.0;
float max_curvature_rate = 0.0;

std::string vel_controller_name = "";  // NOLINT

bool dbw_ok = false;
uint16_t engaged = 0;
bool engage_pressed = false;
double last_joystick_msg = 0.0;
int8_t speed_last = 0;
float desired_speed = 0.0;
float desired_curvature = 0.0;
bool steering_active_last_loop = false;
int8_t steer_last = 0;
bool brake_inited = false;  // Brake axes default is 0 (50%) until it's pressed
bool brake_active = false;
float deceleration = 0.0;

uint8_t current_gear = apm::msg::Gear::NONE;
float current_velocity = 1.0;

apm::msg::SpeedMode speed_command_msg;
std::shared_ptr<rclcpp::Publisher<apm::msg::SpeedMode>> speed_command_pub;

apm::msg::SteerMode steer_command_msg;
std::shared_ptr<rclcpp::Publisher<apm::msg::SteerMode>> steer_command_pub;

apm::msg::GearCommand gear_command_msg;
std::shared_ptr<rclcpp::Publisher<apm::msg::GearCommand>> gear_command_pub;

apm::msg::TurnSignalCommand turn_signal_command_msg;
std::shared_ptr<rclcpp::Publisher<apm::msg::TurnSignalCommand>> turn_signal_command_pub;

std_msgs::msg::String config_msg;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> config_pub;
size_t config_subscription_count = 0;


void disengage()
{
  std::cout << "DISENGAGED" << std::endl;
  engaged = 0;
}

void tryToEngage()
{
  if (!dbw_ok) {
    std::cout << "Drive by wire system not ready to engage" << std::endl;

  } else if ((current_gear != apm::msg::Gear::PARK) && (current_gear != apm::msg::Gear::NEUTRAL)) {
    std::cout << "Gear must be in park or neutral to engage" << std::endl;

  } else {
    std::cout << "ENGAGED" << std::endl;
    desired_speed = 0.0;
    desired_curvature = 0.0;
    gear_command_msg.command.gear = current_gear;
    engaged = 1;
  }
}

// Callback functions
void joystickCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  last_joystick_msg = rclcpp::Time(msg->header.stamp).seconds();

  if ((msg->buttons.at(engage1_button) > 0) && (msg->buttons.at(engage2_button) > 0)) {
    if (!engage_pressed) {
      if (engaged > 0) {
        disengage();

      } else {
        tryToEngage();
      }
      engage_pressed = true;
    }

  } else if ((msg->buttons.at(engage1_button) > 0) || (msg->buttons.at(engage2_button) > 0)) {
    if ((engaged > 0) && !engage_pressed) {
      disengage();
      engage_pressed = true;
    }

  } else {
    engage_pressed = false;
  }

  if (engaged > 0) {
    if (msg->buttons.at(park_button) > 0) {
      if (current_velocity > 0.1) {
        std::cout << "Must be stopped to change to park" << std::endl;

      } else {
        gear_command_msg.command.gear = apm::msg::Gear::PARK;
      }

    } else if (msg->buttons.at(neutral_button) > 0) {
      gear_command_msg.command.gear = apm::msg::Gear::NEUTRAL;

    } else if (msg->buttons.at(drive_button) > 0) {
      gear_command_msg.command.gear = apm::msg::Gear::DRIVE;

    } else if (msg->buttons.at(reverse_button) > 0) {
      gear_command_msg.command.gear = apm::msg::Gear::REVERSE;
    }

    if (msg->buttons.at(right_turn_button) > 0) {
      turn_signal_command_msg.turn_signal = apm::msg::TurnSignalCommand::RIGHT;
      turn_signal_command_msg.mode = 1;

    } else if (msg->buttons.at(left_turn_button) > 0) {
      turn_signal_command_msg.turn_signal = apm::msg::TurnSignalCommand::LEFT;
      turn_signal_command_msg.mode = 1;

    } else {
      turn_signal_command_msg.turn_signal = apm::msg::TurnSignalCommand::NONE;
      turn_signal_command_msg.mode = 0;
    }

    float speed = msg->axes.at(speed_axes);
    bool speed_updated = false;
    if (speed > 0.1) {
      if (speed_last != 1) {
        desired_speed += speed_up_sign * speed_step;
        speed_updated = true;
      }
      speed_last = 1;

    } else if (speed < -0.1) {
      if (speed_last != -1) {
        desired_speed -= speed_up_sign * speed_step;
        speed_updated = true;
      }
      speed_last = -1;

    } else {
      speed_last = 0;
    }

    float brake = msg->axes.at(brake_axes);

    if (brake != 0.0) {
      brake_inited = true;
    }

    if (brake_inited) {
      brake *= brake_sign;
      if (brake < 0.95) {
        if (!brake_active) {
          brake_active = true;
          desired_speed = 0.0;
          speed_updated = true;
        }
        deceleration = static_cast<float>(
          AS::map2pt(brake, -0.95, 0.95, max_deceleration_limit, deceleration_limit));

      } else {
        if (brake_active) {
          brake_active = false;
          desired_speed = current_velocity / 0.44704f;
          desired_speed = static_cast<float>(speed_step * floor(desired_speed / speed_step));
          speed_updated = true;
          deceleration = deceleration_limit;
        }
      }
    }

    if (speed_updated) {
      desired_speed = static_cast<float>(speed_step * round(desired_speed / speed_step));

      if (desired_speed > speed_max) {
        desired_speed = speed_max;

      } else if (desired_speed < 0.1) {
        desired_speed = 0.0;
      }

      std::cout << "Desired Speed: " << desired_speed << std::endl;
    }

    float steering = msg->axes.at(steering_axes);
    if ((steering > 0.01) || (steering < -0.01)) {
      float raw = steering * steering_sign;
      float raw_sign = 1.0f;
      if (raw < 0.0) {
        raw *= -1.0f;
        raw_sign = -1.0f;
      }
      desired_curvature =
        static_cast<float>(pow(raw, steering_exponent) * steering_gain * raw_sign);

      steering_active_last_loop = true;

    } else if (steering_active_last_loop) {
      desired_curvature = 0.0;
      steering_active_last_loop = false;

    } else {
      float steer = msg->axes.at(steer_btn_axes);
      bool steer_updated = false;

      if (steer > 0.1) {
        if (steer_last != 1) {
          desired_curvature += steer_btn_sign * steer_btn_step;
          steer_updated = true;
        }
        steer_last = 1;

      } else if (steer < -0.1) {
        if (steer_last != -1) {
          desired_curvature -= steer_btn_sign * steer_btn_step;
          steer_updated = true;
        }
        steer_last = -1;

      } else {
        steer_last = 0;
      }

      if (steer_updated) {
        desired_curvature =
          static_cast<float>(steer_btn_step * round(desired_curvature / steer_btn_step));

        if (desired_curvature > steering_gain) {
          desired_curvature = steering_gain;

        } else if (desired_curvature < -steering_gain) {
          desired_curvature = -steering_gain;
        }

        std::cout << "Desired Steering Curvature: " << desired_curvature << std::endl;
      }
    }
  } else {
    desired_speed = 0.0;
    desired_curvature = 0.0;
  }
}

void gearFeedbackCallback(const apm::msg::GearFeedback::SharedPtr msg)
{
  current_gear = msg->current_gear.gear;
}

void velocityCallback(const apm::msg::VelocityAccelCov::SharedPtr msg)
{
  current_velocity = msg->velocity;
}

void inputAdasCallback(const apm::msg::UserInputADAS::SharedPtr msg)
{
  if (msg->btn_cc_set_inc && msg->btn_acc_gap_inc) {
    if (engaged > 0) {
      disengage();
    }

  } else if (msg->btn_cc_set_dec && msg->btn_acc_gap_dec) {
    tryToEngage();
  }
}

void moduleStateCallback(const anm::msg::ModuleState::SharedPtr msg)
{
  if (msg->name == vel_controller_name) {
    if (msg->state == "not_ready") {
      dbw_ok = false;

    } else if ((msg->state == "ready") || (msg->state == "engaged") || (msg->state == "active")) {
      dbw_ok = true;

    } else if (msg->state == "failure") {
      if (dbw_ok && (engaged > 0)) {
        std::cout << "Joystick control DISENGAGED due to " << msg->info << std::endl;
        engaged = 0;
      }
      dbw_ok = false;

    } else if (msg->state == "fatal") {
      if (dbw_ok) {
        std::cout << "Joystick control unavailable due to " << msg->info << std::endl;
        std::cout << "Nodes must be stopped and restarted once the problem is fixed" << std::endl;
        engaged = 0;
      }
      dbw_ok = false;
    }
  }
}

void update(void)
{
  // Get current time
  auto now = rclcpp::Clock().now();
  double now_sec = now.seconds();

  if (last_joystick_msg == 0.0) {
    // Give joystick node a few seconds to start up
    last_joystick_msg = now_sec + 5.0;

  } else if ((now_sec - last_joystick_msg) > joy_fault_timeout) {
    // Joystick has timed out
    std::cout << "JOYSTICK TIMEOUT" << std::endl;
    last_joystick_msg = now_sec;
    engaged = 0;
  }

  // Send output messages
  speed_command_msg.header.stamp = now;
  speed_command_msg.mode = engage_speed_module > 0 ? engaged : 0;
  speed_command_msg.speed = desired_speed * 0.44704f;
  speed_command_msg.acceleration_limit = acceleration_limit;
  speed_command_msg.deceleration_limit = deceleration;
  speed_command_pub->publish(speed_command_msg);

  steer_command_msg.header.stamp = now;
  steer_command_msg.mode = engage_steering_module > 0 ? engaged : 0;
  steer_command_msg.curvature = desired_curvature;
  steer_command_msg.max_curvature_rate = max_curvature_rate;
  steer_command_pub->publish(steer_command_msg);

  gear_command_pub->publish(gear_command_msg);

  turn_signal_command_pub->publish(turn_signal_command_msg);

  // Publish the config message if there's a new subscriber
  size_t count =
    config_pub->get_subscription_count() + config_pub->get_intra_process_subscription_count();

  if (count > config_subscription_count) {
    config_pub->publish(config_msg);
  }

  config_subscription_count = count;
}

// Main routine
int main(int argc, char ** argv)
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  int c;
  bool exit = false;
  bool config_file_provided = false;
  std::string config_file;

  double publish_interval = 0.0;

  optind = 0;  // Resets the parsing index for getopt.
  opterr = 1;  // Re-enables showing an error for invalid parameters.

  while ((c = getopt(argc, argv, "hf:")) != -1) {
    switch (c) {
      case 'h':
        std::cout << std::endl;
        std::cout << "Joystick Controller for AutonomouStuff Speed and Steering Control Modules" <<
          std::endl;
        std::cout << "    -h             Show this help menu and exit." << std::endl;
        std::cout <<
          "    -f <file.json> The JSON configuration file for all remaining parameters." <<
          " See ssc_joystick.json for an example." << std::endl;
        std::cout << std::endl;
        exit = true;
        break;
      case 'f':
        config_file = optarg;
        config_file_provided = true;
        break;
      case '?':
        exit = true;
        break;
      default:
        break;
    }
  }

  if (!exit && !config_file_provided) {
    std::cout << std::endl;
    std::cout << "Required parameters: " << std::endl;
    std::cout <<
      "    -f <file.json>   The JSON configuration file for all required parameters." <<
      " See ssc_joystick.json for an example." << std::endl;
    std::cout << std::endl;
    exit = true;
  }

  std_msgs::msg::String config_msg;

  if (!exit) {
    // Parse JSON configuration parameters
    json json_obj = AS::JSON::deserialize(config_file, &config_msg.data);

    bool config_ok = true;
    std::string mod_name = "ssc joystick";

    config_ok &=
      AS::readJsonWithLimit(mod_name, json_obj, "publish_interval", ">", 0.0, &publish_interval);

    config_ok &=
      AS::readJsonWithLimit(mod_name, json_obj, "joy_fault_timeout", ">", 0.0, &joy_fault_timeout);

    config_ok &=
      AS::readJsonWithError(mod_name, json_obj, "vel_controller_name", &vel_controller_name);

    config_ok &=
      AS::readJsonWithError(mod_name, json_obj, "engage_speed_module", &engage_speed_module);
    config_ok &=
      AS::readJsonWithError(mod_name, json_obj, "engage_steering_module", &engage_steering_module);

    config_ok &= AS::readJsonWithError(mod_name, json_obj, "engage1_button", &engage1_button);
    config_ok &= AS::readJsonWithError(mod_name, json_obj, "engage2_button", &engage2_button);

    config_ok &= AS::readJsonWithError(mod_name, json_obj, "park_button", &park_button);
    config_ok &= AS::readJsonWithError(mod_name, json_obj, "neutral_button", &neutral_button);
    config_ok &= AS::readJsonWithError(mod_name, json_obj, "drive_button", &drive_button);
    config_ok &= AS::readJsonWithError(mod_name, json_obj, "reverse_button", &reverse_button);

    config_ok &= AS::readJsonWithError(mod_name, json_obj, "right_turn_button", &right_turn_button);
    config_ok &= AS::readJsonWithError(mod_name, json_obj, "left_turn_button", &left_turn_button);

    config_ok &= AS::readJsonWithError(mod_name, json_obj, "speed_axes", &speed_axes);
    config_ok &= AS::readJsonWithError(mod_name, json_obj, "speed_up_sign", &speed_up_sign);
    config_ok &= AS::readJsonWithLimit(mod_name, json_obj, "speed_step", ">", 0.0f, &speed_step);
    config_ok &= AS::readJsonWithLimit(mod_name, json_obj, "speed_max", ">", 0.0f, &speed_max);

    config_ok &= AS::readJsonWithLimit(
      mod_name,
      json_obj,
      "acceleration_limit",
      ">",
      0.0f,
      &acceleration_limit);

    config_ok &= AS::readJsonWithLimit(
      mod_name,
      json_obj,
      "deceleration_limit",
      ">",
      0.0f,
      &deceleration_limit);

    config_ok &= AS::readJsonWithError(mod_name, json_obj, "brake_axes", &brake_axes);
    config_ok &= AS::readJsonWithError(mod_name, json_obj, "brake_sign", &brake_sign);

    config_ok &= AS::readJsonWithLimit(
      mod_name,
      json_obj,
      "max_deceleration_limit",
      ">",
      0.0f,
      &max_deceleration_limit);

    config_ok &= AS::readJsonWithError(mod_name, json_obj, "steer_btn_axes", &steer_btn_axes);
    config_ok &= AS::readJsonWithError(mod_name, json_obj, "steer_btn_sign", &steer_btn_sign);
    config_ok &=
      AS::readJsonWithLimit(mod_name, json_obj, "steer_btn_step", ">", 0.0f, &steer_btn_step);
    config_ok &= AS::readJsonWithError(mod_name, json_obj, "steering_axes", &steering_axes);
    config_ok &= AS::readJsonWithError(mod_name, json_obj, "steering_sign", &steering_sign);
    config_ok &=
      AS::readJsonWithLimit(mod_name, json_obj, "steering_gain", ">", 0.0f, &steering_gain);
    config_ok &=
      AS::readJsonWithLimit(mod_name, json_obj, "steering_exponent", ">", 0.0f, &steering_exponent);

    config_ok &= AS::readJsonWithLimit(
      mod_name,
      json_obj,
      "max_curvature_rate",
      ">",
      0.0f,
      &max_curvature_rate);

    if (!config_ok) {
      exit = true;
    }
  }

  deceleration = deceleration_limit;

  if (engage_speed_module || engage_steering_module) {
    std::cout << "\nSPEED MODULE MODE: " << engage_speed_module << "\nSTEERING MODULE MODE: " <<
      engage_steering_module << std::endl;

    if (engage_speed_module && engage_steering_module) {
      std::cout << "\nSPEED AND STEERING CONTROL SET TO ENGAGE" << std::endl;
    }

  } else {
    std::cout << "\nNO MODULE HAS BEEN SET TO ENGAGE, SSC WILL NOT BE ACTIVE" << std::endl;
  }

  if (exit) {
    return 0;
  }

  // ROS initialization
  rclcpp::init(argc, argv);

  auto options = rclcpp::NodeOptions();
  auto node = rclcpp::Node::make_shared("ssc_joystick", options);

  std::string config_topic = node->get_name();
  config_topic.append("_config");

  // Advertise messages to send
  speed_command_pub = node->create_publisher<apm::msg::SpeedMode>("arbitrated_speed_commands", 2);
  steer_command_pub =
    node->create_publisher<apm::msg::SteerMode>("arbitrated_steering_commands", 2);
  gear_command_pub = node->create_publisher<apm::msg::GearCommand>("gear_select", 2);
  turn_signal_command_pub =
    node->create_publisher<apm::msg::TurnSignalCommand>("turn_signal_command", 2);
  config_pub = node->create_publisher<std_msgs::msg::String>(config_topic, 1);

  // Subscribe to messages to read
  auto state_sub =
    node->create_subscription<anm::msg::ModuleState>("module_states", 10, moduleStateCallback);
  auto joy_sub = node->create_subscription<sensor_msgs::msg::Joy>("joy", 10, joystickCallback);
  auto gear_sub =
    node->create_subscription<apm::msg::GearFeedback>("gear_feedback", 10, gearFeedbackCallback);

  auto velocity_sub = node->create_subscription<apm::msg::VelocityAccelCov>(
    "velocity_accel_cov",
    10,
    velocityCallback);

  auto adas_input_sub =
    node->create_subscription<apm::msg::UserInputADAS>("adas_input", 10, inputAdasCallback);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);

  // Wait for time to be valid
  while (rclcpp::Clock().now().seconds() == 0.0) {}

  std::shared_ptr<rclcpp::TimerBase> timer = node->create_wall_timer(
    std::chrono::duration<double>(publish_interval),
    update);

  exec.spin();

  rclcpp::shutdown();

  return 0;
}
