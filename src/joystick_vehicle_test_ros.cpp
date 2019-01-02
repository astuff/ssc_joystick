/*
* Unpublished Copyright (c) 2009-2019 AutonomouStuff, LLC, All Rights Reserved.
*
* This file is part of the Joystick Vehicle Test ROS 1.0 application which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/

#include <ros/ros.h>

#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <diagnostic_msgs/DiagnosticArray.h>

#include <automotive_platform_msgs/GearCommand.h>
#include <automotive_platform_msgs/GearFeedback.h>
#include <automotive_platform_msgs/TurnSignalCommand.h>
#include <automotive_platform_msgs/UserInputADAS.h>
#include <automotive_platform_msgs/SpeedMode.h>
#include <automotive_platform_msgs/SteerMode.h>
#include <automotive_platform_msgs/VelocityAccel.h>
#include <automotive_navigation_msgs/ModuleState.h>

#include <Json.hpp>
#include <GeneralUtils.hpp>

using namespace std;
using namespace AS;

class JoystickVehicleTestRos
{
public:
  JoystickVehicleTestRos() :
    joy_fault_timeout_(0.0),
    engage1_button_(0),
    engage2_button_(0),
    park_button_(0),
    neutral_button_(0),
    drive_button_(0),
    reverse_button_(0),
    right_turn_button_(0),
    left_turn_button_(0),
    speed_axes_(0),
    speed_up_sign_(0.0),
    speed_step_(0.0),
    speed_max_(0.0),
    acceleration_rate_(0.0),
    deceleration_rate_(0.0),
    acceleration_limit_(0.0),
    deceleration_limit_(0.0),
    brake_axes_(0),
    brake_sign_(0.0),
    max_deceleration_limit_(0.0),
    steer_btn_axes_(0),
    steer_btn_sign_(0),
    steer_btn_step_(0),
    steering_axes_(0),
    steering_sign_(0.0),
    steering_gain_(0.0),
    steering_exponent_(0.0),
    max_curvature_rate_(0.0),
    vel_controller_name_(),
    dbw_ok_(false),
    engaged_(0),
    engage_pressed_(false),
    last_joystick_msg_(0.0),
    speed_last_(0),
    desired_speed_(0.0),
    desired_speed_last_(0.0),
    desired_curvature_(0.0),
    steering_active_last_loop_(false),
    steer_last_(0), brake_inited_(false),
    brake_active_(false),
    deceleration_(0.0),
    current_gear_(automotive_platform_msgs::Gear::NONE),
    current_velocity_(1.0),
    gear_command_msg_(),
    gear_command_pub_(),
    turn_signal_command_msg_(),
    turn_signal_command_pub_()
  {
  }

  double joy_fault_timeout_;

  uint32_t engage1_button_;
  uint32_t engage2_button_;

  uint32_t park_button_;
  uint32_t neutral_button_;
  uint32_t drive_button_;
  uint32_t reverse_button_;

  uint32_t right_turn_button_;
  uint32_t left_turn_button_;

  uint32_t speed_axes_;
  float speed_up_sign_;
  float speed_step_;
  float speed_max_;
  float acceleration_rate_;
  float deceleration_rate_;
  float acceleration_limit_;
  float deceleration_limit_;
  uint32_t brake_axes_;
  float brake_sign_;
  float max_deceleration_limit_;

  uint32_t steer_btn_axes_;
  int32_t steer_btn_sign_;
  float steer_btn_step_;
  uint32_t steering_axes_;
  float steering_sign_;
  float steering_gain_;
  float steering_exponent_;
  float max_curvature_rate_;

  string vel_controller_name_;

  bool dbw_ok_;
  uint16_t engaged_;
  bool engage_pressed_;
  double last_joystick_msg_;
  int32_t speed_last_;
  float desired_speed_;  // MPH
  float desired_speed_last_;  // M/S
  float desired_curvature_;
  bool steering_active_last_loop_;
  int32_t steer_last_;
  bool brake_inited_;  // Brake axes default is 0 (50%) until it's pressed
  bool brake_active_;
  float deceleration_;

  uint8_t current_gear_;
  float current_velocity_;

  automotive_platform_msgs::GearCommand gear_command_msg_;
  ros::Publisher gear_command_pub_;

  automotive_platform_msgs::TurnSignalCommand turn_signal_command_msg_;
  ros::Publisher turn_signal_command_pub_;

  void disengage()
  {
    cout << "DISENGAGED" << endl;
    engaged_ = 0;
    if ((current_gear_ == automotive_platform_msgs::Gear::DRIVE) ||
        (current_gear_ == automotive_platform_msgs::Gear::REVERSE))
    {
      gear_command_msg_.command.gear = automotive_platform_msgs::Gear::PARK;
      gear_command_pub_.publish(gear_command_msg_);
    }
  }

  void tryToEngage()
  {
    if (!dbw_ok_)
    {
      cout << "Drive by wire system not ready to engage" << endl;
    }
    else if ((current_gear_ != automotive_platform_msgs::Gear::PARK) &&
             (current_gear_ != automotive_platform_msgs::Gear::NEUTRAL))
    {
      cout << "Gear must be in park or neutral to engage" << endl;
    }
    else
    {
      cout << "ENGAGED" << endl;
      desired_speed_ = 0.0;
      desired_speed_last_ = 0.0;
      desired_curvature_ = 0.0;
      engaged_ = 1;
    }
  }

  // Callback functions
  void joystickCallback(const sensor_msgs::Joy::ConstPtr &msg)
  {
    if ((msg->buttons.at(engage1_button_) > 0) &&
        (msg->buttons.at(engage2_button_) > 0))
    {
      if (!engage_pressed_)
      {
        if (engaged_ > 0)
        {
          disengage();
        }
        else
        {
          tryToEngage();
        }
        engage_pressed_ = true;
      }
    }
    else if ((msg->buttons.at(engage1_button_) > 0) ||
             (msg->buttons.at(engage2_button_) > 0))
    {
      if ((engaged_ > 0) && !engage_pressed_)
      {
        disengage();
        engage_pressed_ = true;
      }
    }
    else
    {
      engage_pressed_ = false;
    }

    if (engaged_ > 0)
    {
      if (msg->buttons.at(park_button_) > 0)
      {
        if (fabs(current_velocity_) > 0.1)
        {
          cout << "Must be stopped to change to park" << endl;
        }
        else
        {
          gear_command_msg_.command.gear = automotive_platform_msgs::Gear::PARK;
          gear_command_pub_.publish(gear_command_msg_);
        }
      }
      else if (msg->buttons.at(neutral_button_) > 0)
      {
        gear_command_msg_.command.gear = automotive_platform_msgs::Gear::NEUTRAL;
        gear_command_pub_.publish(gear_command_msg_);
      }
      else if (msg->buttons.at(drive_button_) > 0)
      {
        gear_command_msg_.command.gear = automotive_platform_msgs::Gear::DRIVE;
        gear_command_pub_.publish(gear_command_msg_);
      }
      else if (msg->buttons.at(reverse_button_) > 0)
      {
        gear_command_msg_.command.gear = automotive_platform_msgs::Gear::REVERSE;
        gear_command_pub_.publish(gear_command_msg_);
      }

      if (msg->buttons.at(right_turn_button_) > 0)
      {
        turn_signal_command_msg_.turn_signal = automotive_platform_msgs::TurnSignalCommand::RIGHT;
        turn_signal_command_msg_.mode = 1;
      }
      else if (msg->buttons.at(left_turn_button_) > 0)
      {
        turn_signal_command_msg_.turn_signal = automotive_platform_msgs::TurnSignalCommand::LEFT;
        turn_signal_command_msg_.mode = 1;
      }
      else
      {
        turn_signal_command_msg_.turn_signal = automotive_platform_msgs::TurnSignalCommand::NONE;
        turn_signal_command_msg_.mode = 0;
      }

      float speed = msg->axes.at(speed_axes_);
      bool speed_updated = false;
      if (speed > 0.1)
      {
        if (speed_last_ != 1)
        {
          desired_speed_ += speed_up_sign_ * speed_step_;
          speed_updated = true;
        }
        speed_last_ = 1;
      }
      else if (speed < -0.1)
      {
        if (speed_last_ != -1)
        {
          desired_speed_ -= speed_up_sign_ * speed_step_;
          speed_updated = true;
        }
        speed_last_ = -1;
      }
      else
      {
        speed_last_ = 0;
      }

      float brake = msg->axes.at(brake_axes_);
      if (brake != 0.0)
      {
        brake_inited_ = true;
      }
      if (brake_inited_)
      {
        brake *= brake_sign_;
        if (brake < 0.95)
        {
          if (!brake_active_)
          {
            brake_active_ = true;
            desired_speed_ = 0.0;
            desired_speed_last_ = 0.0;
            speed_updated = true;
          }
          deceleration_ = static_cast<float>(map2pt(brake, -0.95, 0.95, max_deceleration_limit_, deceleration_limit_));
        }
        else
        {
          if (brake_active_)
          {
            brake_active_ = false;
            desired_speed_ = fabs(current_velocity_) / 0.44704f;
            desired_speed_ = speed_step_ * floor(desired_speed_ / speed_step_);
            desired_speed_last_ = desired_speed_ * 0.44704f;
            speed_updated = true;
            deceleration_ = deceleration_limit_;
          }
        }
      }

      if (speed_updated)
      {
        desired_speed_ = speed_step_ * round(desired_speed_ / speed_step_);

        if (desired_speed_ > speed_max_)
        {
          desired_speed_ = speed_max_;
        }
        else if (desired_speed_ < 0.1)
        {
          desired_speed_ = 0.0;
        }

        cout << "Desired Speed: " << desired_speed_ << endl;
      }

      float steering = msg->axes.at(steering_axes_);
      if ((steering > 0.01) || (steering < -0.01))
      {
        float raw = steering * steering_sign_;
        float raw_sign = 1.0f;
        if (raw < 0.0)
        {
          raw *= -1.0f;
          raw_sign = -1.0f;
        }
        desired_curvature_ = pow(raw, steering_exponent_) * steering_gain_ * raw_sign;
        steering_active_last_loop_ = true;
      }
      else if (steering_active_last_loop_)
      {
        desired_curvature_ = 0.0;
        steering_active_last_loop_ = false;
      }
      else
      {
        float steer = msg->axes.at(steer_btn_axes_);
        bool steer_updated = false;
        if (steer > 0.1)
        {
          if (steer_last_ != 1)
          {
            desired_curvature_ += steer_btn_sign_ * steer_btn_step_;
            steer_updated = true;
          }
          steer_last_ = 1;
        }
        else if (steer < -0.1)
        {
          if (steer_last_ != -1)
          {
            desired_curvature_ -= steer_btn_sign_ * steer_btn_step_;
            steer_updated = true;
          }
          steer_last_ = -1;
        }
        else
        {
          steer_last_ = 0;
        }

        if (steer_updated)
        {
          desired_curvature_ = steer_btn_step_ * round(desired_curvature_ / steer_btn_step_);

          if (desired_curvature_ > steering_gain_)
          {
            desired_curvature_ = steering_gain_;
          }
          else if (desired_curvature_ < -steering_gain_)
          {
            desired_curvature_ = -steering_gain_;
          }

          cout << "Desired Steering Curvature: " << desired_curvature_ << endl;
        }
      }
    }
    else
    {
      desired_speed_ = 0.0;
      desired_speed_last_ = 0.0;
      desired_curvature_ = 0.0;
    }
  }

  void diagnosticCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr &msg)
  {
    for (auto it = msg->status.begin(); it < msg->status.end(); it++)
    {
      if (it->name.find("Joystick Driver Status") != string::npos)
      {
        last_joystick_msg_ = msg->header.stamp.toSec();
        if (it->level != diagnostic_msgs::DiagnosticStatus::OK)
        {
          cout << "JOYSTICK FAULT" << endl;
          engaged_ = 0;
          brake_inited_ = false;
          brake_active_ = false;
        }
      }
    }
  }

  void gearFeedbackCallback(const automotive_platform_msgs::GearFeedback::ConstPtr &msg)
  {
    current_gear_ = msg->current_gear.gear;
  }

  void velocityCallback(const automotive_platform_msgs::VelocityAccel::ConstPtr &msg)
  {
    current_velocity_ = msg->velocity;
  }

  void inputAdasCallback(const automotive_platform_msgs::UserInputADAS::ConstPtr &msg)
  {
    if (msg->btn_cc_set_inc && msg->btn_acc_gap_inc)
    {
      if (engaged_ > 0)
      {
        disengage();
      }
    }
    else if (msg->btn_cc_set_dec && msg->btn_acc_gap_dec)
    {
      tryToEngage();
    }
  }

  void moduleStateCallback(const automotive_navigation_msgs::ModuleState::ConstPtr &msg)
  {
    if (msg->name == vel_controller_name_)
    {
      if (msg->state == "not_ready")
      {
        dbw_ok_ = false;
      }
      else if ((msg->state == "ready") || (msg->state == "engaged") || (msg->state == "active"))
      {
        dbw_ok_ = true;
      }
      else if (msg->state == "failure")
      {
        if (dbw_ok_ && (engaged_ > 0))
        {
          cout << "Joystick control DISENGAGED due to " << msg->info << endl;
          gear_command_msg_.command.gear = automotive_platform_msgs::Gear::NEUTRAL;
          gear_command_pub_.publish(gear_command_msg_);
          engaged_ = 0;
        }
        dbw_ok_ = false;
      }
      else if (msg->state == "fatal")
      {
        if (dbw_ok_)
        {
          cout << "Joystick control unavailable due to " << msg->info << endl;
          cout << "Software must be stopped and restarted once the problem is fixed" << endl;
          engaged_ = 0;
        }
        dbw_ok_ = false;
      }
    }
  }

  // Main routine
  int main(int argc, char **argv)
  {
    int c;
    bool exit = false;
    bool config_file_provided = false;
    string config_file;

    double publish_interval = 0.0;

    optind = 0;  // Resets the parsing index for getopt.
    opterr = 1;  // Re-enables showing an error for invalid parameters.

    while ((c = getopt(argc, argv, "hf:")) != -1)
    {
      switch (c)
      {
        case 'h':
          cout << endl;
          cout << "Joystick testing for AutonomouStuff Vehicle Control Modules" << endl;
          cout << "    -h             Show this help menu and exit." << endl;
          cout
            << "    -f <file.json> The JSON configuration file for all remaining parameters. "
            << "See joystick_vehicle_test.json for an example."
            << endl;
          cout << endl;
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

    if (!exit && !config_file_provided)
    {
      cout << endl;
      cout << "Required parameters: " << endl;
      cout
        << "    -f <file.json>   The JSON configuration file for all required parameters. "
        << "See joystick_vehicle_test.json for an example."
        << endl;
      cout << endl;
      exit = true;
    }

    std_msgs::String config_msg;

    if (!exit)
    {
      // Parse JSON configuration parameters
      json json_obj = JSON::deserialize(config_file, &config_msg.data);


      bool config_ok = true;
      string mod_name = "joystick vehicle test";

      config_ok &= readJsonWithLimit(mod_name, json_obj, "publish_interval", ">", 0.0, &publish_interval);

      config_ok &= readJsonWithLimit(mod_name, json_obj, "joy_fault_timeout", ">", 0.0, &joy_fault_timeout_);

      config_ok &= readJsonWithError(mod_name, json_obj, "vel_controller_name", &vel_controller_name_);

      config_ok &= readJsonWithLimit(mod_name, json_obj, "engage1_button", ">=", 0u, &engage1_button_);
      config_ok &= readJsonWithLimit(mod_name, json_obj, "engage2_button", ">=", 0u, &engage2_button_);

      config_ok &= readJsonWithLimit(mod_name, json_obj, "park_button", ">=", 0u, &park_button_);
      config_ok &= readJsonWithLimit(mod_name, json_obj, "neutral_button", ">=", 0u, &neutral_button_);
      config_ok &= readJsonWithLimit(mod_name, json_obj, "drive_button", ">=", 0u, &drive_button_);
      config_ok &= readJsonWithLimit(mod_name, json_obj, "reverse_button", ">=", 0u, &reverse_button_);

      config_ok &= readJsonWithLimit(mod_name, json_obj, "right_turn_button", ">=", 0u, &right_turn_button_);
      config_ok &= readJsonWithLimit(mod_name, json_obj, "left_turn_button", ">=", 0u, &left_turn_button_);

      config_ok &= readJsonWithLimit(mod_name, json_obj, "speed_axes", ">=", 0u, &speed_axes_);
      config_ok &= readJsonWithError(mod_name, json_obj, "speed_up_sign", &speed_up_sign_);
      config_ok &= readJsonWithLimit(mod_name, json_obj, "speed_step", ">", 0.0f, &speed_step_);
      config_ok &= readJsonWithLimit(mod_name, json_obj, "speed_max", ">", 0.0f, &speed_max_);
      config_ok &= readJson(json_obj, "acceleration_rate", &acceleration_rate_);
      config_ok &= readJson(json_obj, "deceleration_rate", &deceleration_rate_);
      config_ok &= readJsonWithLimit(mod_name, json_obj, "acceleration_limit", ">", 0.0f, &acceleration_limit_);
      config_ok &= readJsonWithLimit(mod_name, json_obj, "deceleration_limit", ">", 0.0f, &deceleration_limit_);
      config_ok &= readJsonWithLimit(mod_name, json_obj, "brake_axes", ">=", 0u, &brake_axes_);
      config_ok &= readJsonWithError(mod_name, json_obj, "brake_sign", &brake_sign_);
      config_ok &= readJsonWithLimit(mod_name, json_obj, "max_deceleration_limit", ">", 0.0f, &max_deceleration_limit_);

      config_ok &= readJsonWithLimit(mod_name, json_obj, "steer_btn_axes", ">=", 0u, &steer_btn_axes_);
      config_ok &= readJsonWithError(mod_name, json_obj, "steer_btn_sign", &steer_btn_sign_);
      config_ok &= readJsonWithLimit(mod_name, json_obj, "steer_btn_step", ">", 0.0f, &steer_btn_step_);
      config_ok &= readJsonWithLimit(mod_name, json_obj, "steering_axes", ">=", 0u, &steering_axes_);
      config_ok &= readJsonWithError(mod_name, json_obj, "steering_sign", &steering_sign_);
      config_ok &= readJsonWithLimit(mod_name, json_obj, "steering_gain", ">", 0.0f, &steering_gain_);
      config_ok &= readJsonWithLimit(mod_name, json_obj, "steering_exponent", ">", 0.0f, &steering_exponent_);
      config_ok &= readJsonWithLimit(mod_name, json_obj, "max_curvature_rate", ">", 0.0f, &max_curvature_rate_);

      if (!config_ok)
      {
        exit = true;
      }
    }

    acceleration_rate_ *= publish_interval;
    deceleration_rate_ *= publish_interval;
    if (deceleration_rate_ > 0.0)
    {
      deceleration_rate_ *= -1.0;
    }

    deceleration_ = deceleration_limit_;

    if (exit)
    {
      return 0;
    }

    // ROS initialization
    ros::init(argc, argv, "joystick_vehicle_test");
    ros::NodeHandle n;
    ros::Rate loop_rate(1.0 / publish_interval);

    string config_topic = ros::this_node::getName();
    size_t split = config_topic.find_last_of('/');
    if (split != string::npos)
    {
      config_topic = config_topic.substr(split + 1);
    }
    config_topic.append("_config");

    // Advertise messages to send
    gear_command_pub_ = n.advertise<automotive_platform_msgs::GearCommand>("gear_select", 1);
    turn_signal_command_pub_ = n.advertise<automotive_platform_msgs::TurnSignalCommand>("turn_signal_command", 1);
    ros::Publisher speed_pub = n.advertise<automotive_platform_msgs::SpeedMode>("arbitrated_speed_commands", 1);
    ros::Publisher steer_pub = n.advertise<automotive_platform_msgs::SteerMode>("arbitrated_steering_commands", 1);
    ros::Publisher config_pub = n.advertise<std_msgs::String>(config_topic, 1, true);

    // Subscribe to messages to read
    ros::Subscriber state_sub = n.subscribe("module_states", 10, &JoystickVehicleTestRos::moduleStateCallback, this);
    ros::Subscriber joy_sub = n.subscribe("joy", 10, &JoystickVehicleTestRos::joystickCallback, this);
    ros::Subscriber joy_fault_sub = n.subscribe("diagnostics", 10, &JoystickVehicleTestRos::diagnosticCallback, this);
    ros::Subscriber gear_sub = n.subscribe("gear_feedback", 10, &JoystickVehicleTestRos::gearFeedbackCallback, this);
    ros::Subscriber velocity_sub = n.subscribe("velocity_accel", 10, &JoystickVehicleTestRos::velocityCallback, this);
    ros::Subscriber adas_input_sub = n.subscribe("adas_input", 10, &JoystickVehicleTestRos::inputAdasCallback, this);

    // Wait for time to be valid
    while (ros::Time::now().nsec == 0)
    {}

    automotive_platform_msgs::SpeedMode speed_msg;
    automotive_platform_msgs::SteerMode steer_msg;

    // Publish latched message containing the .json config file
    config_pub.publish(config_msg);

    // Loop as long as module should run
    while (ros::ok())
    {
      // Get current time
      ros::Time now = ros::Time::now();
      double now_sec = now.toSec();

      if (last_joystick_msg_ == 0.0)
      {
        // Give joystick node a few seconds to start up
        last_joystick_msg_ = now_sec + 5.0;
      }
      else if ((now_sec - last_joystick_msg_) > joy_fault_timeout_)
      {
        // Joystick has timed out
        cout << "JOYSTICK TIMEOUT" << endl;
        last_joystick_msg_ = now_sec;
        engaged_ = 0;
      }

      // Rate limit speed
      float speed = desired_speed_ * 0.44704f;
      if (speed > desired_speed_last_)
      {
        if (acceleration_rate_ > 0.0)
        {
          desired_speed_last_ += acceleration_rate_;
          if (desired_speed_last_ > speed)
          {
            desired_speed_last_ = speed;
          }
        }
        else
        {
          desired_speed_last_ = speed;
        }
      }
      else if (speed < desired_speed_last_)
      {
        if (deceleration_rate_ < 0.0)
        {
          desired_speed_last_ += deceleration_rate_;
          if (desired_speed_last_ < speed)
          {
            desired_speed_last_ = speed;
          }
        }
        else
        {
          desired_speed_last_ = speed;
        }
      }

      // Send output messages
      speed_msg.header.stamp = now;
      speed_msg.mode = engaged_;
      speed_msg.speed = desired_speed_last_;
      speed_msg.acceleration_limit = acceleration_limit_;
      speed_msg.deceleration_limit = deceleration_;
      speed_pub.publish(speed_msg);

      steer_msg.header.stamp = now;
      steer_msg.mode = engaged_;
      steer_msg.curvature = desired_curvature_;
      steer_msg.max_curvature_rate = max_curvature_rate_;
      steer_pub.publish(steer_msg);

      turn_signal_command_pub_.publish(turn_signal_command_msg_);

      // Wait for next loop
      loop_rate.sleep();

      // Receive messages
      ros::spinOnce();
    }

    return 0;
  }
};

int main(int argc, char **argv)
{
  JoystickVehicleTestRos ros_module;
  return ros_module.main(argc, argv);
}
