/*
* Unpublished Copyright (c) 2009-2019 AutonomouStuff, LLC, All Rights Reserved.
*
* This file is part of the Joystick Speed and Steering Control ROS 1.0 application which is released under the MIT
* license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/

#include <ros/ros.h>

#include <sensor_msgs/Joy.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <std_msgs/String.h>

#include <automotive_platform_msgs/GearCommand.h>
#include <automotive_platform_msgs/GearFeedback.h>
#include <automotive_platform_msgs/TurnSignalCommand.h>
#include <automotive_platform_msgs/UserInputADAS.h>
#include <automotive_platform_msgs/SpeedMode.h>
#include <automotive_platform_msgs/SteerMode.h>
#include <automotive_platform_msgs/VelocityAccelCov.h>
#include <automotive_navigation_msgs/ModuleState.h>

#include <Json.hpp>
#include <GeneralUtils.hpp>
#include <string>

#include <ssc_joystick/TractorControlMode.h>

using namespace AS;  // NOLINT

double joy_fault_timeout = 0.0;

bool engage_speed_module = 0;
bool engage_steering_module = 0;

int engage1_button = -1;
int engage2_button = -1;
int park_button = -1;
int neutral_button = -1;
int drive_button = -1;
int reverse_button = -1;

int right_turn_button = -1;
int left_turn_button = -1;

int speed_axes = -1;
float speed_up_sign = 0.0;
float speed_step = 0.0;
float speed_max = 0.0;
float acceleration_limit = 0.0;
float deceleration_limit = 0.0;
int brake_axes = -1;
float brake_sign = 0.0;
float max_deceleration_limit = 0.0;

int steer_btn_axes = -1;
int steer_btn_sign = 0;
float steer_btn_step = 0;
int steering_axes = -1;
float steering_sign = 0.0;
float steering_gain = 0.0;
float steering_exponent = 0.0;
float max_curvature_rate = 0.0;

std::string vel_controller_name = "";
std::string vehicle_platform = "Lexus";

bool dbw_ok = false;
uint16_t engaged = 0;
bool engage_pressed = false;
double last_joystick_msg = 0.0;
int speed_last = 0;
float desired_speed = 0.0;
float desired_curvature = 0.0;
bool steering_active_last_loop = false;
int steer_last = 0;
bool brake_inited = false;  // Brake axes default is 0 (50%) until it's pressed
bool brake_active = false;
float deceleration = 0.0;

// Test quick brake mode
bool test_quick_brake = false;
float quick_brake_speed = 0.0;

bool joy_engage = 0;
bool rpm_dial_engage = 0;
bool hydraulics_engage = 0;
uint8_t joy_sens = 0;
float rpm_dial_val = 0.0;
float hyd_in = 0.0;
uint16_t hyd_in_id = 0;
bool beacon_state = 0;
bool horn_state = 0;

uint8_t current_gear = automotive_platform_msgs::Gear::NONE;
float current_velocity = 1.0;

automotive_platform_msgs::GearCommand gear_command_msg;
ros::Publisher gear_command_pub;

automotive_platform_msgs::TurnSignalCommand turn_signal_command_msg;
ros::Publisher turn_signal_command_pub;

bool vehicle_flag;
ros::Publisher tractor_user_pub;

void disengage()
{
  std::cout << "DISENGAGED" << std::endl;
  engaged = 0;
}

void tryToEngage()
{
  if (!dbw_ok)
  {
    std::cout << "Drive by wire system not ready to engage" << std::endl;
  }
  else if ((current_gear != automotive_platform_msgs::Gear::PARK) &&
           (current_gear != automotive_platform_msgs::Gear::NEUTRAL))
  {
    std::cout << "Gear must be in park or neutral to engage" << std::endl;
  }
  else
  {
    std::cout << "ENGAGED" << std::endl;
    desired_speed = 0.0;
    desired_curvature = 0.0;
    gear_command_msg.command.gear = current_gear;
    engaged = 1;
  }
}

// Callback functions
void joystickCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  if ((msg->buttons.at((unsigned int) engage1_button) > 0) &&
      (msg->buttons.at((unsigned int) engage2_button) > 0))
  {
    if (!engage_pressed)
    {
      if (engaged > 0)
      {
        disengage();
      }
      else
      {
        tryToEngage();
      }
      engage_pressed = true;
    }
  }
  else if ((msg->buttons.at((unsigned int) engage1_button) > 0) ||
           (msg->buttons.at((unsigned int) engage2_button) > 0))
  {
    if ((engaged > 0) && !engage_pressed)
    {
      disengage();
      engage_pressed = true;
    }
  }
  else
  {
    engage_pressed = false;
  }

  if (engaged > 0)
  {
    if (msg->buttons.at((unsigned int) park_button) > 0)
    {
      if (current_velocity > 0.1)
      {
        std::cout << "Must be stopped to change to park" << std::endl;
      }
      else
      {
        gear_command_msg.command.gear = automotive_platform_msgs::Gear::PARK;
      }
    }
    else if (msg->buttons.at((unsigned int) neutral_button) > 0)
    {
      gear_command_msg.command.gear = automotive_platform_msgs::Gear::NEUTRAL;
    }
    else if (msg->buttons.at((unsigned int) drive_button) > 0)
    {
      gear_command_msg.command.gear = automotive_platform_msgs::Gear::DRIVE;
    }
    else if (msg->buttons.at((unsigned int) reverse_button) > 0)
    {
      gear_command_msg.command.gear = automotive_platform_msgs::Gear::REVERSE;
    }

    if (msg->buttons.at((unsigned int) right_turn_button) > 0)
    {
      turn_signal_command_msg.turn_signal = automotive_platform_msgs::TurnSignalCommand::RIGHT;
      turn_signal_command_msg.mode = 1;
    }
    else if (msg->buttons.at((unsigned int) left_turn_button) > 0)
    {
      turn_signal_command_msg.turn_signal = automotive_platform_msgs::TurnSignalCommand::LEFT;
      turn_signal_command_msg.mode = 1;
    }
    else
    {
      turn_signal_command_msg.turn_signal = automotive_platform_msgs::TurnSignalCommand::NONE;
      turn_signal_command_msg.mode = 0;
    }

    float speed = msg->axes.at((unsigned int) speed_axes);
    bool speed_updated = false;
    if (speed > 0.1)
    {
      if (speed_last != 1)
      {
        if (!test_quick_brake ||(test_quick_brake && (desired_speed < quick_brake_speed)))
        {
          desired_speed += speed_up_sign * speed_step;
          speed_updated = true;
        }
        else if (test_quick_brake && (desired_speed > quick_brake_speed))
        {
          desired_speed = 0.0;
          deceleration = 0.0;

          std::cout << "Quick Brake Test: Make sure related SSC speed_model.json values are non-zero. " << '\n';
        }
      }
      speed_last = 1;
    }
    else if (speed < -0.1)
    {
      if (speed_last != -1)
      {
        desired_speed -= speed_up_sign * speed_step;
        speed_updated = true;
      }
      speed_last = -1;
    }
    else
    {
      speed_last = 0;
    }

    float brake = msg->axes.at((unsigned int) brake_axes);
    if (brake != 0.0) brake_inited = true;
    if (brake_inited)
    {
      brake *= brake_sign;
      if (brake < 0.95)
      {
        if (!brake_active)
        {
          brake_active = true;
          desired_speed = 0.0;
          speed_updated = true;
        }
        deceleration = static_cast<float>(map2pt(brake, -0.95, 0.95, max_deceleration_limit, deceleration_limit));
      }
      else
      {
        if (brake_active)
        {
          brake_active = false;
          desired_speed = current_velocity / 0.44704f;
          desired_speed = static_cast<float>(speed_step * floor(desired_speed / speed_step));
          speed_updated = true;
          deceleration = deceleration_limit;
        }
      }
    }

    if (speed_updated)
    {
      desired_speed = static_cast<float> (speed_step * round(desired_speed / speed_step));

      if (desired_speed > speed_max)
      {
        desired_speed = speed_max;
      }
      else if (desired_speed < 0.1)
      {
        desired_speed = 0.0;
      }

      std::cout << "Desired Speed: " << desired_speed << std::endl;
    }

    float steering = msg->axes.at((unsigned int) steering_axes);
    if ((steering > 0.01) || (steering < -0.01))
    {
      float raw = steering * steering_sign;
      float raw_sign = 1.0f;
      if (raw < 0.0)
      {
        raw *= -1.0f;
        raw_sign = -1.0f;
      }
      desired_curvature = static_cast<float> (pow(raw, steering_exponent) * steering_gain * raw_sign);
      steering_active_last_loop = true;
    }
    else if (steering_active_last_loop)
    {
      desired_curvature = 0.0;
      steering_active_last_loop = false;
    }
    else
    {
      float steer = msg->axes.at((unsigned int) steer_btn_axes);
      bool steer_updated = false;
      if (steer > 0.1)
      {
        if (steer_last != 1)
        {
          desired_curvature += steer_btn_sign * steer_btn_step;
          steer_updated = true;
        }
        steer_last = 1;
      }
      else if (steer < -0.1)
      {
        if (steer_last != -1)
        {
          desired_curvature -= steer_btn_sign * steer_btn_step;
          steer_updated = true;
        }
        steer_last = -1;
      }
      else
      {
        steer_last = 0;
      }

      if (steer_updated)
      {
        desired_curvature = static_cast<float> (steer_btn_step * round(desired_curvature / steer_btn_step));

        if (desired_curvature > steering_gain)
        {
          desired_curvature = steering_gain;
        }
        else if (desired_curvature < -steering_gain)
        {
          desired_curvature = -steering_gain;
        }

        std::cout << "Desired Steering Curvature: " << desired_curvature << std::endl;
      }
    }
  }
  else
  {
    desired_speed = 0.0;
    desired_curvature = 0.0;
  }
}

void diagnosticCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr& msg)
{
  for (auto it = msg->status.begin(); it < msg->status.end(); it++)
  {
    if (it->name.find("Joystick Driver Status") != std::string::npos)
    {
      last_joystick_msg = msg->header.stamp.toSec();
      if (it->level != diagnostic_msgs::DiagnosticStatus::OK)
      {
        std::cout << "JOYSTICK FAULT" << std::endl;
        engaged = 0;
        brake_inited = false;
        brake_active = false;
      }
    }
  }
}

void gearFeedbackCallback(const automotive_platform_msgs::GearFeedback::ConstPtr& msg)
{
  current_gear = msg->current_gear.gear;
}

void velocityCallback(const automotive_platform_msgs::VelocityAccelCov::ConstPtr& msg)
{
  current_velocity = msg->velocity;
}

void inputAdasCallback(const automotive_platform_msgs::UserInputADAS::ConstPtr& msg)
{
  if (msg->btn_cc_set_inc && msg->btn_acc_gap_inc)
  {
    if (engaged > 0)
    {
      disengage();
    }
  }
  else if (msg->btn_cc_set_dec && msg->btn_acc_gap_dec)
  {
    tryToEngage();
  }
}

void moduleStateCallback(const automotive_navigation_msgs::ModuleState::ConstPtr& msg)
{
  if (msg->name == vel_controller_name)
  {
    if (msg->state == "not_ready")
    {
      dbw_ok = false;
    }
    else if ((msg->state == "ready") || (msg->state == "engaged") || (msg->state == "active"))
    {
      dbw_ok = true;
    }
    else if (msg->state == "failure")
    {
      if (dbw_ok && (engaged > 0))
      {
        std::cout << "Joystick control DISENGAGED due to " << msg->info << std::endl;
        engaged = 0;
      }
      dbw_ok = false;
    }
    else if (msg->state == "fatal")
    {
      if (dbw_ok)
      {
        std::cout << "Joystick control unavailable due to " << msg->info << std::endl;
        std::cout << "Software must be stopped and restarted once the problem is fixed" << std::endl;
        engaged = 0;
      }
      dbw_ok = false;
    }
  }
}

// Main routine
int main(int argc, char **argv)
{
  int c;
  bool exit = false;
  bool config_file_provided = false;
  std::string config_file;

  double publish_interval = 0.0;

  optind = 0;  // Resets the parsing index for getopt.
  opterr = 1;  // Re-enables showing an error for invalid parameters.

  while ((c = getopt(argc, argv, "hf:")) != -1)
  {
    switch (c)
    {
      case 'h':
        std::cout << std::endl;
        std::cout << "Joystick Controller for AutonomouStuff Speed and Steering Control Modules" << std::endl;
        std::cout << "    -h             Show this help menu and exit." << std::endl;
        std::cout
          << "    -f <file.json> The JSON configuration file for all remaining parameters."
          << " See ssc_joystick.json for an example."
          << std::endl;
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

  if (!exit && !config_file_provided)
  {
    std::cout << std::endl;
    std::cout << "Required parameters: " << std::endl;
    std::cout
      << "    -f <file.json>   The JSON configuration file for all required parameters."
      << " See ssc_joystick.json for an example."
      << std::endl;
    std::cout << std::endl;
    exit = true;
  }

  std_msgs::String config_msg;

  if (!exit)
  {
    // Parse JSON configuration parameters
    json json_obj = AS::JSON::deserialize(config_file, &config_msg.data);

    bool config_ok = true;
    std::string mod_name = "ssc joystick";

    config_ok &= AS::readJsonWithLimit(mod_name, json_obj, "publish_interval", ">", 0.0, &publish_interval);

    config_ok &= AS::readJsonWithLimit(mod_name, json_obj, "joy_fault_timeout", ">", 0.0, &joy_fault_timeout);

    config_ok &= AS::readJsonWithError(mod_name, json_obj, "vel_controller_name", &vel_controller_name);
    config_ok &= AS::readJsonWithError(mod_name, json_obj, "vehicle_platform", &vehicle_platform);

    config_ok &= AS::readJsonWithError(mod_name, json_obj, "engage_speed_module", &engage_speed_module);
    config_ok &= AS::readJsonWithError(mod_name, json_obj, "engage_steering_module", &engage_steering_module);

    config_ok &= AS::readJsonWithLimit(mod_name, json_obj, "engage1_button", ">=", 0, &engage1_button);
    config_ok &= AS::readJsonWithLimit(mod_name, json_obj, "engage2_button", ">=", 0, &engage2_button);

    config_ok &= AS::readJsonWithLimit(mod_name, json_obj, "park_button", ">=", 0, &park_button);
    config_ok &= AS::readJsonWithLimit(mod_name, json_obj, "neutral_button", ">=", 0, &neutral_button);
    config_ok &= AS::readJsonWithLimit(mod_name, json_obj, "drive_button", ">=", 0, &drive_button);
    config_ok &= AS::readJsonWithLimit(mod_name, json_obj, "reverse_button", ">=", 0, &reverse_button);

    config_ok &= AS::readJsonWithLimit(mod_name, json_obj, "right_turn_button", ">=", 0, &right_turn_button);
    config_ok &= AS::readJsonWithLimit(mod_name, json_obj, "left_turn_button", ">=", 0, &left_turn_button);

    config_ok &= AS::readJsonWithLimit(mod_name, json_obj, "speed_axes", ">=", 0, &speed_axes);
    config_ok &= AS::readJsonWithError(mod_name, json_obj, "speed_up_sign", &speed_up_sign);
    config_ok &= AS::readJsonWithLimit(mod_name, json_obj, "speed_step", ">", 0.0f, &speed_step);
    config_ok &= AS::readJsonWithLimit(mod_name, json_obj, "speed_max", ">", 0.0f, &speed_max);
    config_ok &= AS::readJsonWithLimit(mod_name, json_obj, "acceleration_limit", ">", 0.0f, &acceleration_limit);
    config_ok &= AS::readJsonWithLimit(mod_name, json_obj, "deceleration_limit", ">", 0.0f, &deceleration_limit);
    config_ok &= AS::readJsonWithLimit(mod_name, json_obj, "brake_axes", ">=", 0, &brake_axes);
    config_ok &= AS::readJsonWithError(mod_name, json_obj, "brake_sign", &brake_sign);
    config_ok &=
      AS::readJsonWithLimit(mod_name, json_obj, "max_deceleration_limit", ">", 0.0f, &max_deceleration_limit);

    config_ok &= AS::readJsonWithLimit(mod_name, json_obj, "steer_btn_axes", ">=", 0, &steer_btn_axes);
    config_ok &= AS::readJsonWithError(mod_name, json_obj, "steer_btn_sign", &steer_btn_sign);
    config_ok &= AS::readJsonWithLimit(mod_name, json_obj, "steer_btn_step", ">", 0.0f, &steer_btn_step);
    config_ok &= AS::readJsonWithLimit(mod_name, json_obj, "steering_axes", ">=", 0, &steering_axes);
    config_ok &= AS::readJsonWithError(mod_name, json_obj, "steering_sign", &steering_sign);
    config_ok &= AS::readJsonWithLimit(mod_name, json_obj, "steering_gain", ">", 0.0f, &steering_gain);
    config_ok &= AS::readJsonWithLimit(mod_name, json_obj, "steering_exponent", ">", 0.0f, &steering_exponent);
    config_ok &= AS::readJsonWithLimit(mod_name, json_obj, "max_curvature_rate", ">", 0.0f, &max_curvature_rate);

    config_ok &= AS::readJsonWithError(mod_name, json_obj, "test_quick_brake", &test_quick_brake);
    config_ok &= AS::readJsonWithError(mod_name, json_obj, "quick_brake_speed", &quick_brake_speed);

    if (vehicle_platform == "hexagon_tractor")
    {
      config_ok &= AS::readJsonWithError(mod_name, json_obj, "joy_engage", &joy_engage);
      config_ok &= AS::readJsonWithError(mod_name, json_obj, "rpm_dial_engage", &rpm_dial_engage);
      config_ok &= AS::readJsonWithError(mod_name, json_obj, "hydraulics_engage", &hydraulics_engage);
      config_ok &= AS::readJsonWithError(mod_name, json_obj, "joy_sens", &joy_sens);
      config_ok &= AS::readJsonWithLimit(mod_name, json_obj, "rpm_dial_val", ">=", 0.0f, &rpm_dial_val);
      config_ok &= AS::readJsonWithLimit(mod_name, json_obj, "hyd_in", ">=", 0.0f, &hyd_in);
      config_ok &= AS::readJsonWithError(mod_name, json_obj, "hyd_in_id", &hyd_in_id);
      config_ok &= AS::readJsonWithError(mod_name, json_obj, "beacon_in", &beacon_state);
      config_ok &= AS::readJsonWithError(mod_name, json_obj, "horn_in", &horn_state);
    }

    if (!config_ok)
    {
      exit = true;
    }
  }

  deceleration = deceleration_limit;

  if (engage_speed_module || engage_steering_module)
  {
    std::cout <<"\nSPEED MODULE MODE: " << engage_speed_module <<"\nSTEERING MODULE MODE: " <<
    engage_steering_module << std::endl;

    if (engage_speed_module && engage_steering_module)
    {
      std::cout <<"\nSPEED AND STEERING CONTROL SET TO ENGAGE" << std::endl;
    }
  }
  else
  {
    std::cout << "\nNO MODULE HAS BEEN SET TO ENGAGE, SSC WILL NOT BE ACTIVE" << std::endl;
  }

  if (vehicle_platform == "hexagon_tractor")
  {
    if (joy_engage)
    {
      std::cout <<"\nUSER JOYSTICK SET TO ENGAGE" << std::endl;
    }
    if (hydraulics_engage)
    {
      std::cout <<"\nUSER HYDRAULICS SET TO ENGAGE" << std::endl;
    }
    if (rpm_dial_engage)
    {
      std::cout <<"\nUSER RPM DIAL SET TO ENGAGE" << std::endl;
    }
  }

  if (exit)
    return 0;

  // ROS initialization
  ros::init(argc, argv, "ssc_joystick");
  ros::NodeHandle n;
  ros::Rate loop_rate(1.0 / publish_interval);

  std::string config_topic = ros::this_node::getName();
  size_t split = config_topic.find_last_of('/');
  if (split != std::string::npos)
  {
    config_topic = config_topic.substr(split + 1);
  }
  config_topic.append("_config");

  // Advertise messages to send
  gear_command_pub = n.advertise<automotive_platform_msgs::GearCommand>("gear_select", 1);
  turn_signal_command_pub = n.advertise<automotive_platform_msgs::TurnSignalCommand>("turn_signal_command", 1);
  ros::Publisher speed_pub = n.advertise<automotive_platform_msgs::SpeedMode>("arbitrated_speed_commands", 1);
  ros::Publisher steer_pub = n.advertise<automotive_platform_msgs::SteerMode>("arbitrated_steering_commands", 1);
  ros::Publisher config_pub = n.advertise<std_msgs::String>(config_topic, 1, true);

  if (vehicle_platform == "hexagon_tractor")
  {
    tractor_user_pub = n.advertise<ssc_joystick::TractorControlMode>("user_control_commands", 1);
    vehicle_flag = true;
  }

  // Subscribe to messages to read
  ros::Subscriber state_sub = n.subscribe("module_states", 10, moduleStateCallback);
  ros::Subscriber joy_sub = n.subscribe("joy", 10, joystickCallback);
  ros::Subscriber joy_fault_sub = n.subscribe("diagnostics", 10, diagnosticCallback);
  ros::Subscriber gear_sub = n.subscribe("gear_feedback", 10, gearFeedbackCallback);
  ros::Subscriber velocity_sub = n.subscribe("velocity_accel_cov", 10, velocityCallback);
  ros::Subscriber adas_input_sub = n.subscribe("adas_input", 10, inputAdasCallback);

  // Wait for time to be valid
  ros::Time::waitForValid();

  // Publish latched message containing the .json config file
  config_pub.publish(config_msg);

  automotive_platform_msgs::SpeedMode speed_msg;
  automotive_platform_msgs::SteerMode steer_msg;

  // Hexagon tractor specific
  ssc_joystick::TractorControlMode tractor_msg;

  // Loop as long as module should run
  while (ros::ok())
  {
    // Get current time
    ros::Time now = ros::Time::now();
    double now_sec = now.toSec();

    if (last_joystick_msg == 0.0)
    {
      // Give joystick node a few seconds to start up
      last_joystick_msg = now_sec + 5.0;
    }
    else if ((now_sec - last_joystick_msg) > joy_fault_timeout)
    {
      // Joystick has timed out
      std::cout << "JOYSTICK TIMEOUT" << std::endl;
      last_joystick_msg = now_sec;
      engaged = 0;
    }

    // Send output messages
    speed_msg.header.stamp = now;
    speed_msg.mode = engage_speed_module > 0 ? engaged : 0;
    speed_msg.speed = desired_speed * 0.44704f;
    speed_msg.acceleration_limit = acceleration_limit;
    speed_msg.deceleration_limit = deceleration;
    speed_pub.publish(speed_msg);

    steer_msg.header.stamp = now;
    steer_msg.mode = engage_steering_module > 0 ? engaged : 0;
    steer_msg.curvature = desired_curvature;
    steer_msg.max_curvature_rate = max_curvature_rate;
    steer_pub.publish(steer_msg);

    gear_command_pub.publish(gear_command_msg);

    turn_signal_command_pub.publish(turn_signal_command_msg);

    if (vehicle_flag)
    {
      tractor_msg.header.stamp = now;
      tractor_msg.joystick_mode = joy_engage > 0 ? engaged : 0;
      tractor_msg.rpm_dial_mode = rpm_dial_engage > 0 ? engaged : 0;
      tractor_msg.hydraulics_mode = hydraulics_engage > 0 ? engaged : 0;
      tractor_msg.joystick_sens = joy_sens;
      tractor_msg.rpm_dial = rpm_dial_val;
      tractor_msg.hydraulics_in = hyd_in;
      tractor_msg.hydraulics_implement_id = hyd_in_id;
      tractor_msg.beacon_state_in = beacon_state;
      tractor_msg.horn_state_in = horn_state;

      tractor_user_pub.publish(tractor_msg);
    }

    // Wait for next loop
    loop_rate.sleep();

    // Receive messages
    ros::spinOnce();
  }
}
