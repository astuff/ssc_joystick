/*
* AutonomouStuff, LLC ("COMPANY") CONFIDENTIAL
* Unpublished Copyright (c) 2009-2017 AutonomouStuff, LLC, All Rights Reserved.
*
* NOTICE:  All information contained herein is, and remains the property of COMPANY. The intellectual and technical concepts contained
* herein are proprietary to COMPANY and may be covered by U.S. and Foreign Patents, patents in process, and are protected by trade secret or copyright law.
* Dissemination of this information or reproduction of this material is strictly forbidden unless prior written permission is obtained
* from COMPANY.  Access to the source code contained herein is hereby forbidden to anyone except current COMPANY employees, managers or contractors who have executed
* Confidentiality and Non-disclosure agreements explicitly covering such access.
*
* The copyright notice above does not evidence any actual or intended publication or disclosure  of  this source code, which includes
* information that is confidential and/or proprietary, and is a trade secret, of  COMPANY.   ANY REPRODUCTION, MODIFICATION, DISTRIBUTION, PUBLIC  PERFORMANCE,
* OR PUBLIC DISPLAY OF OR THROUGH USE  OF THIS  SOURCE CODE  WITHOUT  THE EXPRESS WRITTEN CONSENT OF COMPANY IS STRICTLY PROHIBITED, AND IN VIOLATION OF APPLICABLE
* LAWS AND INTERNATIONAL TREATIES.  THE RECEIPT OR POSSESSION OF  THIS SOURCE CODE AND/OR RELATED INFORMATION DOES NOT CONVEY OR IMPLY ANY RIGHTS
* TO REPRODUCE, DISCLOSE OR DISTRIBUTE ITS CONTENTS, OR TO MANUFACTURE, USE, OR SELL ANYTHING THAT IT  MAY DESCRIBE, IN WHOLE OR IN PART.
*/

#include <ros/ros.h>

#include <sensor_msgs/Joy.h>
#include <diagnostic_msgs/DiagnosticArray.h>

#include <automation_msgs/ModuleState.h>

#include <automation_msgs/GearCommand.h>
#include <automation_msgs/TurnSignalCommand.h>
#include <automation_msgs/ModuleState.h>
#include <highway_msgs/SpeedMode.h>
#include <highway_msgs/SteerMode.h>

#include <dbw_mkz_msgs/Misc1Report.h>

#include <Json.hpp>

using namespace std;
using namespace AS;

double joy_fault_timeout = 0.0;

int engage_button = -1;
int disengage_button = -1;

int park_button = -1;
int neutral_button = -1;
int drive_button = -1;

int right_turn_button = -1;
int left_turn_button = -1;

int speed_axes = -1;
float speed_up_sign = 0.0;
float speed_step = 0.0;
float speed_max = 0.0;
float acceleration_limit = 0.0;
float deceleration_limit = 0.0;

int steering_axes = -1;
float steering_sign = 0.0;
float steering_gain = 0.0;
float steering_exponent = 0.0;
float max_curvature_rate = 0.0;

string vel_controller_name = "";

bool dbw_ok = false;
uint16_t engaged = 0;
double last_joystick_msg = 0.0;
int speed_last = 0;
float desired_speed = 0.0;
float desired_curvature = 0.0;

automation_msgs::GearCommand gear_command_msg;
ros::Publisher gear_command_pub;

automation_msgs::TurnSignalCommand turn_signal_command_msg;
ros::Publisher turn_signal_command_pub;

// Callback functions
void joystickCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  if (msg->buttons.at((unsigned int) disengage_button) > 0)
  {
    if (engaged > 0)
    {
      cout << "DISENGAGED" << endl;
    }
    engaged = 0;
  }
  else if (msg->buttons.at((unsigned int) engage_button) > 0)
  {
    if (!dbw_ok)
    {
      cout << "System not ready to engage" << endl;
    }
    else if (engaged == 0)
    {
      cout << "ENGAGED" << endl;
      engaged = 1;
    }
  }

  if (engaged > 0)
  {
    if (msg->buttons.at((unsigned int) park_button) > 0)
    {
      gear_command_msg.command.gear = automation_msgs::Gear::PARK;
    }
    else if (msg->buttons.at((unsigned int) neutral_button) > 0)
    {
      gear_command_msg.command.gear = automation_msgs::Gear::NEUTRAL;
    }
    else if (msg->buttons.at((unsigned int) drive_button) > 0)
    {
      gear_command_msg.command.gear = automation_msgs::Gear::DRIVE;
    }
    gear_command_pub.publish(gear_command_msg);

    if (msg->buttons.at((unsigned int) right_turn_button) > 0)
    {
      turn_signal_command_msg.turn_signal = automation_msgs::TurnSignalCommand::RIGHT;
      turn_signal_command_msg.mode = 1;
    }
    else if (msg->buttons.at((unsigned int) left_turn_button) > 0)
    {
      turn_signal_command_msg.turn_signal = automation_msgs::TurnSignalCommand::LEFT;
      turn_signal_command_msg.mode = 1;
    }
    else
    {
      turn_signal_command_msg.turn_signal = automation_msgs::TurnSignalCommand::NONE;
      turn_signal_command_msg.mode = 0;
    }

    float speed = msg->axes.at((unsigned int) speed_axes);
    bool speed_updated = false;
    if (speed > 0.1)
    {
      if (speed_last != 1)
      {
        desired_speed += speed_up_sign * speed_step;
        speed_updated = true;
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

    if (speed_updated)
    {
      if (desired_speed > speed_max)
      {
        desired_speed = speed_max;
      }
      else if (desired_speed < 0.1)
      {
        desired_speed = 0.0;
      }

      cout << "Desired Speed: " << desired_speed << endl;
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
      desired_curvature = pow(raw, steering_exponent) * steering_gain * raw_sign;
    }
    else
    {
      desired_curvature = 0.0;
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
    if (it->name.find("Joystick Driver Status") != string::npos)
    {
      last_joystick_msg = msg->header.stamp.toSec();
      if (it->level != diagnostic_msgs::DiagnosticStatus::OK)
      {
        cout << "JOYSTICK FAULT" << endl;
        engaged = 0;
      }
    }
  }
}

void misc1Callback(const dbw_mkz_msgs::Misc1Report::ConstPtr& msg)
{
  if (msg->btn_cc_set_inc && msg->btn_cc_gap_inc)
  {
    if (engaged > 0)
    {
      cout << "DISENGAGED" << endl;
      desired_speed = 0.0;
      desired_curvature = 0.0;
    }
    engaged = 0;
  }
  else if (msg->btn_cc_set_dec && msg->btn_cc_gap_dec)
  {
    if (!dbw_ok)
    {
      cout << "System not ready to engage" << endl;
    }
    else if (engaged == 0)
    {
      cout << "ENGAGED" << endl;
      engaged = 1;
      desired_speed = 0.0;
      desired_curvature = 0.0;
    }
  }
}

void moduleStateCallback(const automation_msgs::ModuleState::ConstPtr& msg)
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
        cout << "Joystick control DISENGAGED due to " << msg->info << endl;
        engaged = 0;
      }
      dbw_ok = false;
    }
    else if (msg->state == "fatal")
    {
      if (dbw_ok)
      {
        cout << "Joystick control unavailable due to " << msg->info << endl;
        cout << "Software must be stopped and restarted once the problem is fixed" << endl;
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
  string config_file;

  double publish_interval = 0.0;

  optind = 0; //Resets the parsing index for getopt.
  opterr = 1; //Re-enables showing an error for invalid parameters.

  while ((c = getopt(argc, argv, "hf:")) != -1)
  {
    switch (c)
    {
      case 'h':
        cout << endl;
        cout << "Joystick testing for AutonomouStuff Vehicle Control Modules" << endl;
        cout << "    -h             Show this help menu and exit." << endl;
        cout
          << "    -f <file.json> The JSON configuration file for all remaining parameters. See joystick_vehicle_test.json for an example."
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
      << "    -f <file.json>   The JSON configuration file for all required parameters. See joystick_vehicle_test.json for an example."
      << endl;
    cout << endl;
    exit = true;
  }

  if (!exit)
  {
    //Parse JSON configuration parameters
    json json_obj = JSON::deserialize(config_file);
    try
    {
      publish_interval = json_obj["publish_interval"];

      joy_fault_timeout = json_obj["joy_fault_timeout"];

      vel_controller_name = json_obj["vel_controller_name"];

      engage_button = json_obj["engage_button"];
      disengage_button = json_obj["disengage_button"];

      park_button = json_obj["park_button"];
      neutral_button = json_obj["neutral_button"];
      drive_button = json_obj["drive_button"];

      right_turn_button = json_obj["right_turn_button"];
      left_turn_button = json_obj["left_turn_button"];

      speed_axes = json_obj["speed_axes"];
      speed_up_sign = json_obj["speed_up_sign"];
      speed_step = json_obj["speed_step"];
      speed_max = json_obj["speed_max"];
      acceleration_limit = json_obj["acceleration_limit"];
      deceleration_limit = json_obj["deceleration_limit"];

      steering_axes = json_obj["steering_axes"];
      steering_sign = json_obj["steering_sign"];
      steering_gain = json_obj["steering_gain"];
      steering_exponent = json_obj["steering_exponent"];
      max_curvature_rate = json_obj["max_curvature_rate"];
    }
    catch (const std::exception &e)
    {
      exit = true;
    }

    if (publish_interval <= 0.0 ||

        joy_fault_timeout <= 0 ||

        vel_controller_name.empty() ||

        engage_button < 0 ||
        disengage_button < 0 ||

        park_button < 0 ||
        neutral_button < 0 ||
        drive_button < 0 ||

        right_turn_button < 0 ||
        left_turn_button < 0 ||

        speed_axes < 0 ||
        speed_up_sign == 0 ||
        speed_step == 0 ||
        speed_max == 0 ||
        acceleration_limit == 0 ||
        deceleration_limit == 0 ||

        steering_axes < 0 ||
        steering_sign == 0 ||
        steering_gain == 0 ||
        steering_exponent == 0 ||
        max_curvature_rate == 0)
    {
      cout << endl;
      cerr << "The required parameters were not found in the provided JSON file." << endl;
      cout << endl;
      exit = true;
    }
  }

  if (exit)
    return 0;

  // ROS initialization
  ros::init(argc, argv, "joystick_vehicle_test");
  ros::NodeHandle n;
  ros::Rate loop_rate(1.0 / publish_interval);

  // Advertise messages to send
  gear_command_pub = n.advertise<automation_msgs::GearCommand>("gear_select", 1);
  turn_signal_command_pub = n.advertise<automation_msgs::TurnSignalCommand>("turn_signal_command", 1);
  ros::Publisher speed_pub = n.advertise<highway_msgs::SpeedMode>("arbitrated_speed_commands", 1);
  ros::Publisher steer_pub = n.advertise<highway_msgs::SteerMode>("arbitrated_steering_commands", 1);

  // Subscribe to messages to read
  ros::Subscriber state_sub = n.subscribe("module_states", 5, moduleStateCallback);
  ros::Subscriber joy_sub = n.subscribe("joy", 5, joystickCallback);
  ros::Subscriber joy_fault_sub = n.subscribe("diagnostics", 1, diagnosticCallback);
  ros::Subscriber misc1_sub = n.subscribe("misc_1_report", 1, misc1Callback);

  // Wait for time to be valid
  while (ros::Time::now().nsec == 0);

  highway_msgs::SpeedMode speed_msg;
  highway_msgs::SteerMode steer_msg;

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
      cout << "JOYSTICK TIMEOUT" << endl;
      last_joystick_msg = now_sec;
      engaged = 0;
    }

    // Send output messages
    speed_msg.header.stamp = now;
    speed_msg.mode = engaged;
    speed_msg.speed = desired_speed * 0.44704f;
    speed_msg.acceleration_limit = acceleration_limit;
    speed_msg.deceleration_limit = deceleration_limit;
    speed_pub.publish(speed_msg);

    steer_msg.header.stamp = now;
    steer_msg.mode = engaged;
    steer_msg.curvature = desired_curvature;
    steer_msg.max_curvature_rate = max_curvature_rate;
    steer_pub.publish(steer_msg);

    turn_signal_command_pub.publish(turn_signal_command_msg);

    // Wait for next loop
    loop_rate.sleep();

    // Receive messages
    ros::spinOnce();
  }
}
