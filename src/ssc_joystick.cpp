/*
 * AutonomouStuff, LLC ("COMPANY") CONFIDENTIAL
 * Unpublished Copyright (c) 2009-2021 AutonomouStuff, LLC, All Rights Reserved.
 *
 * NOTICE:  All information contained herein is, and remains the property of COMPANY. The intellectual and technical
 * concepts contained herein are proprietary to COMPANY and may be covered by U.S. and Foreign Patents, patents in
 * process, and are protected by trade secret or copyright law. Dissemination of this information or reproduction of
 * this material is strictly forbidden unless prior written permission is obtained from COMPANY.  Access to the source
 * code contained herein is hereby forbidden to anyone except current COMPANY employees, managers or contractors who
 * have executed Confidentiality and Non-disclosure agreements explicitly covering such access.
 *
 * The copyright notice above does not evidence any actual or intended publication or disclosure  of  this source code,
 * which includes information that is confidential and/or proprietary, and is a trade secret, of  COMPANY.   ANY
 * REPRODUCTION, MODIFICATION, DISTRIBUTION, PUBLIC  PERFORMANCE, OR PUBLIC DISPLAY OF OR THROUGH USE  OF THIS  SOURCE
 * CODE  WITHOUT  THE EXPRESS WRITTEN CONSENT OF COMPANY IS STRICTLY PROHIBITED, AND IN VIOLATION OF APPLICABLE LAWS AND
 * INTERNATIONAL TREATIES.  THE RECEIPT OR POSSESSION OF  THIS SOURCE CODE AND/OR RELATED INFORMATION DOES NOT CONVEY OR
 * IMPLY ANY RIGHTS TO REPRODUCE, DISCLOSE OR DISTRIBUTE ITS CONTENTS, OR TO MANUFACTURE, USE, OR SELL ANYTHING THAT IT
 * MAY DESCRIBE, IN WHOLE OR IN PART.
 */

#include "ssc_joystick/ssc_joystick.hpp"

#include <algorithm>
#include <math.h>
#include <string>
#include <utility>

namespace astuff
{
namespace
{
template<typename T>
T clamp(const T value, T bound1, T bound2)
{
  if (bound1 > bound2) {
    std::swap(bound1, bound2);
  }

  if (value < bound1) {
    return bound1;
  } else if (value > bound2) {
    return bound2;
  }
  return value;
}
}  // namespace

using std::placeholders::_1;

SscJoystickNode::SscJoystickNode()
: Node("ssc_joystick")
{
  loadParams();

  // Give joystick a few seconds to start up
  last_joystick_msg_timestamp_ = this->now().seconds() + 5.0;

  // Publishers
  gear_cmd_pub_ = this->create_publisher<automotive_platform_msgs::msg::GearCommand>(
    "gear_select",
    1);
  turn_signal_cmd_pub_ = this->create_publisher<automotive_platform_msgs::msg::TurnSignalCommand>(
    "turn_signal_command", 1);
  speed_cmd_pub_ = this->create_publisher<automotive_platform_msgs::msg::SpeedMode>(
    "arbitrated_speed_commands", 1);
  steer_cmd_pub_ = this->create_publisher<automotive_platform_msgs::msg::SteerMode>(
    "arbitrated_steering_commands", 1);

  // Subscribers
  joy_sub_ =
    this->create_subscription<sensor_msgs::msg::Joy>(
    "joy", 10,
    std::bind(&SscJoystickNode::joystickCallback, this, _1));
  joy_fault_sub_ = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "diagnostics",
    10,
    std::bind(&SscJoystickNode::diagnosticCallback, this, _1));
  gear_sub_ = this->create_subscription<automotive_platform_msgs::msg::GearFeedback>(
    "gear_feedback", 10, std::bind(&SscJoystickNode::gearFeedbackCallback, this, _1));
  velocity_sub_ = this->create_subscription<automotive_platform_msgs::msg::VelocityAccelCov>(
    "velocity_accel_cov", 10, std::bind(&SscJoystickNode::velocityCallback, this, _1));
  adas_input_sub_ = this->create_subscription<automotive_platform_msgs::msg::UserInputADAS>(
    "adas_input", 10, std::bind(&SscJoystickNode::inputAdasCallback, this, _1));
  module_state_sub_ = this->create_subscription<automotive_navigation_msgs::msg::ModuleState>(
    "module_states", 10, std::bind(&SscJoystickNode::moduleStateCallback, this, _1));

  // Vehicle command timer
  vehicle_cmd_timer_ = this->create_wall_timer(
    std::chrono::duration<float>(
      publish_interval_), std::bind(&SscJoystickNode::publishVehicleCommand, this));
  RCLCPP_INFO(this->get_logger(), "ssc_joystick initialized");
}

void SscJoystickNode::loadParams()
{
  publish_interval_ = this->declare_parameter("publish_interval", 0.05f);
  joystick_fault_timeout_ = this->declare_parameter("joystick_fault_timeout", 3.0f);

  veh_controller_name_ = this->declare_parameter<std::string>(
    "veh_controller_name",
    "/ssc/veh_controller");

  engage_speed_module_ = this->declare_parameter("engage_speed_module", true);
  engage_steering_module_ = this->declare_parameter("engage_steering_module", true);
  engage1_button_ = this->declare_parameter("engage1_button", 6);
  engage2_button_ = this->declare_parameter("engage2_button", 7);

  park_button_ = this->declare_parameter("park_button", 3);
  neutral_button_ = this->declare_parameter("neutral_button", 2);
  drive_button_ = this->declare_parameter("drive_button", 0);
  reverse_button_ = this->declare_parameter("reverse_button", 1);
  right_turn_button_ = this->declare_parameter("right_turn_button", 5);
  left_turn_button_ = this->declare_parameter("left_turn_button", 4);

  speed_axes_ = this->declare_parameter("speed_axes", 7);
  speed_up_sign_ = this->declare_parameter("speed_up_sign", 1.0f);

  speed_step_ = this->declare_parameter("speed_step", 1.0f);
  max_speed_ = this->declare_parameter("max_speed", 15.0f);
  acceleration_limit_ = this->declare_parameter("acceleration_limit", 2.0f);
  deceleration_limit_ = this->declare_parameter("deceleration_limit", 2.5f);
  max_deceleration_limit_ = this->declare_parameter("max_deceleration_limit", 4.0f);
  deceleration_ = deceleration_limit_;
  brake_axes_ = this->declare_parameter("brake_axes", 2);
  brake_sign_ = this->declare_parameter("brake_sign", 1.0f);

  steer_btn_axes_ = this->declare_parameter("steer_btn_axes", 6);
  steer_btn_sign_ = this->declare_parameter("steer_btn_sign", 1);
  steer_btn_step_ = this->declare_parameter("steer_btn_step", 0.01f);
  steering_axes_ = this->declare_parameter("steering_axes", 3);
  steering_sign_ = this->declare_parameter("steering_sign", 1.0f);
  steering_exponent_ = this->declare_parameter("steering_exponent", 2.5f);
  max_curvature_ = this->declare_parameter("max_curvature", 0.12f);
  max_curvature_rate_ = this->declare_parameter("max_curvature_rate", 0.10f);

  test_quick_brake_ = this->declare_parameter("test_quick_brake", false);
  quick_brake_speed_ = this->declare_parameter("quick_brake_speed", 0.0f);

  RCLCPP_INFO(this->get_logger(), "Parameters Loaded");

  if (engage_speed_module_ || engage_steering_module_) {
    RCLCPP_INFO(
      this->get_logger(), "SPEED MODULE MODE: %d STEERING MODULE MODE: %d", engage_speed_module_,
      engage_steering_module_);

    if (engage_speed_module_ && engage_steering_module_) {
      RCLCPP_INFO(this->get_logger(), "SPEED AND STEERING CONTROL SET TO ENGAGE");
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "NO MODULE HAS BEEN SET TO ENGAGE, SSC WILL NOT BE ACTIVE");
  }
}

void SscJoystickNode::joystickCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  createEngageCommand(msg);

  if (engaged_) {
    createShiftCommand(msg);
    createSpeedCommand(msg);
    createSteeringCommand(msg);
    createAuxCommand(msg);
  } else {
    desired_velocity_ = 0.0;
    desired_curvature_ = 0.0;
  }
}

void SscJoystickNode::createEngageCommand(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  if ((msg->buttons.at((uint32_t)engage1_button_) > 0) &&
    (msg->buttons.at((uint32_t)engage2_button_) > 0))
  {
    if (!engage_pressed_) {
      if (engaged_) {
        disengage();
      } else {
        tryToEngage();
      }
      engage_pressed_ = true;
    }
  } else if ((msg->buttons.at((uint32_t)engage1_button_) > 0) ||
    (msg->buttons.at((uint32_t)engage2_button_) > 0))
  {
    if (engaged_ && !engage_pressed_) {
      disengage();
      engage_pressed_ = true;
    }
  } else {
    engage_pressed_ = false;
  }
}

void SscJoystickNode::createShiftCommand(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  if (msg->buttons.at((uint32_t)park_button_) > 0) {
    if (current_velocity_ > 0.1) {
      RCLCPP_WARN(this->get_logger(), "Must be stopped to change to park");
    } else {
      desired_gear_ = automotive_platform_msgs::msg::Gear::PARK;
    }
  } else if (msg->buttons.at((uint32_t)neutral_button_) > 0) {
    desired_gear_ = automotive_platform_msgs::msg::Gear::NEUTRAL;
  } else if (msg->buttons.at((uint32_t)drive_button_) > 0) {
    desired_gear_ = automotive_platform_msgs::msg::Gear::DRIVE;
  } else if (msg->buttons.at((uint32_t)reverse_button_) > 0) {
    desired_gear_ = automotive_platform_msgs::msg::Gear::REVERSE;
  }
}

void SscJoystickNode::createSpeedCommand(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  float speed = msg->axes.at((uint32_t)speed_axes_);
  bool speed_updated = false;
  if (speed > 0.1) {
    if (speed_last_ != 1) {
      if (!test_quick_brake_ || (test_quick_brake_ && (desired_velocity_ < quick_brake_speed_))) {
        desired_velocity_ += speed_up_sign_ * speed_step_;
        speed_updated = true;
      } else if (test_quick_brake_ && (desired_velocity_ > quick_brake_speed_)) {
        desired_velocity_ = 0.0;
        deceleration_ = 0.0;
        speed_updated = true;

        RCLCPP_INFO(
          this->get_logger(), "Quick Brake Test: Make sure related SSC values are non-zero.");
      }
    }
    speed_last_ = 1;
  } else if (speed < -0.1) {
    if (speed_last_ != -1) {
      desired_velocity_ -= speed_up_sign_ * speed_step_;
      speed_updated = true;
    }
    speed_last_ = -1;
  } else {
    speed_last_ = 0;
  }

  float brake = msg->axes.at((uint32_t)brake_axes_);
  if (brake != 0.0 || brake_initialized_) {
    brake_initialized_ = true;
    brake *= brake_sign_;
    if (brake < 0.95f) {
      if (!brake_active_) {
        brake_active_ = true;
        desired_velocity_ = 0.0f;
        speed_updated = true;
      }
      auto map2pt = [](float in, float min_in, float max_in, float min_out, float max_out) {  // NOLINT
          float out;
          if (in <= min_in) {
            out = min_out;
          } else if (in >= max_in) {
            out = max_out;
          } else {
            out = (in - min_in) / (max_in - min_in) * (max_out - min_out) + min_out;
          }
          return out;
        };
      deceleration_ = map2pt(brake, -0.95f, 0.95f, max_deceleration_limit_, deceleration_limit_);
    } else {
      if (brake_active_) {
        brake_active_ = false;
        // convert from m/s to mph
        desired_velocity_ = current_velocity_ / 0.44704f;
        desired_velocity_ =
          static_cast<float>(speed_step_ * std::floor(desired_velocity_ / speed_step_));
        speed_updated = true;
        deceleration_ = deceleration_limit_;
      }
    }
  }

  if (speed_updated) {
    desired_velocity_ =
      static_cast<float>(speed_step_ * std::round(desired_velocity_ / speed_step_));

    if (desired_velocity_ > max_speed_) {
      desired_velocity_ = max_speed_;
    } else if (desired_velocity_ < 0.1) {
      desired_velocity_ = 0.0;
    }

    RCLCPP_INFO(this->get_logger(), "Desired velocity: %f", desired_velocity_);
  }
}

void SscJoystickNode::createSteeringCommand(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  float steering = msg->axes.at((uint32_t)steering_axes_);
  if ((steering > 0.01) || (steering < -0.01)) {
    float raw = steering * steering_sign_;
    desired_curvature_ = std::copysign(
      std::pow(
        std::fabs(
          raw), steering_exponent_) * max_curvature_, raw);
    steering_active_ = true;
  } else if (steering_active_) {
    desired_curvature_ = 0.0;
    steering_active_ = false;
  } else {
    float steer = msg->axes.at((uint32_t)steer_btn_axes_);
    bool steer_updated = false;
    if (steer > 0.1) {
      if (steer_last_ != 1) {
        desired_curvature_ += steer_btn_sign_ * steer_btn_step_;
        steer_updated = true;
      }
      steer_last_ = 1;
    } else if (steer < -0.1) {
      if (steer_last_ != -1) {
        desired_curvature_ -= steer_btn_sign_ * steer_btn_step_;
        steer_updated = true;
      }
      steer_last_ = -1;
    } else {
      steer_last_ = 0;
    }

    if (steer_updated) {
      desired_curvature_ =
        static_cast<float>(steer_btn_step_ * round(desired_curvature_ / steer_btn_step_));
      desired_curvature_ = clamp(desired_curvature_, -max_curvature_, max_curvature_);
      RCLCPP_INFO(this->get_logger(), "Desired Curvature: %f", desired_curvature_);
    }
  }
}

void SscJoystickNode::createAuxCommand(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  if (msg->buttons.at((uint32_t)right_turn_button_) > 0) {
    desired_turn_signal_ = automotive_platform_msgs::msg::TurnSignalCommand::RIGHT;
  } else if (msg->buttons.at((uint32_t)left_turn_button_) > 0) {
    desired_turn_signal_ = automotive_platform_msgs::msg::TurnSignalCommand::LEFT;
  } else {
    desired_turn_signal_ = automotive_platform_msgs::msg::TurnSignalCommand::NONE;
  }
}

void SscJoystickNode::diagnosticCallback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
{
  for (auto it = msg->status.begin(); it < msg->status.end(); it++) {
    if (it->name.find("Joystick Driver Status") != std::string::npos) {
      last_joystick_msg_timestamp_ = msg->header.stamp.sec;
      if (it->level != diagnostic_msgs::msg::DiagnosticStatus::OK) {
        RCLCPP_WARN(this->get_logger(), "JOYSTICK FAULT");
        engaged_ = 0;
        brake_initialized_ = false;
        brake_active_ = false;
      }
    }
  }
}

void SscJoystickNode::gearFeedbackCallback(
  const automotive_platform_msgs::msg::GearFeedback::SharedPtr msg)
{
  current_gear_ = msg->current_gear.gear;
}

void SscJoystickNode::velocityCallback(
  const automotive_platform_msgs::msg::VelocityAccelCov::SharedPtr msg)
{
  current_velocity_ = msg->velocity;
}

void SscJoystickNode::inputAdasCallback(
  const automotive_platform_msgs::msg::UserInputADAS::SharedPtr msg)
{
  if (msg->btn_cc_set_inc && msg->btn_acc_gap_inc) {
    if (engaged_ > 0) {
      disengage();
    }
  } else if (msg->btn_cc_set_dec && msg->btn_acc_gap_dec) {
    tryToEngage();
  }
}

void SscJoystickNode::disengage()
{
  RCLCPP_INFO(this->get_logger(), "Disengaged");
  engaged_ = false;
}

void SscJoystickNode::tryToEngage()
{
  if (!dbw_ok_) {
    RCLCPP_INFO(this->get_logger(), "Drive by wire system not ready to engage");
  } else if ((current_gear_ != automotive_platform_msgs::msg::Gear::PARK) &&
    (current_gear_ != automotive_platform_msgs::msg::Gear::NEUTRAL))
  {
    RCLCPP_WARN(this->get_logger(), "Gear must be in park or neutral to engage");
  } else {
    RCLCPP_INFO(this->get_logger(), "Engaged");
    desired_velocity_ = 0.0;
    desired_curvature_ = 0.0;
    desired_gear_ = current_gear_;
    engaged_ = true;
  }
}

void SscJoystickNode::moduleStateCallback(
  const automotive_navigation_msgs::msg::ModuleState::SharedPtr msg)
{
  if (msg->name == veh_controller_name_) {
    if (msg->state == "not_ready") {
      dbw_ok_ = false;
    } else if ((msg->state == "ready") || (msg->state == "engaged") || (msg->state == "active")) {
      dbw_ok_ = true;
    } else if (msg->state == "failure") {
      if (dbw_ok_ && (engaged_ > 0)) {
        RCLCPP_WARN(this->get_logger(), "Joystick control DISENGAGED due to %s", msg->info.c_str());
        engaged_ = 0;
      }
      dbw_ok_ = false;
    } else if (msg->state == "fatal") {
      if (dbw_ok_) {
        RCLCPP_WARN(
          this->get_logger(), "Joystick control unavailable due to %s",
          msg->info.c_str());
        RCLCPP_WARN(
          this->get_logger(), "Software must be stopped and restarted once the problem is fixed");
        engaged_ = 0;
      }
      dbw_ok_ = false;
    }
  }
}

void SscJoystickNode::publishVehicleCommand()
{
  auto current_time = this->now();
  if (current_time.seconds() - last_joystick_msg_timestamp_ > joystick_fault_timeout_) {
    // Joystick has timed out
    RCLCPP_WARN(this->get_logger(), "JOYSTICK TIMEOUT");
    last_joystick_msg_timestamp_ = current_time.seconds();
    engaged_ = 0;
  }

  automotive_platform_msgs::msg::SpeedMode speed_cmd_msg;
  speed_cmd_msg.header.stamp = current_time;
  speed_cmd_msg.mode = engage_speed_module_ ? engaged_ : false;
  speed_cmd_msg.speed = desired_velocity_ * 0.44704f;
  speed_cmd_msg.acceleration_limit = acceleration_limit_;
  speed_cmd_msg.deceleration_limit = deceleration_;
  speed_cmd_pub_->publish(speed_cmd_msg);

  automotive_platform_msgs::msg::SteerMode steer_cmd_msg;
  steer_cmd_msg.header.stamp = current_time;
  steer_cmd_msg.mode = engage_steering_module_ ? engaged_ : false;
  steer_cmd_msg.curvature = desired_curvature_;
  steer_cmd_msg.max_curvature_rate = max_curvature_rate_;
  steer_cmd_pub_->publish(steer_cmd_msg);

  automotive_platform_msgs::msg::GearCommand gear_cmd_msg;
  gear_cmd_msg.command.gear = desired_gear_;
  gear_cmd_pub_->publish(gear_cmd_msg);

  automotive_platform_msgs::msg::TurnSignalCommand turn_signal_cmd_msg;
  if (desired_turn_signal_ == automotive_platform_msgs::msg::TurnSignalCommand::LEFT ||
    desired_turn_signal_ == automotive_platform_msgs::msg::TurnSignalCommand::RIGHT)
  {
    turn_signal_cmd_msg.mode = 1;
  } else {
    turn_signal_cmd_msg.mode = 0;
  }
  turn_signal_cmd_msg.turn_signal = desired_turn_signal_;
  turn_signal_cmd_pub_->publish(turn_signal_cmd_msg);
}
}  // namespace astuff
