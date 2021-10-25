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

#ifndef SSC_JOYSTICK__SSC_JOYSTICK_HPP_
#define SSC_JOYSTICK__SSC_JOYSTICK_HPP_

#include <string>

#include <rclcpp/rclcpp.hpp>

#include <automotive_navigation_msgs/msg/module_state.hpp>
#include <automotive_platform_msgs/msg/gear_command.hpp>
#include <automotive_platform_msgs/msg/gear_feedback.hpp>
#include <automotive_platform_msgs/msg/speed_mode.hpp>
#include <automotive_platform_msgs/msg/steer_mode.hpp>
#include <automotive_platform_msgs/msg/turn_signal_command.hpp>
#include <automotive_platform_msgs/msg/user_input_adas.hpp>
#include <automotive_platform_msgs/msg/velocity_accel_cov.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace astuff
{
class SscJoystickNode : public rclcpp::Node
{
public:
  SscJoystickNode();

private:
  // Init
  void loadParams();

  // Subscriber callbacks
  void joystickCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void createEngageCommand(const sensor_msgs::msg::Joy::SharedPtr msg);
  void createShiftCommand(const sensor_msgs::msg::Joy::SharedPtr msg);
  void createSpeedCommand(const sensor_msgs::msg::Joy::SharedPtr msg);
  void createSteeringCommand(const sensor_msgs::msg::Joy::SharedPtr msg);
  void createAuxCommand(const sensor_msgs::msg::Joy::SharedPtr msg);
  void diagnosticCallback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg);
  void moduleStateCallback(const automotive_navigation_msgs::msg::ModuleState::SharedPtr msg);
  void gearFeedbackCallback(const automotive_platform_msgs::msg::GearFeedback::SharedPtr msg);
  void velocityCallback(const automotive_platform_msgs::msg::VelocityAccelCov::SharedPtr msg);
  void inputAdasCallback(const automotive_platform_msgs::msg::UserInputADAS::SharedPtr msg);

  // Publish vehicle command
  void publishVehicleCommand();

  void disengage();
  void tryToEngage();

  // Publishers
  rclcpp::Publisher<automotive_platform_msgs::msg::GearCommand>::SharedPtr gear_cmd_pub_;
  rclcpp::Publisher<automotive_platform_msgs::msg::TurnSignalCommand>::SharedPtr
    turn_signal_cmd_pub_;
  rclcpp::Publisher<automotive_platform_msgs::msg::SpeedMode>::SharedPtr speed_cmd_pub_;
  rclcpp::Publisher<automotive_platform_msgs::msg::SteerMode>::SharedPtr steer_cmd_pub_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr joy_fault_sub_;
  rclcpp::Subscription<automotive_platform_msgs::msg::GearFeedback>::SharedPtr gear_sub_;
  rclcpp::Subscription<automotive_platform_msgs::msg::VelocityAccelCov>::SharedPtr velocity_sub_;
  rclcpp::Subscription<automotive_platform_msgs::msg::UserInputADAS>::SharedPtr adas_input_sub_;
  rclcpp::Subscription<automotive_navigation_msgs::msg::ModuleState>::SharedPtr module_state_sub_;

  // Timers
  rclcpp::TimerBase::SharedPtr vehicle_cmd_timer_;

  bool engaged_ = false;

  // Parameters
  float publish_interval_ = 0.05;
  float joystick_fault_timeout_ = 3.0;

  bool engage_speed_module_ = 0;
  bool engage_steering_module_ = 0;

  int32_t engage1_button_ = -1;
  int32_t engage2_button_ = -1;
  int32_t park_button_ = -1;
  int32_t neutral_button_ = -1;
  int32_t drive_button_ = -1;
  int32_t reverse_button_ = -1;

  int32_t right_turn_button_ = -1;
  int32_t left_turn_button_ = -1;

  int32_t speed_axes_ = -1;
  float speed_up_sign_ = 0.0;
  float speed_step_ = 0.0;
  float max_speed_ = 0.0;
  float acceleration_limit_ = 0.0;
  float deceleration_limit_ = 0.0;
  int32_t brake_axes_ = -1;
  float brake_sign_ = 0.0;
  float max_deceleration_limit_ = 0.0;

  int32_t steer_btn_axes_ = -1;
  int32_t steer_btn_sign_ = 0;
  float steer_btn_step_ = 0;
  int32_t steering_axes_ = -1;
  float steering_sign_ = 0.0;
  float max_curvature_ = 0.0;
  float max_curvature_rate_ = 0.0;
  float steering_exponent_ = 0.0;

  std::string veh_controller_name_ = "";
  std::string vehicle_platform_ = "Lexus";

  bool dbw_ok_ = false;
  bool engage_pressed_ = false;
  double last_joystick_msg_timestamp_ = 0.0;
  int32_t speed_last_ = 0;
  bool steering_active_ = false;
  int32_t steer_last_ = 0;
  bool brake_initialized_ = false;  // Brake axes default is 0 (50%) until it's pressed
  bool brake_active_ = false;
  float deceleration_ = 0.0;

  // Test quick brake mode
  bool test_quick_brake_ = false;
  float quick_brake_speed_ = 0.0;

  uint8_t current_gear_ = automotive_platform_msgs::msg::Gear::NONE;
  float current_velocity_ = 1.0;

  float desired_velocity_ = 0.0;
  float desired_curvature_ = 0.0;
  uint8_t desired_gear_ = automotive_platform_msgs::msg::Gear::NONE;
  uint8_t desired_turn_signal_ = automotive_platform_msgs::msg::TurnSignalCommand::NONE;
};

}  // namespace astuff
#endif  // SSC_JOYSTICK__SSC_JOYSTICK_HPP_
