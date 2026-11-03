// Copyright 2026 OM6DOF maintainers.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "om6dof_controllers/gravity_current_budget_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include "hardware_interface/component_parser.hpp"
#include "om6dof_controllers/controller_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "urdf/model.h"

namespace om6dof_controllers
{
using Callback = controller_interface::CallbackReturn;
using Return = controller_interface::return_type;

Callback GravityCurrentBudgetController::on_init()
{
  auto_declare<std::string>("robot_description", "");
  // Raw Goal Current units per Nm, matching the installed XM430 variants.
  // Datasheet-derived estimates, NOT measured torque calibration.
  auto_declare<std::vector<double>>("raw_per_nm",
    {208.541119, 208.541119, 208.541119, 285.006196, 208.541119, 285.006196});
  // Defaults preserve the old gravity-only calculation. The opt-in bringup
  // profile supplies bounded, experimental holding/motion reserves.
  auto_declare<double>("gravity_scale", 1.0);
  auto_declare<std::vector<double>>("holding_current_raw", std::vector<double>(6, 0.0));
  auto_declare<std::vector<double>>("velocity_current_gain", std::vector<double>(6, 0.0));
  auto_declare<std::vector<double>>("acceleration_current_gain", std::vector<double>(6, 0.0));
  auto_declare<std::vector<double>>("max_current_raw", std::vector<double>(6, 32767.0));
  auto_declare<std::vector<double>>("current_slew_raw_per_second", std::vector<double>(6, 1000.0));
  auto_declare<std::string>("reference_state_topic", "/arm_controller/controller_state");
  auto_declare<double>("reference_timeout_seconds", 0.25);
  auto_declare<double>("observer_filter_seconds", 0.10);
  return Callback::SUCCESS;
}

controller_interface::InterfaceConfiguration
GravityCurrentBudgetController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration result;
  result.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= 6; ++i) {result.names.push_back("joint" + std::to_string(i) + "/effort");}
  return result;
}

controller_interface::InterfaceConfiguration
GravityCurrentBudgetController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration result;
  result.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= 6; ++i) {
    result.names.push_back("joint" + std::to_string(i) + "/position");
    result.names.push_back("joint" + std::to_string(i) + "/velocity");
    // Driver exports signed RAW Present Current on arm effort states.
    result.names.push_back("joint" + std::to_string(i) + "/effort");
    result.names.push_back("dxl" + std::to_string(i) + "/stored_current_limit");
  }
  result.names.push_back("gravity_budget/read_ok");
  return result;
}

Callback GravityCurrentBudgetController::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    std::string error;
    const auto xml = fetchRobotDescription(get_node()->get_parameter("robot_description").as_string(),
      get_node()->get_namespace(), "/robot_description", 10.0, error);
    if (xml.empty()) {throw std::runtime_error(error);}
    bool valid = false;
    for (const auto & hw : hardware_interface::parse_control_resources_from_urdf(xml)) {
      if (hw.hardware_class_type != "dynamixel_hardware_interface/DynamixelHardware" ||
        !hw.hardware_parameters.count("gravity_current_budget") ||
        hw.hardware_parameters.at("gravity_current_budget") != "true") {continue;}
      valid = true;
      for (int i = 1; i <= 6; ++i) {
        const auto gpio = std::find_if(hw.gpios.begin(), hw.gpios.end(), [&](const auto & item) {
          return item.name == "dxl" + std::to_string(i);});
        if (gpio == hw.gpios.end() || !gpio->parameters.count("Operating Mode") ||
          gpio->parameters.at("Operating Mode") != "5") {valid = false;}
      }
    }
    if (!valid) {throw std::runtime_error("requires verified Mode-5 gravity_current_budget hardware");}
    scale_ = get_node()->get_parameter("raw_per_nm").as_double_array();
    if (scale_.size() != 6 || std::any_of(scale_.begin(), scale_.end(), [](double v) {
        return !std::isfinite(v) || v <= 0.0;})) {throw std::runtime_error("invalid raw_per_nm");}
    const auto values = [&](const std::string & name, bool positive) {
        auto result = get_node()->get_parameter(name).as_double_array();
        if (result.size() != 6 || std::any_of(result.begin(), result.end(), [&](double v) {
            return !std::isfinite(v) || (positive ? v <= 0.0 : v < 0.0);}))
        {throw std::runtime_error("invalid " + name);}
        return result;
      };
    holding_ = values("holding_current_raw", false);
    velocity_gain_ = values("velocity_current_gain", false);
    acceleration_gain_ = values("acceleration_current_gain", false);
    max_current_ = values("max_current_raw", true);
    slew_ = values("current_slew_raw_per_second", true);
    gravity_scale_ = get_node()->get_parameter("gravity_scale").as_double();
    reference_timeout_ = get_node()->get_parameter("reference_timeout_seconds").as_double();
    observer_filter_ = get_node()->get_parameter("observer_filter_seconds").as_double();
    const auto topic = get_node()->get_parameter("reference_state_topic").as_string();
    if (!std::isfinite(gravity_scale_) || gravity_scale_ < 1.0 || gravity_scale_ > 2.0 ||
      !std::isfinite(reference_timeout_) || reference_timeout_ <= 0.0 || reference_timeout_ > 1.0 ||
      !std::isfinite(observer_filter_) || observer_filter_ < 0.02 || observer_filter_ > 1.0 ||
      topic.empty() || std::any_of(max_current_.begin(), max_current_.end(), [](double v) {
        return v > 32767.0;})) {throw std::runtime_error("invalid budget scale, timeout or ceiling");}
    urdf::Model urdf;
    if (!urdf.initString(xml)) {throw std::runtime_error("invalid URDF");}
    std::vector<std::string> names;
    lower_.clear(); upper_.clear(); effort_.clear();
    for (int i = 1; i <= 6; ++i) {
      names.push_back("joint" + std::to_string(i));
      const auto joint = urdf.getJoint(names.back());
      if (!joint || !joint->limits || joint->type != urdf::Joint::REVOLUTE ||
        !std::isfinite(joint->limits->lower) || !std::isfinite(joint->limits->upper) ||
        joint->limits->lower >= joint->limits->upper ||
        !std::isfinite(joint->limits->effort) || joint->limits->effort <= 0.0)
      {throw std::runtime_error("invalid joint limits");}
      lower_.push_back(joint->limits->lower); upper_.push_back(joint->limits->upper);
      effort_.push_back(joint->limits->effort);
    }
    error = model_.configure(xml, "link1", "end_effector_link", names, {0.0, 0.0, -9.80665});
    if (!error.empty()) {throw std::runtime_error(error);}
    error = GravityModel::readUrdfFriction(xml, names, 0.05, friction_);
    if (!error.empty()) {throw std::runtime_error(error);}
    q_.assign(6, 0.0); g_.assign(6, 0.0);
    qd_filtered_.assign(6, 0.0); qdd_filtered_.assign(6, 0.0); model_torque_.assign(6, 0.0);
    previous_current_.assign(6, 0.0);
    reference_.writeFromNonRT(MotionReference{});
    admittance_reference_.writeFromNonRT(MotionReference{});
    reference_sub_ = get_node()->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
      topic, rclcpp::QoS(1), [this](const control_msgs::msg::JointTrajectoryControllerState & msg) {
        receiveReference(msg);
      });
    admittance_reference_sub_ = get_node()->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
      "/om6dof/admittance/reference", rclcpp::QoS(1),
      [this](const control_msgs::msg::JointTrajectoryControllerState & msg) {receiveReference(msg, true);});
    publisher_ = std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(
      get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("~/goal_current_raw", 10));
    publisher_->msg_.data.resize(6);
    observer_pub_ = std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(
      get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("~/observer_sample", 10));
    observer_pub_->msg_.data.resize(observer_snapshot_.size());
    observer_pub_->msg_.layout.dim.resize(1);
    observer_pub_->msg_.layout.dim[0].label = "v1:stamp,quality,q6,dq6,current_raw6,model_Nm6,ceiling_raw6";
    observer_pub_->msg_.layout.dim[0].size = observer_snapshot_.size();
    observer_pub_->msg_.layout.dim[0].stride = observer_snapshot_.size();
    return Callback::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "%s", e.what());
    return Callback::ERROR;
  }
}

void GravityCurrentBudgetController::receiveReference(
  const control_msgs::msg::JointTrajectoryControllerState & message, bool admittance)
{
  MotionReference result;
  auto & destination = admittance ? admittance_reference_ : reference_;
  const auto & point = message.reference.positions.empty() ? message.desired : message.reference;
  const auto finite = [](const auto & values) {
      return std::all_of(values.begin(), values.end(), [](double v) {return std::isfinite(v);});
    };
  if (message.header.stamp.sec < 0 || message.header.stamp.nanosec >= 1000000000U) {
    destination.writeFromNonRT(result); return;
  }
  const double stamp_age = get_node()->now().seconds() - rclcpp::Time(message.header.stamp).seconds();
  if (message.joint_names.size() == 6 && point.positions.size() == 6 &&
    point.velocities.size() == 6 && (point.accelerations.empty() || point.accelerations.size() == 6) &&
    finite(point.positions) && finite(point.velocities) && finite(point.accelerations) &&
    std::isfinite(stamp_age) && stamp_age >= -0.05 && stamp_age <= reference_timeout_)
  {
    result.valid = true;
    for (size_t i = 0; i < 6; ++i) {
      const auto name = "joint" + std::to_string(i + 1);
      const auto found = std::find(message.joint_names.begin(), message.joint_names.end(), name);
      if (std::count(message.joint_names.begin(), message.joint_names.end(), name) != 1) {
        result.valid = false; break;
      }
      const auto index = static_cast<size_t>(std::distance(message.joint_names.begin(), found));
      result.velocity[i] = point.velocities[index];
      result.acceleration[i] = point.accelerations.empty() ? 0.0 : point.accelerations[index];
    }
    result.received = std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    result.stamp = rclcpp::Time(message.header.stamp).seconds();
  }
  // Malformed/stale messages clear the motion request; output then decays at
  // the configured slew rate. No inference from measured speed or pose error.
  destination.writeFromNonRT(result);
}

void GravityCurrentBudgetController::zero()
{
  for (auto & command : command_interfaces_) {command.set_value(0.0);}
  std::fill(previous_current_.begin(), previous_current_.end(), 0.0);
}

Callback GravityCurrentBudgetController::on_activate(const rclcpp_lifecycle::State &)
{
  zero(); fault_ = true;
  const auto find = [](const auto & interfaces, const std::string & name) {
    for (size_t i = 0; i < interfaces.size(); ++i) {
      if (interfaces[i].get_name() == name) {return i;}
    }
    throw std::runtime_error("missing " + name);
  };
  try {
    positions_.clear(); limits_.clear(); commands_.clear(); velocities_.clear(); measured_currents_.clear();
    for (int i = 1; i <= 6; ++i) {
      positions_.push_back(find(state_interfaces_, "joint" + std::to_string(i) + "/position"));
      limits_.push_back(find(state_interfaces_, "dxl" + std::to_string(i) + "/stored_current_limit"));
      commands_.push_back(find(command_interfaces_, "joint" + std::to_string(i) + "/effort"));
      velocities_.push_back(find(state_interfaces_, "joint" + std::to_string(i) + "/velocity"));
      measured_currents_.push_back(find(state_interfaces_, "joint" + std::to_string(i) + "/effort"));
      const double limit = state_interfaces_[limits_.back()].get_value();
      if (!std::isfinite(limit) || limit < 1.0 || limit > 32767.0) {
        throw std::runtime_error("unverified stored Current Limit");
      }
      RCLCPP_INFO(get_node()->get_logger(),
        "joint%d current cap %.1f raw (stored %.1f, URDF %.1f, configured %.1f)", i,
        std::min({limit, effort_[i - 1] * scale_[i - 1], max_current_[i - 1]}),
        limit, effort_[i - 1] * scale_[i - 1], max_current_[i - 1]);
    }
    health_ = find(state_interfaces_, "gravity_budget/read_ok");
    if (state_interfaces_[health_].get_value() != 1.0) {throw std::runtime_error("bus read not healthy");}
    elapsed_ = 0.0; previous_update_ = std::chrono::steady_clock::now(); fault_ = false;
    // Discard pre-activation trajectory data; a fresh state is required.
    reference_.writeFromNonRT(MotionReference{});
    admittance_reference_.writeFromNonRT(MotionReference{});
    std::fill(qd_filtered_.begin(), qd_filtered_.end(), 0.0);
    std::fill(qdd_filtered_.begin(), qdd_filtered_.end(), 0.0);
    observer_snapshot_.fill(0.0); observer_snapshot_[1] = 1.0;
    return Callback::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "%s", e.what());
    return Callback::ERROR;
  }
}

Callback GravityCurrentBudgetController::on_deactivate(const rclcpp_lifecycle::State &)
{
  zero(); fault_ = true;
  return Callback::SUCCESS;
}

void GravityCurrentBudgetController::publishObserver(const rclcpp::Time & time, double dt, bool healthy)
{
  observer_snapshot_[0] = time.seconds();
  int quality = !healthy ? 2 : elapsed_ < 1.0 ? 1 : 0;
  if (healthy) {
    const double alpha = dt / (observer_filter_ + dt);
    for (size_t i = 0; i < 6; ++i) {
      const double velocity = state_interfaces_[velocities_[i]].get_value();
      const double current = state_interfaces_[measured_currents_[i]].get_value();
      if (!std::isfinite(velocity) || !std::isfinite(current)) {quality = 3; break;}
      const double previous = qd_filtered_[i];
      qd_filtered_[i] += alpha * (velocity - previous);
      qdd_filtered_[i] += alpha * ((qd_filtered_[i] - previous) / dt - qdd_filtered_[i]);
      observer_snapshot_[2 + i] = q_[i];
      observer_snapshot_[8 + i] = velocity;
      observer_snapshot_[14 + i] = current;
      const double ceiling = command_interfaces_[commands_[i]].get_value();
      observer_snapshot_[26 + i] = ceiling;
      if (quality == 0 && (ceiling < 3.0 || std::abs(current) >= std::max(3.0, ceiling - 2.0))) {
        quality = 5;  // Saturation cannot reliably distinguish weak support from contact.
      }
    }
    if (quality != 3) {
      if (!model_.computeDynamicsChecked(q_, qd_filtered_, qdd_filtered_, model_torque_)) {quality = 4;}
      else {
        for (size_t i = 0; i < 6; ++i) {
          observer_snapshot_[20 + i] = model_torque_[i] + GravityModel::friction(qd_filtered_[i], friction_[i]);
          if (!std::isfinite(observer_snapshot_[20 + i])) {quality = 4;}
        }
      }
    }
  }
  observer_snapshot_[1] = quality;
  if (++observer_count_ % 2 == 0 && observer_pub_->trylock()) {
    std::copy(observer_snapshot_.begin(), observer_snapshot_.end(), observer_pub_->msg_.data.begin());
    observer_pub_->unlockAndPublish();
  }
}

Return GravityCurrentBudgetController::update(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  const auto now = std::chrono::steady_clock::now();
  const double dt = period.seconds();
  bool valid = !fault_ && std::isfinite(dt) && dt > 0.0 && dt <= 0.1 &&
    std::chrono::duration<double>(now - previous_update_).count() <= 0.1 &&
    state_interfaces_[health_].get_value() == 1.0;
  previous_update_ = now;
  if (valid) {
    for (size_t i = 0; i < 6; ++i) {
      q_[i] = state_interfaces_[positions_[i]].get_value();
      const double limit = state_interfaces_[limits_[i]].get_value();
      valid = valid && std::isfinite(q_[i]) && q_[i] >= lower_[i] && q_[i] <= upper_[i] &&
        std::isfinite(limit) && limit >= 1.0 && limit <= 32767.0;
    }
    valid = valid && model_.computeChecked(q_, g_);
  }
  if (!valid) {
    zero();
    if (!fault_) {RCLCPP_ERROR(get_node()->get_logger(),
      "Gravity current budget fault latched: zero ceilings; support arm before recovery");}
    fault_ = true;
    publishObserver(time, dt, false);
    if (++publish_count_ % 10 == 0 && publisher_->trylock()) {
      std::fill(publisher_->msg_.data.begin(), publisher_->msg_.data.end(), 0.0);
      publisher_->unlockAndPublish();
    }
    return Return::OK;  // Remain owner of zero commands; never fall back to full current.
  }
  elapsed_ += dt;
  const double ramp = std::min(1.0, elapsed_);  // One-second supported startup, from zero.
  const auto normal_reference = *reference_.readFromRT();
  const auto direct_reference = *admittance_reference_.readFromRT();
  const double steady_seconds = std::chrono::duration<double>(now.time_since_epoch()).count();
  const auto fresh = [&](const MotionReference & reference) {
      const double reference_age = steady_seconds - reference.received;
      const double stamp_age = time.seconds() - reference.stamp;
      return reference.valid && reference_age >= 0.0 && reference_age <= reference_timeout_ &&
             stamp_age >= -0.05 && stamp_age <= reference_timeout_;
    };
  // A delayed zero-speed JTC packet must not erase the direct compliant
  // command's motion reserve. Each source expires independently; only a
  // source with fresh receipt AND source stamp may contribute reserve.
  const auto & reference = fresh(direct_reference) ? direct_reference : normal_reference;
  const bool moving = fresh(reference);
  for (size_t i = 0; i < 6; ++i) {
    const double cap = std::min({state_interfaces_[limits_[i]].get_value(),
      effort_[i] * scale_[i], max_current_[i]});
    const double motion = moving ? velocity_gain_[i] * std::abs(reference.velocity[i]) +
      acceleration_gain_[i] * std::abs(reference.acceleration[i]) : 0.0;
    const double requested = (gravity_scale_ * std::abs(g_[i]) * scale_[i] + holding_[i] + motion) * ramp;
    if (!std::isfinite(requested)) {zero(); fault_ = true; publishObserver(time, dt, false); return Return::OK;}
    // Rate-limit reserve changes in both directions, but apply a reduced
    // absolute ceiling immediately. A fault always bypasses smoothing to zero.
    const double desired = std::min(cap, requested);
    const double smoothed = std::clamp(desired,
      previous_current_[i] - slew_[i] * dt, previous_current_[i] + slew_[i] * dt);
    previous_current_[i] = std::clamp(smoothed, 0.0, cap);
    const double raw = std::floor(previous_current_[i]);
    command_interfaces_[commands_[i]].set_value(raw);
  }
  publishObserver(time, dt, true);
  if (++publish_count_ % 10 == 0 && publisher_->trylock()) {
    for (size_t i = 0; i < 6; ++i) {
      publisher_->msg_.data[i] = command_interfaces_[commands_[i]].get_value();
    }
    publisher_->unlockAndPublish();
  }
  return Return::OK;
}
}  // namespace om6dof_controllers

PLUGINLIB_EXPORT_CLASS(om6dof_controllers::GravityCurrentBudgetController,
  controller_interface::ControllerInterface)
