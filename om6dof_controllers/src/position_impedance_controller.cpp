// Copyright 2026 OM6DOF maintainers. Licensed under Apache-2.0.
#include "om6dof_controllers/position_impedance_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <set>
#include <stdexcept>
#include "hardware_interface/component_parser.hpp"
#include "om6dof_controllers/controller_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "urdf/model.h"

namespace om6dof_controllers
{
namespace
{
bool positive(double x) {return std::isfinite(x) && x > 0.0;}
bool nonnegative(double x) {return std::isfinite(x) && x >= 0.0;}
}

double impedanceBudget(double g, double base, double ratio, double extra_max, double total_max)
{
  if (!std::isfinite(g) || !nonnegative(base) || !nonnegative(ratio) ||
    !nonnegative(extra_max) || !positive(total_max)) {return 0.0;}
  return std::max(0.0, std::min({extra_max, base + ratio * std::abs(g),
    total_max - std::abs(g)}));
}

int64_t PositionImpedanceController::steadyNow()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

controller_interface::CallbackReturn PositionImpedanceController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", {});
    for (const auto * name : {"stiffness", "damping", "effort_scale", "max_effort",
      "extra_torque_base", "extra_torque_ratio", "extra_torque_max", "reference_velocity",
      "max_reference_error", "residual_torque_rate", "max_velocity"})
    {auto_declare<std::vector<double>>(name, {});}
    auto_declare<bool>("enable_on_activation", false);
    auto_declare<std::vector<double>>("activation_pose", {});
    auto_declare<bool>("require_activation_pose", false);
    auto_declare<int>("supported_pose_protocol_version", 1);
    auto_declare<double>("activation_pose_tolerance", 0.03);
    auto_declare<bool>("allow_mock_hardware", false);
    auto_declare<bool>("require_bus_health", true);
    auto_declare<bool>("friction_compensation", false);
    auto_declare<double>("friction_deadzone", 0.05);
    // A commanded wrench is opt-in.  It is expressed in the KDL chain base
    // frame as [Fx, Fy, Fz, Mx, My, Mz] (N, N, N, Nm, Nm, Nm).
    auto_declare<bool>("enable_cartesian_wrench", false);
    auto_declare<std::vector<double>>("max_cartesian_wrench", {0, 0, 0, 0, 0, 0});
    auto_declare<double>("command_timeout", 0.25);
    auto_declare<double>("health_timeout", 0.15);
    auto_declare<double>("health_clean_window", 1.0);
    auto_declare<double>("max_update_period", 0.05);
    auto_declare<double>("joint_limit_margin", 0.05);
    auto_declare<double>("ramp_seconds", 1.0);
    auto_declare<std::string>("base_link", "link1");
    auto_declare<std::string>("tip_link", "end_effector_link");
    auto_declare<std::vector<double>>("gravity", {0.0, 0.0, -9.80665});
    auto_declare<std::string>("robot_description", "");
    auto_declare<std::string>("robot_description_topic", "/robot_description");
    auto_declare<double>("robot_description_timeout", 10.0);
    auto_declare<std::string>("health_topic", "/dynamixel_hardware_interface/health");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "%s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
PositionImpedanceController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration c;
  c.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & j : joints_) {c.names.push_back(j + "/effort");}
  return c;
}

controller_interface::InterfaceConfiguration
PositionImpedanceController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration c;
  c.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & j : joints_) {
    c.names.push_back(j + "/position"); c.names.push_back(j + "/velocity");
  }
  return c;
}

controller_interface::CallbackReturn PositionImpedanceController::on_configure(
  const rclcpp_lifecycle::State &)
{
  try {
    auto node = get_node();
    joints_ = node->get_parameter("joints").as_string_array();
    const size_t n = joints_.size();
    if (!n || std::set<std::string>(joints_.begin(), joints_.end()).size() != n) {
      throw std::runtime_error("joints must be nonempty and unique");
    }
    std::string error;
    const auto read = [&](const char * name, std::vector<double> & out, bool zero_allowed) {
        const auto raw = node->get_parameter(name).as_double_array();
        if (raw.empty() || !perJointParameter(name, raw, n, 0.0, out, error) ||
          !std::all_of(out.begin(), out.end(), zero_allowed ? nonnegative : positive))
        {throw std::runtime_error(std::string("invalid/unspecified parameter: ") + name);}
      };
    read("stiffness", stiffness_, true); read("damping", damping_, true);
    read("effort_scale", scale_, false); read("max_effort", max_raw_, false);
    read("extra_torque_base", base_, true); read("extra_torque_ratio", ratio_, true);
    read("extra_torque_max", extra_max_, true); read("reference_velocity", reference_rate_, false);
    read("max_reference_error", lookahead_, false); read("residual_torque_rate", torque_rate_, false);
    read("max_velocity", velocity_limit_, false);
    activation_pose_ = node->get_parameter("activation_pose").as_double_array();
    require_activation_pose_ = node->get_parameter("require_activation_pose").as_bool();
    activation_pose_tolerance_ = node->get_parameter("activation_pose_tolerance").as_double();
    if ((require_activation_pose_ && activation_pose_.empty()) ||
      !positive(activation_pose_tolerance_) || (!activation_pose_.empty() &&
      (activation_pose_.size() != n || !std::all_of(activation_pose_.begin(), activation_pose_.end(),
        [](double q) {return std::isfinite(q);}))))
    {throw std::runtime_error("invalid activation pose gate");}
    timeout_ = node->get_parameter("command_timeout").as_double();
    health_timeout_ = node->get_parameter("health_timeout").as_double();
    clean_window_ = node->get_parameter("health_clean_window").as_double();
    max_period_ = node->get_parameter("max_update_period").as_double();
    margin_ = node->get_parameter("joint_limit_margin").as_double();
    ramp_seconds_ = node->get_parameter("ramp_seconds").as_double();
    if (!positive(timeout_) || !positive(health_timeout_) || !positive(clean_window_) ||
      !positive(max_period_) || max_period_ > 0.1 || !positive(margin_) ||
      !positive(ramp_seconds_)) {throw std::runtime_error("invalid timeout, margin, or ramp");}
    require_health_ = node->get_parameter("require_bus_health").as_bool();
    use_friction_ = node->get_parameter("friction_compensation").as_bool();
    cartesian_wrench_enabled_ = node->get_parameter("enable_cartesian_wrench").as_bool();
    const auto max_wrench = node->get_parameter("max_cartesian_wrench").as_double_array();
    if (max_wrench.size() != max_cartesian_wrench_.size() ||
      !std::all_of(max_wrench.begin(), max_wrench.end(), nonnegative))
    {throw std::runtime_error("max_cartesian_wrench needs six finite nonnegative values");}
    std::copy(max_wrench.begin(), max_wrench.end(), max_cartesian_wrench_.begin());
    if (cartesian_wrench_enabled_ && std::none_of(max_wrench.begin(), max_wrench.end(), positive)) {
      throw std::runtime_error("enabled Cartesian wrench needs at least one positive limit");
    }
    const auto xml = fetchRobotDescription(node->get_parameter("robot_description").as_string(),
      node->get_namespace(), node->get_parameter("robot_description_topic").as_string(),
      node->get_parameter("robot_description_timeout").as_double(), error);
    if (xml.empty()) {throw std::runtime_error(error);}

    // Refuse position/current-limit hardware. This checks the declared hardware
    // contract, not a servo register readback. The driver remains responsible
    // for applying/verifying mode 0 and its identity motor mapping.
    bool valid_hardware = false;
    for (const auto & hw : hardware_interface::parse_control_resources_from_urdf(xml)) {
      bool all_joints = true;
      for (const auto & j : joints_) {
        const auto found = std::find_if(hw.joints.begin(), hw.joints.end(),
          [&](const auto & item) {return item.name == j;});
        all_joints = all_joints && found != hw.joints.end();
        if (found != hw.joints.end()) {
          all_joints = all_joints && std::any_of(found->command_interfaces.begin(),
            found->command_interfaces.end(), [](const auto & item) {return item.name == "effort";});
        }
      }
      if (!all_joints) {continue;}
      if (hw.hardware_class_type == "mock_components/GenericSystem" &&
        node->get_parameter("allow_mock_hardware").as_bool()) {valid_hardware = true; continue;}
      if (hw.hardware_class_type != "dynamixel_hardware_interface/DynamixelHardware" ||
        !require_health_ || n != 6) {continue;}
      bool modes_ok = true;
      for (unsigned i = 1; i <= 6; ++i) {
        if (std::find(joints_.begin(), joints_.end(), "joint" + std::to_string(i)) == joints_.end()) {
          modes_ok = false;
        }
        const auto motor = std::find_if(hw.gpios.begin(), hw.gpios.end(),
          [&](const auto & item) {return item.name == "dxl" + std::to_string(i);});
        if (motor == hw.gpios.end() || !motor->parameters.count("Operating Mode") ||
          motor->parameters.at("Operating Mode") != "0") {modes_ok = false;}
      }
      valid_hardware = modes_ok;
    }
    if (!valid_hardware) {throw std::runtime_error("requires OM6DOF mode 0 with health, or explicit mock hardware");}
    urdf::Model urdf;
    if (!urdf.initString(xml)) {throw std::runtime_error("invalid URDF");}
    lower_.resize(n); upper_.resize(n); total_max_.resize(n);
    for (size_t i = 0; i < n; ++i) {
      const auto joint = urdf.getJoint(joints_[i]);
      if (!joint || joint->type != urdf::Joint::REVOLUTE || !joint->limits || joint->mimic) {
        throw std::runtime_error("requires bounded, non-mimic revolute joints");
      }
      const auto & limits = *joint->limits;
      lower_[i] = limits.lower + margin_; upper_[i] = limits.upper - margin_;
      total_max_[i] = std::min(max_raw_[i] / scale_[i], limits.effort);
      if (!std::isfinite(lower_[i]) || !std::isfinite(upper_[i]) || lower_[i] >= upper_[i] ||
        !positive(limits.effort) || !positive(total_max_[i]) || !positive(limits.velocity) ||
        velocity_limit_[i] > limits.velocity || reference_rate_[i] > velocity_limit_[i])
      {throw std::runtime_error("invalid URDF limits or requested velocity exceeds URDF");}
    }
    const auto gravity = node->get_parameter("gravity").as_double_array();
    if (gravity.size() != 3 || !std::all_of(gravity.begin(), gravity.end(),
      [](double x) {return std::isfinite(x);})) {throw std::runtime_error("invalid gravity vector");}
    error = model_.configure(xml, node->get_parameter("base_link").as_string(),
      node->get_parameter("tip_link").as_string(), joints_, {gravity[0], gravity[1], gravity[2]});
    if (!error.empty()) {throw std::runtime_error(error);}
    if (model_.chain_joint_names().size() != n) {throw std::runtime_error("all controlled joints must belong to gravity chain");}
    error = GravityModel::readUrdfFriction(xml, joints_,
      node->get_parameter("friction_deadzone").as_double(), friction_);
    if (!error.empty()) {throw std::runtime_error(error);}
    for (auto * values : {&q_, &v_, &gravity_, &reference_, &target_, &residual_, &budget_, &raw_, &idle_damping_, &joint_wrench_}) {
      values->assign(n, 0.0);
    }
    jacobian_.assign(6 * n, 0.0);
    goal_buffer_.writeFromNonRT(Goal{}); wrench_buffer_.writeFromNonRT(Wrench{});
    health_buffer_.writeFromNonRT(Health{});
    callback_health_ = Health{}; health_instance_.clear();
    last_goal_stamp_ns_ = 0; last_health_stamp_ns_ = 0;
    goal_sub_ = node->create_subscription<sensor_msgs::msg::JointState>(
      "~/target", rclcpp::QoS(1).reliable().durability_volatile(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {receiveGoal(*msg);});
    wrench_sub_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
      "~/wrench", rclcpp::QoS(1).reliable().durability_volatile(),
      [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) {receiveWrench(*msg);});
    health_sub_ = node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      node->get_parameter("health_topic").as_string(), rclcpp::QoS(1).reliable().transient_local(),
      [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg) {receiveHealth(*msg);});
    telemetry_ = std::make_unique<Publisher>(node->create_publisher<std_msgs::msg::Float64MultiArray>(
      "~/state", rclcpp::QoS(1)));
    telemetry_->msg_.data.resize(2 + 7 * n);
    limit_telemetry_ = std::make_unique<Publisher>(node->create_publisher<std_msgs::msg::Float64MultiArray>(
      "~/limit_event", rclcpp::QoS(1)));
    limit_telemetry_->msg_.data.resize(limit_event_.size() + 1);
    // Never acknowledge gain/model/limit changes that the configured RT loop
    // would not use. Reconfigure from YAML with the arm supported instead.
    activation_attempted_.store(false);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "position impedance: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

void PositionImpedanceController::receiveGoal(const sensor_msgs::msg::JointState & msg)
{
  if (!active_.load()) {return;}
  const uint64_t epoch = epoch_.load();
  try {
    const int64_t stamp = rclcpp::Time(msg.header.stamp).nanoseconds();
    const double age = (get_node()->now().nanoseconds() - stamp) * 1e-9;
    if (stamp <= last_goal_stamp_ns_ || age < 0.0 || age > timeout_ ||
      msg.name.size() != joints_.size() || msg.position.size() != joints_.size() ||
      !msg.velocity.empty() || !msg.effort.empty()) {throw std::runtime_error("invalid target");}
    Goal goal; goal.q.resize(joints_.size()); goal.epoch = epoch;
    std::set<std::string> names(msg.name.begin(), msg.name.end());
    if (names.size() != joints_.size()) {throw std::runtime_error("duplicate target joint");}
    for (size_t i = 0; i < joints_.size(); ++i) {
      const auto found = std::find(msg.name.begin(), msg.name.end(), joints_[i]);
      if (found == msg.name.end()) {throw std::runtime_error("missing target joint");}
      const double q = msg.position[std::distance(msg.name.begin(), found)];
      if (!std::isfinite(q) || q < lower_[i] || q > upper_[i]) {throw std::runtime_error("target outside soft bounds");}
      goal.q[i] = q;
    }
    last_goal_stamp_ns_ = stamp;
    // Source age consumes the watchdog budget too; a delayed fresh delivery
    // cannot renew an old target for another full timeout.
    goal.received_ns = steadyNow() - static_cast<int64_t>(age * 1e9);
    goal.sequence = command_sequence_.fetch_add(1) + 1;
    goal_buffer_.writeFromNonRT(goal);
  } catch (const std::exception &) {
    invalid_goals_.fetch_add(1);
  }
}

void PositionImpedanceController::receiveWrench(const std_msgs::msg::Float64MultiArray & msg)
{
  if (!active_.load()) {return;}
  try {
    if (!cartesian_wrench_enabled_ || msg.data.size() != max_cartesian_wrench_.size()) {
      throw std::runtime_error("Cartesian wrench is disabled or malformed");
    }
    Wrench wrench;
    wrench.epoch = epoch_.load();
    for (size_t i = 0; i < wrench.value.size(); ++i) {
      const double value = msg.data[i];
      if (!std::isfinite(value) || std::abs(value) > max_cartesian_wrench_[i]) {
        throw std::runtime_error("Cartesian wrench exceeds configured limit");
      }
      wrench.value[i] = value;
    }
    wrench.received_ns = steadyNow();
    wrench.sequence = command_sequence_.fetch_add(1) + 1;
    wrench_buffer_.writeFromNonRT(wrench);
  } catch (const std::exception &) {
    invalid_goals_.fetch_add(1);
  }
}

void PositionImpedanceController::receiveHealth(const diagnostic_msgs::msg::DiagnosticArray & msg)
{
  auto & h = callback_health_;
  const int64_t now = steadyNow();
  bool healthy = false;
  try {
    const diagnostic_msgs::msg::DiagnosticStatus * status = nullptr;
    for (const auto & item : msg.status) {
      if (item.name != "dynamixel_hardware_interface/BusHealth") {continue;}
      if (status) {throw std::runtime_error("duplicate health status");}
      status = &item;
    }
    if (!status || status->level != diagnostic_msgs::msg::DiagnosticStatus::OK) {
      throw std::runtime_error("missing or unhealthy bus status");
    }
    const int64_t stamp = rclcpp::Time(msg.header.stamp).nanoseconds();
    const double age = (get_node()->now().nanoseconds() - stamp) * 1e-9;
    if (stamp <= last_health_stamp_ns_ || age < 0.0 || age > health_timeout_) {
      throw std::runtime_error("stale health sample");
    }
    last_health_stamp_ns_ = stamp;
    std::map<std::string, std::string> fields;
    for (const auto & item : status->values) {
      if (!fields.emplace(item.key, item.value).second) {throw std::runtime_error("duplicate health key");}
    }
    const auto number = [&](const char * key) {
        const auto & value = fields.at(key);
        if (value.empty() || value.find_first_not_of("0123456789") != std::string::npos) {
          throw std::runtime_error("invalid health integer");
        }
        return std::stoull(value);
      };
    const auto flag = [&](const char * key, bool expected) {
        const auto & value = fields.at(key);
        return expected ? (value == "true" || value == "1") : (value == "false" || value == "0");
      };
    const auto reads = number("read_failure_count"), writes = number("write_failure_count");
    const auto instance = fields.at("driver_instance_id");
    if (!health_instance_.empty() && (instance != health_instance_ || reads != last_read_count_ ||
      writes != last_write_count_)) {++h.faults; h.clean_since_ns = 0;}
    last_read_count_ = reads; last_write_count_ = writes; health_instance_ = instance;
    healthy = number("schema_version") == 1 && !instance.empty() &&
      number("consecutive_read_failures") == 0 && number("consecutive_write_failures") == 0 &&
      fields.at("current_read_error") == "0" && fields.at("current_write_error") == "0" &&
      number("hardware_error_mask") == 0 && flag("fail_safe_triggered", false) &&
      flag("torque_all_enabled", true) && flag("torque_enable_inhibited", false) &&
      flag("hardware_error_monitoring_complete", true) && flag("input_voltage_monitoring_complete", true);
    if (now - h.received_ns > static_cast<int64_t>(health_timeout_ * 1e9)) {
      ++h.faults; h.clean_since_ns = 0;
    }
    h.received_ns = now - static_cast<int64_t>(age * 1e9);
  } catch (const std::exception &) {healthy = false; h.received_ns = now;}
  if (!healthy) {++h.faults; h.clean_since_ns = 0;}
  else if (!h.clean_since_ns) {h.clean_since_ns = now;}
  h.healthy = healthy;
  health_buffer_.writeFromNonRT(h);
}

bool PositionImpedanceController::healthReady(int64_t now)
{
  if (!require_health_) {return true;}
  const auto * h = health_buffer_.readFromRT();
  return h && h->healthy && h->received_ns > 0 && now >= h->received_ns &&
    (now - h->received_ns) * 1e-9 <= health_timeout_ && h->clean_since_ns > 0 &&
    (now - h->clean_since_ns) * 1e-9 >= clean_window_;
}

controller_interface::CallbackReturn PositionImpedanceController::on_activate(
  const rclcpp_lifecycle::State &)
{
  activation_attempted_.store(true);
  active_.store(false);
  const size_t n = joints_.size();
  const auto fail = [this](const char * reason) {
      RCLCPP_ERROR(get_node()->get_logger(), "Impedance activation refused: %s", reason);
      return controller_interface::CallbackReturn::ERROR;
    };
  if (!get_node()->get_parameter("enable_on_activation").as_bool()) {
    return fail("enable_on_activation is false; commissioning gate remains closed");
  }
  // The supervisor captures and explicitly confirms a stationary supported
  // pose AFTER the mode switch. Read that frozen reference at activation,
  // not the pre-switch READY value cached during configure. The requirement
  // and tolerance remain fixed at configure; parameter writes cannot loosen
  // either while this controller instance is inactive/active.
  const auto confirmed_pose = get_node()->get_parameter("activation_pose").as_double_array();
  if ((require_activation_pose_ && confirmed_pose.empty()) ||
    (!confirmed_pose.empty() && (confirmed_pose.size() != n ||
    !std::all_of(confirmed_pose.begin(), confirmed_pose.end(),
      [](double q) {return std::isfinite(q);}))))
  {return fail("invalid or missing confirmed activation pose");}
  activation_pose_ = confirmed_pose;
  if (command_interfaces_.size() != n || state_interfaces_.size() != 2 * n) {
    return fail("wrong number of claimed effort/state interfaces");
  }
  if (!healthReady(steadyNow())) {return fail("bus health missing, stale, unhealthy, or clean window incomplete");}
  position_indices_.clear(); velocity_indices_.clear(); effort_indices_.clear();
  for (const auto & joint : joints_) {
    for (const auto & type : {"position", "velocity", "effort"}) {
      const auto name = joint + "/" + type;
      size_t index = 0;
      if (std::string(type) == "effort") {
        while (index < command_interfaces_.size() && command_interfaces_[index].get_name() != name) {++index;}
        if (index == command_interfaces_.size()) {return fail("missing named effort interface");}
        effort_indices_.push_back(index);
      } else {
        while (index < state_interfaces_.size() && state_interfaces_[index].get_name() != name) {++index;}
        if (index == state_interfaces_.size()) {return fail("missing named state interface");}
        (std::string(type) == "position" ? position_indices_ : velocity_indices_).push_back(index);
      }
    }
  }
  if (!readState() || !model_.computeChecked(q_, gravity_)) {return fail("invalid measured state or gravity solver failure");}
  for (size_t i = 0; i < n; ++i) {
    if (!activation_pose_.empty() && std::abs(q_[i] - activation_pose_[i]) > activation_pose_tolerance_) {
      return fail("measured pose moved away from the explicitly confirmed supported pose");
    }
    if (q_[i] < lower_[i] || q_[i] > upper_[i] ||
      std::abs(v_[i]) > std::min(velocity_limit_[i], 0.05) ||
      std::abs(gravity_[i]) > total_max_[i]) {return fail("pose/speed outside limits or insufficient gravity torque capacity");}
  }
  target_ = reference_ = q_;
  std::fill(residual_.begin(), residual_.end(), 0.0);
  ramp_ = 0.0; fault_ = false; hard_fault_ = false; fault_flags_ = 0; status_ = RAMPING;
  limit_event_ = {{1, 0, -1, 0, 0, 0}};
  current_limits_ok_ = false;
  activation_ns_ = steadyNow();
  activation_health_faults_ = health_buffer_.readFromRT()->faults;
  activation_invalid_goals_ = invalid_goals_.load();
  activation_command_sequence_ = command_sequence_.load();
  epoch_.fetch_add(1);  // Inactive/previous-session commands can never replay.
  zeroCommands();
  active_.store(true);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PositionImpedanceController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  active_.store(false); epoch_.fetch_add(1); zeroCommands();
  return controller_interface::CallbackReturn::SUCCESS;
}

bool PositionImpedanceController::readState()
{
  for (size_t i = 0; i < joints_.size(); ++i) {
    q_[i] = state_interfaces_[position_indices_[i]].get_value();
    v_[i] = state_interfaces_[velocity_indices_[i]].get_value();
    if (!std::isfinite(q_[i]) || !std::isfinite(v_[i])) {return false;}
  }
  return true;
}

void PositionImpedanceController::zeroCommands()
{
  for (auto & interface : command_interfaces_) {interface.set_value(0.0);}
  std::fill(raw_.begin(), raw_.end(), 0.0);
  std::fill(residual_.begin(), residual_.end(), 0.0);
  std::fill(idle_damping_.begin(), idle_damping_.end(), 0.0);
  std::fill(joint_wrench_.begin(), joint_wrench_.end(), 0.0);
}

void PositionImpedanceController::publish()
{
  if (limit_telemetry_ && limit_telemetry_->trylock()) {
    std::copy(limit_event_.begin(), limit_event_.end(), limit_telemetry_->msg_.data.begin());
    limit_telemetry_->msg_.data[6] = current_limits_ok_ ? 1.0 : 0.0;
    limit_telemetry_->unlockAndPublish();
  }
  if (!telemetry_ || !telemetry_->trylock()) {return;}
  auto & data = telemetry_->msg_.data;
  data[0] = status_;
  data[1] = fault_flags_;
  size_t offset = 2;
  for (const auto * values : {&q_, &target_, &reference_, &gravity_, &budget_, &residual_, &raw_}) {
    std::copy(values->begin(), values->end(), data.begin() + offset); offset += values->size();
  }
  telemetry_->unlockAndPublish();
}

controller_interface::return_type PositionImpedanceController::update(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  current_limits_ok_ = false;
  if (!active_.load()) {zeroCommands(); return controller_interface::return_type::OK;}
  if (hard_fault_) {status_ = FAULT; zeroCommands(); publish(); return controller_interface::return_type::OK;}
  const int64_t now = steadyNow();
  const double dt = period.seconds();
  const bool state_ok = readState();
  const bool health_ok = healthReady(now);
  const bool model_ok = state_ok && model_.computeChecked(q_, gravity_);
  if (!state_ok) {fault_flags_ |= 1;}
  if (!health_ok) {fault_flags_ |= 2;}
  if (state_ok && !model_ok) {fault_flags_ |= 4;}
  if (!state_ok || !health_ok || !model_ok) {
    fault_ = hard_fault_ = true; status_ = FAULT; zeroCommands(); publish();
    return controller_interface::return_type::OK;
  }
  if (invalid_goals_.load() != activation_invalid_goals_) {fault_flags_ |= 8;}
  if (!positive(dt) || dt > max_period_) {fault_flags_ |= 16;}
  if (require_health_ && health_buffer_.readFromRT()->faults != activation_health_faults_) {fault_flags_ |= 256;}
  current_limits_ok_ = true;
  for (size_t i = 0; i < joints_.size(); ++i) {
    const bool position_trip = q_[i] < lower_[i] || q_[i] > upper_[i];
    const bool speed_trip = std::abs(v_[i]) > velocity_limit_[i];
    if (position_trip || speed_trip) {
      current_limits_ok_ = false;
      if (!(fault_flags_ & 32)) {
        limit_event_ = {{1, position_trip ? 1.0 : 2.0, static_cast<double>(i),
          position_trip ? q_[i] : v_[i], position_trip ? lower_[i] : -velocity_limit_[i],
          position_trip ? upper_[i] : velocity_limit_[i]}};
      }
      fault_flags_ |= 32;
    }
    if (std::abs(gravity_[i]) > total_max_[i]) {fault_flags_ |= 64;}
  }
  fault_ = fault_flags_ != 0;
  if (positive(dt) && dt <= max_period_) {ramp_ = std::min(1.0, ramp_ + dt / ramp_seconds_);}
  const auto * goal = goal_buffer_.readFromRT();
  const auto * wrench = wrench_buffer_.readFromRT();
  const uint64_t active_epoch = epoch_.load();
  const bool fresh_goal = goal && goal->q.size() == joints_.size() && goal->epoch == active_epoch &&
    goal->sequence > activation_command_sequence_ && goal->received_ns >= activation_ns_ &&
    now >= goal->received_ns && (now - goal->received_ns) * 1e-9 <= timeout_;
  const bool fresh_wrench = wrench && cartesian_wrench_enabled_ && wrench->epoch == active_epoch &&
    wrench->sequence > activation_command_sequence_ && wrench->received_ns >= activation_ns_ &&
    now >= wrench->received_ns && (now - wrench->received_ns) * 1e-9 <= timeout_;
  // Position targets and direct Cartesian wrenches are two mutually exclusive
  // control laws.  Mixing them is never resolved by last-writer-wins.
  if (fresh_goal && fresh_wrench) {fault_flags_ |= 512;}
  const bool wrench_tracking = fresh_wrench && !fresh_goal && !(fault_flags_ & 512);
  const bool tracking = fresh_goal || wrench_tracking;
  if (fresh_goal) {std::copy(goal->q.begin(), goal->q.end(), target_.begin());}
  if (wrench_tracking && !model_.computeJacobianChecked(q_, jacobian_)) {fault_flags_ |= 4;}
  fault_ = fault_flags_ != 0;
  status_ = fault_ ? FAULT : ramp_ < 1.0 ? RAMPING : tracking ? TRACKING :
    ((goal && goal->epoch == active_epoch) || (wrench && wrench->epoch == active_epoch) ? TIMEOUT : WAITING);
  for (size_t i = 0; i < joints_.size(); ++i) {
    budget_[i] = impedanceBudget(gravity_[i], base_[i], ratio_[i], extra_max_[i], total_max_[i]);
  }
  double wrench_scale = 1.0;
  if (wrench_tracking && !fault_) {
    for (size_t i = 0; i < joints_.size(); ++i) {
      joint_wrench_[i] = 0.0;
      for (size_t row = 0; row < wrench->value.size(); ++row) {
        joint_wrench_[i] += jacobian_[row * joints_.size() + i] * wrench->value[row];
      }
      if (!std::isfinite(joint_wrench_[i])) {fault_flags_ |= 128; break;}
      if (std::abs(joint_wrench_[i]) > 1e-12) {
        wrench_scale = std::min(wrench_scale, budget_[i] / std::abs(joint_wrench_[i]));
      }
    }
    wrench_scale = std::clamp(wrench_scale, 0.0, 1.0);
    if (fault_flags_ & 128) {fault_ = hard_fault_ = true; status_ = FAULT;}
  }
  if (hard_fault_) {zeroCommands(); publish(); return controller_interface::return_type::OK;}
  for (size_t i = 0; i < joints_.size(); ++i) {
    budget_[i] = impedanceBudget(gravity_[i], base_[i], ratio_[i], extra_max_[i], total_max_[i]);
    if (status_ == TRACKING && wrench_tracking) {
      // Virtual work: tau = J^T w.  One common scale preserves wrench
      // direction whenever a joint's residual-torque budget is exhausted.
      reference_[i] = target_[i] = q_[i];
      idle_damping_[i] = 0.0;
      const double cap = ramp_ * budget_[i];
      const double request = std::clamp(
        wrench_scale * joint_wrench_[i] - ramp_ * damping_[i] * v_[i], -cap, cap);
      // Build authority at the configured torque slew rate, but never keep a
      // stale-sign force while an operator releases or reverses an axis.
      if (request == 0.0 || residual_[i] * request <= 0.0 ||
        std::abs(request) < std::abs(residual_[i]))
      {
        residual_[i] = request;
      } else {
        residual_[i] += std::clamp(request - residual_[i], -torque_rate_[i] * dt,
          torque_rate_[i] * dt);
      }
    } else if (status_ == TRACKING) {
      idle_damping_[i] = 0.0;  // Never carry a spring/friction torque into idle.
      reference_[i] += std::clamp(target_[i] - reference_[i], -reference_rate_[i] * dt, reference_rate_[i] * dt);
      reference_[i] = std::clamp(reference_[i], std::max(lower_[i], q_[i] - lookahead_[i]),
        std::min(upper_[i], q_[i] + lookahead_[i]));
      double request = stiffness_[i] * (reference_[i] - q_[i]) - damping_[i] * v_[i];
      if (use_friction_) {request += GravityModel::friction(v_[i], friction_[i]);}
      residual_[i] += std::clamp(request - residual_[i], -torque_rate_[i] * dt, torque_rate_[i] * dt);
      residual_[i] = std::clamp(residual_[i], -budget_[i], budget_[i]);
    } else if (!fault_) {
      // Healthy WAITING, TIMEOUT and RAMPING: no position spring, no friction
      // feedforward, only bounded damping opposing measured velocity. The
      // previous tracking residual must be discarded immediately on timeout.
      reference_[i] = q_[i];
      const double cap = ramp_ * budget_[i];
      const double request = std::clamp(-ramp_ * damping_[i] * v_[i], -cap, cap);
      // Drop stale-sign torque immediately on reversal/zero speed, or when
      // the requested magnitude/budget shrinks. These bounds take precedence
      // over slew: idle torque must never assist measured motion.
      idle_damping_[i] = std::clamp(idle_damping_[i], std::min(0.0, request), std::max(0.0, request));
      idle_damping_[i] += std::clamp(request - idle_damping_[i],
        -torque_rate_[i] * dt, torque_rate_[i] * dt);
      residual_[i] = idle_damping_[i];
    } else {
      reference_[i] = q_[i]; residual_[i] = 0.0;
      idle_damping_[i] = 0.0;  // Fault policy is unchanged; no automatic recovery.
    }
    // The gravity ramp is support-dependent, not a bumpless loaded transfer.
    raw_[i] = std::clamp((ramp_ * gravity_[i] + residual_[i]) * scale_[i],
      -total_max_[i] * scale_[i], total_max_[i] * scale_[i]);
    if (!std::isfinite(raw_[i])) {
      fault_flags_ |= 128;
      fault_ = hard_fault_ = true; status_ = FAULT; zeroCommands(); publish();
      return controller_interface::return_type::OK;
    }
  }
  // Atomic at the cycle level: validate every joint before writing any effort.
  for (size_t i = 0; i < joints_.size(); ++i) {command_interfaces_[effort_indices_[i]].set_value(raw_[i]);}
  publish();
  return controller_interface::return_type::OK;
}
}  // namespace om6dof_controllers

PLUGINLIB_EXPORT_CLASS(om6dof_controllers::PositionImpedanceController, controller_interface::ControllerInterface)
