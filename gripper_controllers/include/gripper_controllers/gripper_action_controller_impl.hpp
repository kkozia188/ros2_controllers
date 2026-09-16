// Copyright 2014, SRI International
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

/// \author Sachin Chitta, Adolfo Rodriguez Tsouroukdissian, Stu Glaser

#ifndef GRIPPER_CONTROLLERS__GRIPPER_ACTION_CONTROLLER_IMPL_HPP_
#define GRIPPER_CONTROLLERS__GRIPPER_ACTION_CONTROLLER_IMPL_HPP_

#include "gripper_controllers/gripper_action_controller.hpp"

#include <memory>
#include <string>

namespace gripper_action_controller
{
template <const char * HardwareInterface>
void GripperActionController<HardwareInterface>::preempt_active_goal()
{
  // Cancels the currently active goal
  const auto active_goal = *rt_active_goal_.readFromNonRT();
  if (active_goal)
  {
    // Marks the current goal as canceled
    active_goal->setCanceled(std::make_shared<GripperCommandAction::Result>());
    rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
  }
}

template <const char * HardwareInterface>
controller_interface::CallbackReturn GripperActionController<HardwareInterface>::on_init()
{
  try
  {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

template <const char * HardwareInterface>
controller_interface::return_type GripperActionController<HardwareInterface>::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (params_.pp.enabled) {return update_pp(period);}
  command_struct_rt_ = *(command_.readFromRT());

  const double current_position = joint_position_state_interface_->get().get_value();
  const double current_velocity = joint_velocity_state_interface_->get().get_value();

  const double error_position = command_struct_rt_.position_ - current_position;
  const double error_velocity = -current_velocity;

  check_for_success(get_node()->now(), error_position, current_position, current_velocity);

  // Hardware interface adapter: Generate and send commands
  computed_command_ = hw_iface_adapter_.updateCommand(
    command_struct_rt_.position_, 0.0, error_position, error_velocity,
    command_struct_rt_.max_effort_);
  return controller_interface::return_type::OK;
}

template <const char * HardwareInterface>
rclcpp_action::GoalResponse GripperActionController<HardwareInterface>::goal_callback(
  const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const GripperCommandAction::Goal> goal)
{
  std::lock_guard<std::mutex> guard(pp_lifecycle_mutex_);
  if (params_.pp.enabled)
  {
    const auto & command = goal->command;
    bool expected = false;
    if (!pp_active_.load() || !pp_ready_.load() ||
      !std::isfinite(command.position) || !std::isfinite(command.max_effort) ||
      command.position < params_.pp.min_position || command.position > params_.pp.max_position ||
      command.max_effort <= 0.0 || command.max_effort > params_.max_effort ||
      pp_sequence_ >= 9007199254740991ULL || !pp_busy_.compare_exchange_strong(expected, true))
    {
      return rclcpp_action::GoalResponse::REJECT;
    }
    pp_pending_goal_ = uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  RCLCPP_INFO(get_node()->get_logger(), "Received & accepted new action goal");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

template <const char * HardwareInterface>
void GripperActionController<HardwareInterface>::accepted_callback(
  std::shared_ptr<GoalHandle> goal_handle)  // Try to update goal
{
  auto rt_goal = std::make_shared<RealtimeGoalHandle>(goal_handle);
  std::lock_guard<std::mutex> guard(pp_lifecycle_mutex_);

  if (params_.pp.enabled)
  {
    if (!pp_active_.load() || !pp_pending_goal_ || *pp_pending_goal_ != goal_handle->get_goal_id())
    {
      goal_handle->abort(std::make_shared<GripperCommandAction::Result>());
      return;
    }
    pp_pending_goal_.reset();
    command_struct_ = {goal_handle->get_goal()->command.position,
      goal_handle->get_goal()->command.max_effort, ++pp_sequence_, false};
    pp_result_code_.store(0);
    rt_goal->execute();
    rt_active_goal_.writeFromNonRT(rt_goal);
    command_.writeFromNonRT(command_struct_);
    goal_handle_timer_.reset();
    goal_handle_timer_ = get_node()->create_wall_timer(
      action_monitor_period_.to_chrono<std::chrono::nanoseconds>(), [this, rt_goal]()
      {
        // ROS Action publication and its internal locks stay outside update().
        std::lock_guard<std::mutex> guard(pp_lifecycle_mutex_);
        if (!pp_active_.load()) {return;}
        const int result = pp_result_code_.load();
        if (result > 0)
        {
          if (result == 1) {rt_goal->gh_->succeed(pre_alloc_result_);}
          else if (result == 2) {rt_goal->gh_->canceled(pre_alloc_result_);}
          else {rt_goal->gh_->abort(pre_alloc_result_);}
          rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
          pp_busy_.store(false);
          pp_result_code_.store(0);
        }
        else if (rt_goal->gh_->is_executing())
        {
          auto & feedback = rt_goal->preallocated_feedback_;
          feedback->position = pp_position_.load();
          feedback->effort = pp_effort_.load();
          feedback->reached_goal = false;
          feedback->stalled = false;
          rt_goal->gh_->publish_feedback(feedback);
        }
      });
    return;
  }

  // Accept new goal
  preempt_active_goal();

  // This is the non-realtime command_struct
  // We use command_ for sharing
  command_struct_.position_ = goal_handle->get_goal()->command.position;
  command_struct_.max_effort_ = goal_handle->get_goal()->command.max_effort;
  command_.writeFromNonRT(command_struct_);

  pre_alloc_result_->reached_goal = false;
  pre_alloc_result_->stalled = false;

  last_movement_time_ = get_node()->now();
  rt_goal->execute();
  rt_active_goal_.writeFromNonRT(rt_goal);

  // Set smartpointer to expire for create_wall_timer to delete previous entry from timer list
  goal_handle_timer_.reset();

  // Setup goal status checking timer
  goal_handle_timer_ = get_node()->create_wall_timer(
    action_monitor_period_.to_chrono<std::chrono::nanoseconds>(),
    std::bind(&RealtimeGoalHandle::runNonRealtime, rt_goal));
}

template <const char * HardwareInterface>
rclcpp_action::CancelResponse GripperActionController<HardwareInterface>::cancel_callback(
  const std::shared_ptr<GoalHandle> goal_handle)
{
  std::lock_guard<std::mutex> guard(pp_lifecycle_mutex_);
  if (params_.pp.enabled)
  {
    const auto active = *rt_active_goal_.readFromNonRT();
    int expected = 0;
    if (!pp_active_.load() || !active || active->gh_ != goal_handle ||
      !pp_result_code_.compare_exchange_strong(expected, -1))
    {
      return rclcpp_action::CancelResponse::REJECT;
    }
    command_struct_.halt_ = true;
    command_.writeFromNonRT(command_struct_);
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  RCLCPP_INFO(get_node()->get_logger(), "Got request to cancel goal");

  // Check that cancel request refers to currently active goal (if any)
  const auto active_goal = *rt_active_goal_.readFromNonRT();
  if (active_goal && active_goal->gh_ == goal_handle)
  {
    // Enter hold current position mode
    set_hold_position();

    RCLCPP_INFO(
      get_node()->get_logger(), "Canceling active action goal because cancel callback received.");

    // Mark the current goal as canceled
    auto action_res = std::make_shared<GripperCommandAction::Result>();
    active_goal->setCanceled(action_res);
    // Reset current goal
    rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

template <const char * HardwareInterface>
void GripperActionController<HardwareInterface>::set_hold_position()
{
  command_struct_.position_ = joint_position_state_interface_->get().get_value();
  command_struct_.max_effort_ = params_.max_effort;
  command_.writeFromNonRT(command_struct_);
}

template <const char * HardwareInterface>
void GripperActionController<HardwareInterface>::check_for_success(
  const rclcpp::Time & time, double error_position, double current_position,
  double current_velocity)
{
  const auto active_goal = *rt_active_goal_.readFromNonRT();
  if (!active_goal)
  {
    return;
  }

  if (fabs(error_position) < params_.goal_tolerance)
  {
    pre_alloc_result_->effort = computed_command_;
    pre_alloc_result_->position = current_position;
    pre_alloc_result_->reached_goal = true;
    pre_alloc_result_->stalled = false;
    RCLCPP_DEBUG(get_node()->get_logger(), "Successfully moved to goal.");
    active_goal->setSucceeded(pre_alloc_result_);
    rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
  }
  else
  {
    if (fabs(current_velocity) > params_.stall_velocity_threshold)
    {
      last_movement_time_ = time;
    }
    else if ((time - last_movement_time_).seconds() > params_.stall_timeout)
    {
      pre_alloc_result_->effort = computed_command_;
      pre_alloc_result_->position = current_position;
      pre_alloc_result_->reached_goal = false;
      pre_alloc_result_->stalled = true;

      if (params_.allow_stalling)
      {
        RCLCPP_DEBUG(get_node()->get_logger(), "Stall detected moving to goal. Returning success.");
        active_goal->setSucceeded(pre_alloc_result_);
      }
      else
      {
        RCLCPP_DEBUG(get_node()->get_logger(), "Stall detected moving to goal. Aborting action!");
        active_goal->setAborted(pre_alloc_result_);
      }
      rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
    }
  }
}

template <const char * HardwareInterface>
controller_interface::CallbackReturn GripperActionController<HardwareInterface>::on_configure(
  const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> guard(pp_lifecycle_mutex_);
  const auto logger = get_node()->get_logger();
  if (!param_listener_)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }
  params_ = param_listener_->get_params();

  if (params_.pp.enabled &&
    (std::string(HardwareInterface) != hardware_interface::HW_IF_POSITION ||
    !std::isfinite(params_.pp.min_position) || !std::isfinite(params_.pp.max_position) ||
    params_.pp.min_position < 0.0 || params_.pp.max_position <= params_.pp.min_position ||
    !std::isfinite(params_.max_effort) || params_.max_effort <= 0.0 ||
    !std::isfinite(params_.pp.command_timeout) || params_.pp.command_timeout <= 0.0 ||
    !std::isfinite(params_.goal_tolerance) || params_.goal_tolerance <= 0.0 ||
    !std::isfinite(params_.stall_timeout) || params_.stall_timeout <= 0.0 ||
    !std::isfinite(params_.stall_velocity_threshold) || params_.stall_velocity_threshold <= 0.0))
  {
    RCLCPP_ERROR(logger, "PP requires explicit position, force and timeout limits");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Action status checking update rate
  action_monitor_period_ = rclcpp::Duration::from_seconds(1.0 / params_.action_monitor_rate);
  RCLCPP_INFO(
    logger, "Action status changes will be monitored at %f Hz.", params_.action_monitor_rate);

  // Controlled joint
  if (params_.joint.empty())
  {
    RCLCPP_ERROR(logger, "Joint name cannot be empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}
template <const char * HardwareInterface>
controller_interface::CallbackReturn GripperActionController<HardwareInterface>::on_activate(
  const rclcpp_lifecycle::State &)
{
  std::unique_lock<std::mutex> pp_lock(pp_lifecycle_mutex_, std::defer_lock);
  if (params_.pp.enabled) {pp_lock.lock();}
  if (params_.pp.enabled)
  {
    const std::array<std::string, 3> commands{"max_effort", "pp_sequence", "pp_halt"};
    const std::array<std::string, 3> states{"pp_sequence", "pp_state", "effort"};
    for (size_t i = 0; i < commands.size(); ++i)
    {
      pp_commands_[i] = nullptr;
      pp_states_[i] = nullptr;
      for (auto & command : command_interfaces_)
      {
        if (command.get_name() == params_.joint + "/" + commands[i]) {pp_commands_[i] = &command;}
      }
      for (auto & state : state_interfaces_)
      {
        if (state.get_name() == params_.joint + "/" + states[i]) {pp_states_[i] = &state;}
      }
      if (!pp_commands_[i] || !pp_states_[i]) {return controller_interface::CallbackReturn::ERROR;}
    }
    pp_commands_[0]->set_value(0.0);
    const double previous_sequence = pp_states_[0]->get_value();
    if (!std::isfinite(previous_sequence) || previous_sequence < 0.0 ||
      previous_sequence > 9007199254740991.0 || std::floor(previous_sequence) != previous_sequence)
    {
      return controller_interface::CallbackReturn::ERROR;
    }
    pp_sequence_ = std::max(pp_sequence_, static_cast<uint64_t>(previous_sequence));
    pp_commands_[1]->set_value(previous_sequence);
    pp_commands_[2]->set_value(1.0);
    pp_busy_.store(false);
    pp_result_code_.store(0);
    pp_ready_.store(false);
    pp_rt_sequence_ = 0;
    pp_terminal_sequence_ = 0;
  }
  auto command_interface_it = std::find_if(
    command_interfaces_.begin(), command_interfaces_.end(),
    [](const hardware_interface::LoanedCommandInterface & command_interface)
    { return command_interface.get_interface_name() == HardwareInterface; });
  if (command_interface_it == command_interfaces_.end())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Expected 1 %s command interface", HardwareInterface);
    return controller_interface::CallbackReturn::ERROR;
  }
  if (command_interface_it->get_prefix_name() != params_.joint)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Command interface is different than joint name `%s` != `%s`",
      command_interface_it->get_prefix_name().c_str(), params_.joint.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  const auto position_state_interface_it = std::find_if(
    state_interfaces_.begin(), state_interfaces_.end(),
    [](const hardware_interface::LoanedStateInterface & state_interface)
    { return state_interface.get_interface_name() == hardware_interface::HW_IF_POSITION; });
  if (position_state_interface_it == state_interfaces_.end())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Expected 1 position state interface");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (position_state_interface_it->get_prefix_name() != params_.joint)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Position state interface is different than joint name `%s` != `%s`",
      position_state_interface_it->get_prefix_name().c_str(), params_.joint.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  const auto velocity_state_interface_it = std::find_if(
    state_interfaces_.begin(), state_interfaces_.end(),
    [](const hardware_interface::LoanedStateInterface & state_interface)
    { return state_interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY; });
  if (velocity_state_interface_it == state_interfaces_.end())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Expected 1 velocity state interface");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (velocity_state_interface_it->get_prefix_name() != params_.joint)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Velocity command interface is different than joint name `%s` != `%s`",
      velocity_state_interface_it->get_prefix_name().c_str(), params_.joint.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  joint_command_interface_ = *command_interface_it;
  joint_position_state_interface_ = *position_state_interface_it;
  joint_velocity_state_interface_ = *velocity_state_interface_it;

  // Hardware interface adapter
  hw_iface_adapter_.init(joint_command_interface_, get_node());

  // Command - non RT version
  command_struct_.position_ = joint_position_state_interface_->get().get_value();
  command_struct_.max_effort_ = params_.max_effort;
  if (params_.pp.enabled) {command_struct_.sequence_ = 0; command_struct_.halt_ = true;}
  command_.initRT(command_struct_);

  // Result
  pre_alloc_result_ = std::make_shared<control_msgs::action::GripperCommand::Result>();
  pre_alloc_result_->position = command_struct_.position_;
  pre_alloc_result_->reached_goal = false;
  pre_alloc_result_->stalled = false;

  // Action interface
  action_server_ = rclcpp_action::create_server<control_msgs::action::GripperCommand>(
    get_node(), "~/gripper_cmd",
    std::bind(
      &GripperActionController::goal_callback, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&GripperActionController::cancel_callback, this, std::placeholders::_1),
    std::bind(&GripperActionController::accepted_callback, this, std::placeholders::_1));

  pp_active_.store(params_.pp.enabled);

  return controller_interface::CallbackReturn::SUCCESS;
}

template <const char * HardwareInterface>
controller_interface::CallbackReturn GripperActionController<HardwareInterface>::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  std::unique_lock<std::mutex> pp_lock(pp_lifecycle_mutex_, std::defer_lock);
  if (params_.pp.enabled) {pp_lock.lock();}
  if (params_.pp.enabled && pp_active_.exchange(false))
  {
    pp_pending_goal_.reset();
    pp_ready_.store(false);
    pp_commands_[2]->set_value(1.0);
    const auto goal = *rt_active_goal_.readFromNonRT();
    if (goal && goal->gh_->is_active())
    {
      goal->gh_->abort(std::make_shared<GripperCommandAction::Result>());
    }
    goal_handle_timer_.reset();
    rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
    pp_busy_.store(false);
    pp_result_code_.store(0);
  }
  joint_command_interface_ = std::nullopt;
  joint_position_state_interface_ = std::nullopt;
  joint_velocity_state_interface_ = std::nullopt;
  release_interfaces();
  return controller_interface::CallbackReturn::SUCCESS;
}

template <const char * HardwareInterface>
controller_interface::InterfaceConfiguration
GripperActionController<HardwareInterface>::command_interface_configuration() const
{
  if (params_.pp.enabled)
  {
    return {controller_interface::interface_configuration_type::INDIVIDUAL,
      {params_.joint + "/position", params_.joint + "/max_effort",
        params_.joint + "/pp_sequence", params_.joint + "/pp_halt"}};
  }
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {params_.joint + "/" + HardwareInterface}};
}

template <const char * HardwareInterface>
controller_interface::InterfaceConfiguration
GripperActionController<HardwareInterface>::state_interface_configuration() const
{
  if (params_.pp.enabled)
  {
    return {controller_interface::interface_configuration_type::INDIVIDUAL,
      {params_.joint + "/position", params_.joint + "/velocity", params_.joint + "/pp_sequence",
        params_.joint + "/pp_state", params_.joint + "/effort"}};
  }
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {params_.joint + "/" + hardware_interface::HW_IF_POSITION,
     params_.joint + "/" + hardware_interface::HW_IF_VELOCITY}};
}

template <const char * HardwareInterface>
controller_interface::return_type GripperActionController<HardwareInterface>::update_pp(
  const rclcpp::Duration & period)
{
  if (!pp_active_.load()) {return controller_interface::return_type::OK;}
  const double position = joint_position_state_interface_->get().get_value();
  const double velocity = joint_velocity_state_interface_->get().get_value();
  const double sequence = pp_states_[0]->get_value();
  const double state = pp_states_[1]->get_value();
  const double effort = pp_states_[2]->get_value();
  static_assert(std::atomic<double>::is_always_lock_free, "PP snapshots require lock-free doubles");
  pp_position_.store(position);
  pp_effort_.store(effort);
  const bool healthy = std::isfinite(position) && std::isfinite(velocity) &&
    std::isfinite(effort) && std::isfinite(sequence) && state >= 1.0 && state <= 8.0;
  pp_ready_.store(healthy && (state == 1.0 || state == 6.0 || state == 8.0));
  const auto command = *command_.readFromRT();
  if (!pp_busy_.load() || command.sequence_ == 0 || command.sequence_ == pp_terminal_sequence_)
  {
    if (!healthy) {pp_commands_[2]->set_value(1.0);}
    return controller_interface::return_type::OK;
  }
  if (command.sequence_ != pp_rt_sequence_)
  {
    pp_rt_sequence_ = command.sequence_;
    pp_elapsed_ = 0.0;
    pp_stalled_ = 0.0;
    pp_stopping_ = false;
  }
  const double dt = period.seconds();
  const bool canceling = command.halt_ || pp_result_code_.load() == -1;
  pp_elapsed_ += std::isfinite(dt) && dt > 0.0 ? dt : params_.pp.command_timeout;
  // All fields are sampled from the same RealtimeBuffer command envelope.
  joint_command_interface_->get().set_value(command.position_);
  pp_commands_[0]->set_value(command.max_effort_);
  pp_commands_[1]->set_value(static_cast<double>(command.sequence_));
  pp_commands_[2]->set_value(canceling || pp_stopping_ ? 1.0 : 0.0);
  const bool matching = sequence == static_cast<double>(command.sequence_);
  bool reached = matching && state == 6.0 &&
    std::abs(command.position_ - position) <= params_.goal_tolerance;
  bool stalled = false;
  if (matching && state == 5.0 && !canceling && !pp_stopping_)
  {
    pp_stalled_ = std::abs(velocity) > params_.stall_velocity_threshold ? 0.0 : pp_stalled_ + dt;
    stalled = pp_stalled_ >= params_.stall_timeout;
  }
  if ((canceling || stalled) && !pp_stopping_)
  {
    pp_stopping_ = true;
    pp_elapsed_ = 0.0;
    pp_commands_[2]->set_value(1.0);
  }
  const bool stopped = matching && state == 8.0 &&
    std::abs(velocity) <= params_.stall_velocity_threshold;
  const bool failed = !healthy || pp_elapsed_ >= params_.pp.command_timeout;
  if (!failed && !((canceling || pp_stopping_) ? stopped : reached))
  {
    return controller_interface::return_type::OK;
  }
  pre_alloc_result_->position = position;
  pre_alloc_result_->effort = effort;
  pre_alloc_result_->reached_goal = !failed && !pp_stopping_ && reached;
  pre_alloc_result_->stalled = pp_stopping_ && !canceling;
  int result_code = 1;
  if (failed)
  {
    pp_commands_[2]->set_value(1.0);
    result_code = 3;
  }
  else if (canceling)
  {
    result_code = 2;
  }
  else if (pp_stopping_ && !params_.allow_stalling)
  {
    result_code = 3;
  }
  if (result_code == 1)
  {
    int expected = 0;
    if (!pp_result_code_.compare_exchange_strong(expected, 1))
    {
      // A cancel accepted after this cycle's snapshot wins over success.
      pp_commands_[2]->set_value(1.0);
      pp_stopping_ = true;
      pp_elapsed_ = 0.0;
      return controller_interface::return_type::OK;
    }
  }
  else {pp_result_code_.store(result_code);}
  pp_terminal_sequence_ = command.sequence_;
  return controller_interface::return_type::OK;
}

template <const char * HardwareInterface>
GripperActionController<HardwareInterface>::GripperActionController()
: controller_interface::ControllerInterface(),
  action_monitor_period_(rclcpp::Duration::from_seconds(0))
{
}

}  // namespace gripper_action_controller

#endif  // GRIPPER_CONTROLLERS__GRIPPER_ACTION_CONTROLLER_IMPL_HPP_
