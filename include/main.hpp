// "Copyright [2025] jabural"
#ifndef INCLUDE_MAIN_HPP_
#define INCLUDE_MAIN_HPP_

#include <memory>
#include <string>
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "dms_pkg/action/my_action.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"


typedef dms_pkg::action::MyAction ThinkingAction;
typedef rclcpp_action::ClientGoalHandle<ThinkingAction> GoalHandle;

// Example of custom SyncActionNode (synchronous action)
// without ports.
class ListeningAction : public BT::SyncActionNode {
 public:
  explicit ListeningAction(const std::string & name) :
      BT::SyncActionNode(name, {}) {}

  // You must override the virtual function tick()
  BT::NodeStatus tick() override {
    std::cout << "ListeningAction: " << this->name() << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};

class ThinkingNode : public BT::StatefulActionNode {
 public:
  ThinkingNode(const std::string & name, const BT::NodeConfiguration & conf)
    : BT::StatefulActionNode(name, conf),
      action_in_progress_(false),
      action_finished_(false),
      action_success_(false) {
    // Initialize your ROS2 action client here.
    // Note: You may need to pass a shared pointer to an rclcpp::Node via the BT blackboard.
    ros_node_ = conf.blackboard->get<rclcpp::Node::SharedPtr>("node");
    action_client_ = rclcpp_action::create_client<ThinkingAction>(ros_node_, "thinking");
  }

  static BT::PortsList providedPorts() {
    // Optionally, define input ports, e.g., for the action duration.
    return { BT::InputPort<int>("duration") };
  }

  // Called once when the node is first ticked
  BT::NodeStatus onStart() override {
    // (Optionally) retrieve goal parameters from the blackboard.
    if (action_in_progress_) {
      // A goal is already active, so just remain running.
      return BT::NodeStatus::RUNNING;
    }

    int duration;
    if (!getInput<int>("duration", duration))
      duration = 6;  // default value

    // Prepare the goal
    auto goal_msg = ThinkingAction::Goal();
    goal_msg.duration = duration;

    // Ensure the action server is available.
    if (!action_client_->wait_for_action_server()) {
      RCLCPP_ERROR(ros_node_->get_logger(), "Action server not available");
      return BT::NodeStatus::FAILURE;
    }

    // Send the goal asynchronously
    auto send_goal_options = rclcpp_action::Client<ThinkingAction>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      [this](GoalHandle::SharedPtr goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(ros_node_->get_logger(), "Goal was rejected by server");
          this->action_finished_ = true;
          this->action_success_ = false;
        } else {
          RCLCPP_INFO(ros_node_->get_logger(), "Goal accepted by server");
        }
      };
    send_goal_options.feedback_callback =
      [this](GoalHandle::SharedPtr, const std::shared_ptr<const ThinkingAction::Feedback> feedback) {
        // Optionally process feedback
        std::cout << "Feedback: " << feedback->progress << std::endl;
      };
    send_goal_options.result_callback =
      [this](const GoalHandle::WrappedResult & result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            std::cout << "Action succeeded, result: " << result.result->success << std::endl;
            this->action_success_ = true;
            break;
          default:
            std::cout << "Action failed or was canceled" << std::endl;
            this->action_success_ = false;
            break;
        }
        this->action_finished_ = true;
      };

    action_client_->async_send_goal(goal_msg, send_goal_options);
    action_in_progress_ = true;
    return BT::NodeStatus::RUNNING;
  }

  // Called on every tick until the node returns SUCCESS/FAILURE.
  BT::NodeStatus onRunning() override {
    // Optionally process ROS callbacks, e.g., if not using a separate spinning thread.
    // rclcpp::spin_some(ros_node_);

    if (!action_finished_) {
      return BT::NodeStatus::RUNNING;
    } else {
      // Reset state for the next tick
      action_in_progress_ = false;
      return action_success_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
  }

  // Called if the node is halted (e.g., higher priority node interrupts execution).
  void onHalted() override {
    if (action_in_progress_) {
      // Cancel the goal if possible.
      // Note: cancellation APIs may require additional error handling.
      action_client_->async_cancel_all_goals();
    }
    action_in_progress_ = false;
    action_finished_ = false;
  }

 private:
  bool action_in_progress_;
  bool action_finished_;
  bool action_success_;
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp_action::Client<ThinkingAction>::SharedPtr action_client_;
};


class SpeakingAction : public BT::SyncActionNode {
 public:
  explicit SpeakingAction(const std::string & name) :
  BT::SyncActionNode(name, {}) {}
  BT::NodeStatus tick() {
    std::cout << "SpeakingAction open" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};

#endif  // INCLUDE_MAIN_HPP_
