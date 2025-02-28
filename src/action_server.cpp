// "Copyright [2025] <jabural>"

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "dms_pkg/action/my_action.hpp"

typedef dms_pkg::action::MyAction ThinkingAction;
typedef rclcpp_action::ServerGoalHandle<ThinkingAction> GoalHandle;

class ThinkingServerNode : public rclcpp::Node{
 public:
 ThinkingServerNode() : Node("thinking_server_node") {
  action_server_ = rclcpp_action::create_server<ThinkingAction>(
    this,
    "thinking",
    std::bind(&ThinkingServerNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&ThinkingServerNode::handle_cancel, this, std::placeholders::_1),
    std::bind(&ThinkingServerNode::handle_accepted, this, std::placeholders::_1));

  std::cout << "Navigate Action Server Started" << std::endl;
}

 private:
    rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const ThinkingAction::Goal> goal) {
      std::cout << "Received goal_duration: " << goal << std::endl;

      (void)uuid;  // Not using this argument right now
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandle> goal_handle) {
      RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
      (void)goal_handle;
      return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
      // this needs to return quickly to avoid blocking the executor, so spin up a new thread
      std::thread{std::bind(&ThinkingServerNode::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandle> goal_handle) {
      RCLCPP_INFO(this->get_logger(), "Executing goal");
      auto start_time = rclcpp::Clock().now();

      // goal_handle->

      const auto goal = goal_handle->get_goal();
      auto feedback = std::make_shared<ThinkingAction::Feedback>();
      auto result = std::make_shared<ThinkingAction::Result>();

      rclcpp::Rate loop_rate(1);  // Rate in Hz to calculate distance, publish feedback

      while (feedback->progress < goal->duration) {
        feedback->progress = static_cast<float>((rclcpp::Clock().now() - start_time).seconds());

        goal_handle->publish_feedback(feedback);
        loop_rate.sleep();
      }

      result->success = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }

    rclcpp_action::Server<ThinkingAction>::SharedPtr action_server_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ThinkingServerNode>());
  rclcpp::shutdown();

  return 0;
}
