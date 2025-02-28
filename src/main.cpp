// "Copyright [2025] <jabural>"

#include "main.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"
#include "dms_pkg/action/my_action.hpp"


#define DEFAULT_BT_XML "/home/javier/javier_ws/src/dms_pkg/config/main_tree.xml"

typedef dms_pkg::action::MyAction ThinkingAction;
typedef rclcpp_action::ClientGoalHandle<ThinkingAction> GoalHandle;

// Define a ROS2 node that encapsulates the behavior tree
class BehaviorTreeNode : public rclcpp::Node {
 public:
  BehaviorTreeNode()
  : Node("behavior_tree_node") {
    // -----------------------------
    // Set up the BehaviorTreeFactory
    // -----------------------------
    // Register a custom node type using inheritance
    factory.registerNodeType<ListeningAction>("ListeningAction");
    factory.registerNodeType<ThinkingNode>("ThinkingNode");
    factory.registerNodeType<SpeakingAction>("SpeakingAction");

    // ------------------------------------
    // Create a subscriber to activate tree
    // ------------------------------------
    subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "activate_tree", 10,
      std::bind(&BehaviorTreeNode::topic_callback, this, std::placeholders::_1));

    action_client_ = rclcpp_action::create_client<ThinkingAction>(
      this,
      "thinking");

    RCLCPP_INFO(this->get_logger(), "BehaviorTreeNode has been started.");
  }

  void initTree() {
    auto blackboard = BT::Blackboard::create();
    // Now shared_from_this() is safe because the node is managed by a shared_ptr
    blackboard->set<rclcpp::Node::SharedPtr>("node", this->shared_from_this());
    tree = factory.createTreeFromFile(DEFAULT_BT_XML, blackboard);
  }

 private:
  // Callback that is triggered when a message is received
  void topic_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data) {
      RCLCPP_INFO(this->get_logger(), "Received activation signal. Ticking behavior tree...");
      tree.tickRoot();
    } else {
      RCLCPP_INFO(this->get_logger(), "Activation signal was false; tree not ticked.");
    }
  }

  void start_thinking() {
    using std::placeholders::_1; using std::placeholders::_2;

    auto goal_msg = ThinkingAction::Goal();
    goal_msg.duration = 6;

    this->action_client_->wait_for_action_server();

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<ThinkingAction>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&BehaviorTreeNode::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&BehaviorTreeNode::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&BehaviorTreeNode::result_callback, this, _1);

    this->action_client_->async_send_goal(goal_msg, send_goal_options);
  }

  void goal_response_callback(GoalHandle::SharedPtr future) {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandle::SharedPtr future,
    const std::shared_ptr<const ThinkingAction::Feedback> feedback) {
    (void)future;  // Not using right now
    std::cout << "Feedback: " << feedback->progress << std::endl;
  }

  void result_callback(const GoalHandle::WrappedResult & result) {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        std::cout << "Time elapsed: " << result.result->success << std::endl;
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        break;
      default:
        // RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        std::cout << "Unknown result code" << std::endl;
        break;
    }

    rclcpp::shutdown();
  }

  // Member variables:
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
  BT::BehaviorTreeFactory factory;     // Factory for creating our behavior tree
  BT::Tree tree;                   // The behavior tree (assuming BT::Tree is the correct type)
  rclcpp_action::Client<ThinkingAction>::SharedPtr action_client_;
};

int main(int argc, char ** argv) {
  // Initialize ROS2
  rclcpp::init(argc, argv);

  // Create our node instance
  auto node = std::make_shared<BehaviorTreeNode>();

  node->initTree();

  // Spin the node so that the subscriber callback is called upon receiving messages
  rclcpp::spin(node);

  // Shutdown ROS2
  rclcpp::shutdown();
  return 0;
}
