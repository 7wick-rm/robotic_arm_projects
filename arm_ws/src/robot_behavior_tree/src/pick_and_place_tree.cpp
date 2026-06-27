#include <rclcpp/rclcpp.hpp>
#include <behaviortree_ros2/bt_action_node.hpp>
#include <behaviortree_ros2/bt_service_node.hpp>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>

#include "robot_interfaces/action/pick_and_place.hpp"
#include "robot_interfaces/srv/get_plan.hpp"
#include "robot_interfaces/msg/block_pose.hpp"

#include <map>
#include <queue>
#include <mutex>
#include <memory>
#include <string>

// ── Shared types ──────────────────────────────────────────────────────────────

struct BlockInfo {
    double x, y, z;
    int    block_id;
};

// Parsed place goal — passed through the blackboard between PopNextGoal and
// PickAndPlaceAction.
struct PlaceGoal {
    std::string color;
    int         block_id;
    double      place_x, place_y, place_z;
};

// Blackboard aliases
using BlockMap   = std::shared_ptr<std::map<std::string, BlockInfo>>;
using GoalQueue  = std::shared_ptr<std::queue<PlaceGoal>>;

// Target shape — edit to match your scene
static const std::map<std::string, std::array<double,3>> TARGET_SHAPE = {
    {"pos_A", {0.2,  -1.6, 0.25}},
    {"pos_B", {0.2,  -1.6, 0.3}},
    {"pos_C", {0.2,  -1.6, 0.35}},
};

// ── 1. WaitForBlocks ──────────────────────────────────────────────────────────
// Subscribes to /detected_blocks internally.
// Returns SUCCESS once expected_count distinct colours are seen.
// Writes a BlockMap to the blackboard.

class WaitForBlocksNode : public BT::StatefulActionNode
{
public:
    WaitForBlocksNode(const std::string& name,
                      const BT::NodeConfig& config,
                      rclcpp::Node::SharedPtr ros_node)
        : BT::StatefulActionNode(name, config), ros_node_(ros_node)
    {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<int>("expected_count", 3, "Blocks to wait for"),
            BT::OutputPort<BlockMap>("detected_blocks"),
        };
    }

    BT::NodeStatus onStart() override
    {
        expected_count_ = getInput<int>("expected_count").value();
        blocks_         = std::make_shared<std::map<std::string, BlockInfo>>();

        sub_ = ros_node_->create_subscription<robot_interfaces::msg::BlockPose>(
            "/detected_blocks", 10,
            [this](const robot_interfaces::msg::BlockPose::ConstSharedPtr msg) {
                std::lock_guard<std::mutex> lk(mu_);
                (*blocks_)[msg->color] = {
                    msg->pose.pose.position.x,
                    msg->pose.pose.position.y,
                    msg->pose.pose.position.z,
                    static_cast<int>(msg->block_id)
                };
                RCLCPP_INFO(ros_node_->get_logger(),
                    "WaitForBlocks: detected %s (%zu/%d)",
                    msg->color.c_str(), blocks_->size(), expected_count_);
            });

        RCLCPP_INFO(ros_node_->get_logger(),
            "WaitForBlocks: waiting for %d blocks...", expected_count_);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        std::lock_guard<std::mutex> lk(mu_);
        if (static_cast<int>(blocks_->size()) >= expected_count_) {
            setOutput("detected_blocks", blocks_);
            sub_.reset();
            RCLCPP_INFO(ros_node_->get_logger(),
                "WaitForBlocks: all blocks detected -> SUCCESS");
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override { sub_.reset(); }

private:
    rclcpp::Node::SharedPtr ros_node_;
    rclcpp::Subscription<robot_interfaces::msg::BlockPose>::SharedPtr sub_;
    std::mutex mu_;
    BlockMap blocks_;
    int expected_count_{3};
};

// ── 2. CallPlanner ────────────────────────────────────────────────────────────
// RosServiceNode that calls /get_plan.
// Reads BlockMap from blackboard, writes GoalQueue when done.

class CallPlannerNode : public BT::RosServiceNode<robot_interfaces::srv::GetPlan>
{
public:
    CallPlannerNode(const std::string& name,
                    const BT::NodeConfig& config,
                    const BT::RosNodeParams& params)
        : RosServiceNode<robot_interfaces::srv::GetPlan>(name, config, params)
    {}

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({
            BT::InputPort<BlockMap>("detected_blocks"),
            BT::OutputPort<GoalQueue>("goal_queue"),
        });
    }

    bool setRequest(Request::SharedPtr& req) override
    {
        auto blocks = getInput<BlockMap>("detected_blocks").value();

        int i = 1;
        name_to_color_.clear();
        for (auto& [color, info] : *blocks) {
            std::string block_name = "block" + std::to_string(i++);
            name_to_color_[block_name] = color;

            req->block_names.push_back(block_name);
            req->positions_x.push_back(info.x);
            req->positions_y.push_back(info.y);
            req->positions_z.push_back(info.z);
        }

        for (auto& [tname, xyz] : TARGET_SHAPE) {
            req->target_names.push_back(tname);
            req->targets_x.push_back(xyz[0]);
            req->targets_y.push_back(xyz[1]);
            req->targets_z.push_back(xyz[2]);
        }

        blocks_snapshot_ = blocks;

        RCLCPP_INFO(logger(), "CallPlanner: sending request with %zu blocks",
                    blocks->size());
        return true;
    }

    BT::NodeStatus onResponseReceived(const Response::SharedPtr& res) override
    {
        if (!res->success) {
            RCLCPP_ERROR(logger(), "CallPlanner: service returned failure: %s",
                         res->message.c_str());
            return BT::NodeStatus::FAILURE;
        }

        auto queue = std::make_shared<std::queue<PlaceGoal>>();

        for (const auto& action_str : res->plan_actions) {
            std::istringstream ss(action_str);
            std::string verb, arm, block_name, target_name;
            ss >> verb >> arm >> block_name >> target_name;

            if (verb != "place") continue;

            auto color_it = name_to_color_.find(block_name);
            if (color_it == name_to_color_.end()) {
                RCLCPP_WARN(logger(), "Unknown block name: %s", block_name.c_str());
                continue;
            }
            const std::string& color = color_it->second;

            auto target_it = TARGET_SHAPE.find(target_name);
            if (target_it == TARGET_SHAPE.end()) {
                RCLCPP_WARN(logger(), "Unknown target: %s", target_name.c_str());
                continue;
            }

            PlaceGoal goal;
            goal.color    = color;
            goal.block_id = (*blocks_snapshot_)[color].block_id;
            goal.place_x  = target_it->second[0];
            goal.place_y  = target_it->second[1];
            goal.place_z  = target_it->second[2];
            queue->push(goal);

            RCLCPP_INFO(logger(), "  queued: %s (%s) -> %s [%.2f %.2f %.2f]",
                block_name.c_str(), color.c_str(), target_name.c_str(),
                goal.place_x, goal.place_y, goal.place_z);
        }

        setOutput("goal_queue", queue);
        RCLCPP_INFO(logger(), "CallPlanner: %zu goals queued -> SUCCESS",
                    queue->size());
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus onFailure(BT::ServiceNodeErrorCode err) override
    {
        RCLCPP_ERROR(logger(), "CallPlanner: service error %d", (int)err);
        return BT::NodeStatus::FAILURE;
    }

private:
    std::map<std::string, std::string> name_to_color_;
    BlockMap blocks_snapshot_;
};

// ── 3. HasRemainingGoals ──────────────────────────────────────────────────────

class HasRemainingGoalsNode : public BT::ConditionNode
{
public:
    HasRemainingGoalsNode(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config)
    {}

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<GoalQueue>("goal_queue") };
    }

    BT::NodeStatus tick() override
    {
        auto queue = getInput<GoalQueue>("goal_queue").value();
        return queue && !queue->empty()
            ? BT::NodeStatus::SUCCESS
            : BT::NodeStatus::FAILURE;
    }
};

// ── 4. PopNextGoal ────────────────────────────────────────────────────────────

class PopNextGoalNode : public BT::SyncActionNode
{
public:
    PopNextGoalNode(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config)
    {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<GoalQueue>("goal_queue"),
            BT::OutputPort<std::string>("color"),
            BT::OutputPort<int>("block_id"),
            BT::OutputPort<double>("place_x"),
            BT::OutputPort<double>("place_y"),
            BT::OutputPort<double>("place_z"),
        };
    }

    BT::NodeStatus tick() override
    {
        auto queue = getInput<GoalQueue>("goal_queue").value();
        if (!queue || queue->empty())
            return BT::NodeStatus::FAILURE;

        const PlaceGoal& g = queue->front();
        setOutput("color",    g.color);
        setOutput("block_id", g.block_id);
        setOutput("place_x",  g.place_x);
        setOutput("place_y",  g.place_y);
        setOutput("place_z",  g.place_z);
        queue->pop();
        return BT::NodeStatus::SUCCESS;
    }
};

// ── 5. PickAndPlaceAction ─────────────────────────────────────────────────────

class PickAndPlaceActionNode : public BT::RosActionNode<robot_interfaces::action::PickAndPlace>
{
public:
    PickAndPlaceActionNode(const std::string& name,
                           const BT::NodeConfig& config,
                           const BT::RosNodeParams& params)
        : RosActionNode<robot_interfaces::action::PickAndPlace>(name, config, params)
    {}

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({
            BT::InputPort<std::string>("color"),
            BT::InputPort<int>("block_id"),
            BT::InputPort<double>("place_x"),
            BT::InputPort<double>("place_y"),
            BT::InputPort<double>("place_z"),
        });
    }

    bool setGoal(Goal& goal) override
    {
        goal.color    = getInput<std::string>("color").value();
        goal.block_id = static_cast<uint8_t>(getInput<int>("block_id").value());
        goal.place_x  = getInput<double>("place_x").value();
        goal.place_y  = getInput<double>("place_y").value();
        goal.place_z  = getInput<double>("place_z").value();

        RCLCPP_INFO(logger(),
            "PickAndPlace goal: color=%s id=%d place=[%.3f %.3f %.3f]",
            goal.color.c_str(), (int)goal.block_id,
            goal.place_x, goal.place_y, goal.place_z);
        return true;
    }

    BT::NodeStatus onResultReceived(const WrappedResult& wr) override
    {
        if (wr.result->success) {
            RCLCPP_INFO(logger(), "PickAndPlace SUCCESS: %s",
                        wr.result->message.c_str());
            return BT::NodeStatus::SUCCESS;
        }
        RCLCPP_ERROR(logger(), "PickAndPlace FAILURE: %s",
                     wr.result->message.c_str());
        return BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> fb) override
    {
        RCLCPP_INFO(logger(), "[Feedback] %s", fb->status.c_str());
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onFailure(BT::ActionNodeErrorCode err) override
    {
        RCLCPP_ERROR(logger(), "Action error: %d", (int)err);
        return BT::NodeStatus::FAILURE;
    }
};

// ── main ──────────────────────────────────────────────────────────────────────

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto ros_node = std::make_shared<rclcpp::Node>("bt_executor");

    BT::BehaviorTreeFactory factory;

    // WaitForBlocks needs the rclcpp node — pass it via lambda
    factory.registerBuilder<WaitForBlocksNode>(
        "WaitForBlocks",
        [ros_node](const std::string& name, const BT::NodeConfig& cfg) {
            return std::make_unique<WaitForBlocksNode>(name, cfg, ros_node);
        });

    factory.registerNodeType<HasRemainingGoalsNode>("HasRemainingGoals");
    factory.registerNodeType<PopNextGoalNode>("PopNextGoal");

    // Fix applied here: Completed the RosNodeParams registration
    BT::RosNodeParams service_params;
    service_params.nh = ros_node;
    service_params.default_port_value = "get_plan";
    service_params.server_timeout = std::chrono::seconds(10);
    factory.registerNodeType<CallPlannerNode>("CallPlanner", service_params); 

    BT::RosNodeParams action_params;
    action_params.nh = ros_node;
    action_params.default_port_value = "pick_and_place";
    action_params.server_timeout = std::chrono::seconds(10);
    factory.registerNodeType<PickAndPlaceActionNode>("PickAndPlaceAction", action_params); 

    std::string tree_xml_path;
    if (argc > 1) {
        tree_xml_path = argv[1];
    } else {
        tree_xml_path = "pick_and_place.xml";
    }

    RCLCPP_INFO(ros_node->get_logger(), "Loading tree from: %s", tree_xml_path.c_str());

    auto tree = factory.createTreeFromFile(tree_xml_path);
    BT::StdCoutLogger logger(tree);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(ros_node);

    // Spin ROS in background so topic callbacks fire during BT ticks
    auto spin_thread = std::thread([&executor]() { executor.spin(); });

    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
        status = tree.tickOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    RCLCPP_INFO(ros_node->get_logger(), "Tree finished with status: %s",
        status == BT::NodeStatus::SUCCESS ? "SUCCESS" : "FAILURE");

    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return status == BT::NodeStatus::SUCCESS ? 0 : 1;
}
