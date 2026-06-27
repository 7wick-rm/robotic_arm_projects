// #include <rclcpp/rclcpp.hpp>
// #include <moveit/planning_scene/planning_scene.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <moveit/task_constructor/task.h>
// #include <moveit/task_constructor/solvers.h>
// #include <moveit/task_constructor/stages.h>
// #if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #else
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #endif
// #if __has_include(<tf2_eigen/tf2_eigen.hpp>)
// #include <tf2_eigen/tf2_eigen.hpp>
// #else
// #include <tf2_eigen/tf2_eigen.h>
// #endif

// // ── NEW: perception subscription ──────────────────────────────────────────────
// #include "perception/msg/block_pose.hpp"
// #include <map>
// #include <string>
// #include <mutex>
// // ─────────────────────────────────────────────────────────────────────────────

// static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
// namespace mtc = moveit::task_constructor;

// // ── OFFSET TUNING: adjust these after receiving pose from perception ──────────
// // Applied in setupPlanningScene() before passing to MoveIt.
// // Positive z = move collision object UP, negative = DOWN.
// static constexpr double PERCEPTION_X_OFFSET = 0.0;
// static constexpr double PERCEPTION_Y_OFFSET = 0.0;
// static constexpr double PERCEPTION_Z_OFFSET = -0.025;   // ← tune this
// // ─────────────────────────────────────────────────────────────────────────────

// class MTCTaskNode
// {
// public:
//   MTCTaskNode(const rclcpp::NodeOptions& options);

//   rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
//   geometry_msgs::msg::Pose object_pose_;

//   void doTask();
//   void setupPlanningScene();

//   // ── NEW ───────────────────────────────────────────────────────────────────
//   bool hasDetectedBlocks() const;
//   // ─────────────────────────────────────────────────────────────────────────

// private:
//   mtc::Task createTask();
//   mtc::Task task_;
//   rclcpp::Node::SharedPtr node_;

//   // ── NEW ───────────────────────────────────────────────────────────────────
//   void blockPoseCallback(const perception::msg::BlockPose::SharedPtr msg);
//   rclcpp::Subscription<perception::msg::BlockPose>::SharedPtr block_sub_;
//   mutable std::mutex blocks_mutex_;
//   std::map<std::string, geometry_msgs::msg::PoseStamped> detected_blocks_;
//   // ─────────────────────────────────────────────────────────────────────────
// };

// MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
//   : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
// {
//   // ── NEW: subscribe to perception ──────────────────────────────────────────
//   block_sub_ = node_->create_subscription<perception::msg::BlockPose>(
//       "/detected_blocks", 10,
//       std::bind(&MTCTaskNode::blockPoseCallback, this, std::placeholders::_1));
//   RCLCPP_INFO(LOGGER, "Waiting for block detections on /detected_blocks ...");
//   // ─────────────────────────────────────────────────────────────────────────
// }

// rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
// {
//   return node_->get_node_base_interface();
// }

// // ── NEW: perception callback ──────────────────────────────────────────────────
// void MTCTaskNode::blockPoseCallback(const perception::msg::BlockPose::SharedPtr msg)
// {
//   std::lock_guard<std::mutex> lock(blocks_mutex_);
//   detected_blocks_[msg->color] = msg->pose;
//   RCLCPP_INFO(LOGGER, "Detected block | color: %s | x:%.3f y:%.3f z:%.3f",
//               msg->color.c_str(),
//               msg->pose.pose.position.x,
//               msg->pose.pose.position.y,
//               msg->pose.pose.position.z);
// }

// bool MTCTaskNode::hasDetectedBlocks() const
// {
//   std::lock_guard<std::mutex> lock(blocks_mutex_);
//   return !detected_blocks_.empty();
// }
// // ─────────────────────────────────────────────────────────────────────────────

// // ── MODIFIED: uses perception pose instead of hardcoded values ────────────────
// // Dimensions, object id, frame_id — all unchanged.
// void MTCTaskNode::setupPlanningScene()
// {
//   moveit_msgs::msg::CollisionObject object;
//   object.id = "object";
//   object.header.frame_id = "base_link";
//   object.primitives.resize(1);
//   object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
//   object.primitives[0].dimensions = { 0.12, 0.05, 0.1 };  // unchanged

//   geometry_msgs::msg::Pose pose;

//   // ── NEW: read from perception, fall back to hardcoded if nothing received ──
//   {
//     std::lock_guard<std::mutex> lock(blocks_mutex_);
//     auto it = detected_blocks_.find("R");   // ← change "R" to "G" or "B" as needed
//     if (it != detected_blocks_.end())
//     {
//       const auto& p = it->second.pose.position;
//       pose.position.x = p.x + PERCEPTION_X_OFFSET;
//       pose.position.y = p.y + PERCEPTION_Y_OFFSET;
//       pose.position.z = p.z + PERCEPTION_Z_OFFSET;   // ← offset applied here
//       RCLCPP_INFO(LOGGER, "Using perception pose: x:%.3f y:%.3f z:%.3f",
//                   pose.position.x, pose.position.y, pose.position.z);
//     }
//     else
//     {
//       RCLCPP_WARN(LOGGER, "No perception data for 'R' — using fallback hardcoded pose");
//       pose.position.x = 0.0;
//       pose.position.y = -1.2;
//       pose.position.z = 0.02;
//     }
//   }
//   // ─────────────────────────────────────────────────────────────────────────

//   pose.orientation.w = 1.0;
//   object.pose = pose;

//   moveit::planning_interface::PlanningSceneInterface psi;
//   psi.applyCollisionObject(object);
// }

// // ── EVERYTHING BELOW IS COMPLETELY UNCHANGED FROM YOUR ORIGINAL ───────────────

// void MTCTaskNode::doTask()
// {
//   task_ = createTask();

//   try
//   {
//     task_.init();
//   }
//   catch (mtc::InitStageException& e)
//   {
//     RCLCPP_ERROR_STREAM(LOGGER, e);
//     return;
//   }

//   if (!task_.plan(8))
//   {
//     RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
//     return;
//   }
//   task_.introspection().publishSolution(*task_.solutions().front());

//   auto result = task_.execute(*task_.solutions().front());
//   if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
//   {
//     RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
//     return;
//   }

//   return;
// }

// mtc::Task MTCTaskNode::createTask()
// {
//   mtc::Task task;
//   task.stages()->setName("demo task");
//   task.loadRobotModel(node_);

//   const auto& arm_group_name = "arm";
//   const auto& hand_group_name = "gripper";
//   const auto& hand_frame = "gripper_withoutarm_part";

//   task.setProperty("group", arm_group_name);
//   task.setProperty("eef", "end_effector");
//   task.setProperty("ik_frame", hand_frame);

// #pragma GCC diagnostic push
// #pragma GCC diagnostic ignored "-Wunused-but-set-variable"
//   mtc::Stage* current_state_ptr = nullptr; 
//   mtc::Stage* attach_object_stage =nullptr;
// #pragma GCC diagnostic pop

//   auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
//   current_state_ptr = stage_state_current.get();
//   task.add(std::move(stage_state_current));

//   auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
//   auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

//   auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
//   cartesian_planner->setMaxVelocityScalingFactor(0.5);
//   cartesian_planner->setMaxAccelerationScalingFactor(0.5);
//   cartesian_planner->setStepSize(.01);

//   auto stage_open_hand =
//       std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
//   stage_open_hand->setGroup(hand_group_name);
//   stage_open_hand->setGoal("gripper_open");
//   task.add(std::move(stage_open_hand));

//   auto connect = std::make_unique<mtc::stages::Connect>(
//       "move to grasp",
//       mtc::stages::Connect::GroupPlannerVector{
//           { arm_group_name, sampling_planner }
//       });
//   connect->setTimeout(5.0);
//   connect->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
//   task.add(std::move(connect));

// {
// auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
// task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
// grasp->properties().configureInitFrom(mtc::Stage::PARENT,
//                                         { "eef", "group", "ik_frame" });
// {
// auto gen = std::make_unique<mtc::stages::GenerateGraspPose>("generate move towards");
// gen->properties().configureInitFrom(mtc::Stage::PARENT);
// gen->setObject("object");
// gen->setPreGraspPose("gripper_open");
// gen->setAngleDelta(M_PI/2);
// gen->setMonitoredStage(current_state_ptr);

// Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
// tf.translation() = Eigen::Vector3d(-0.25, 0.0, 0.08);
// tf.linear() =
//   Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()).toRotationMatrix();
// auto wrapper= std::make_unique<mtc::stages::ComputeIK>("compute ik",std::move(gen));
// wrapper->setIKFrame(tf,"tcp");
// wrapper->setMaxIKSolutions(8);
// wrapper->setTimeout(2.0);
// wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
// wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
// grasp->insert(std::move(wrapper));
// }
// {
// auto approach = std::make_unique<mtc::stages::MoveRelative>("approach", cartesian_planner);
// approach->setMinMaxDistance(0.20, 0.30);
// approach->properties().set("link", hand_frame);
// approach->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
// geometry_msgs::msg::Vector3Stamped vec;
// vec.header.frame_id = "base_link";
// vec.vector.z = -1.0;  
// approach->setDirection(vec);
// grasp->insert(std::move(approach));
// }

// {
//   auto stage =
//       std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
//   stage->allowCollisions("object",
//                         task.getRobotModel()
//                             ->getJointModelGroup(hand_group_name)
//                             ->getLinkModelNamesWithCollisionGeometry(),
//                         true);
//   grasp->insert(std::move(stage));
// }

// {
//   auto close=std::make_unique<mtc::stages::MoveTo>("close hand",interpolation_planner);
//   close->setGroup(hand_group_name);
//   close->setGoal("gripper_close");
//   grasp->insert(std::move(close));
// }

// {
//     auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
//     stage->attachObject("object", "tcp");
//     attach_object_stage = stage.get();
//     grasp->insert(std::move(stage));
//  }

// {
//   auto stage =
//             std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
//   stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
//   stage->setMinMaxDistance(0.20, 0.30); 
//   stage->setIKFrame(hand_frame);
//   stage->properties().set("link", hand_frame);
//   geometry_msgs::msg::Vector3Stamped vec;
//   vec.header.frame_id = "world";
//   vec.vector.z = 1.0;
//   stage->setDirection(vec);
//   grasp->insert(std::move(stage));
// }

// task.add(std::move(grasp));
// }

// {
//     auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
//       "move to place",
//       mtc::stages::Connect::GroupPlannerVector{
//           { arm_group_name, sampling_planner }
//       });
//   stage_move_to_place->setTimeout(5.0);
//   stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
//   task.add(std::move(stage_move_to_place));
// }
// {
//   auto place=std::make_unique<mtc::SerialContainer>("place object");
//   task.properties().exposeTo(place->properties(),{"eef","group","ik_frame"});
//   place->properties().configureInitFrom(mtc::Stage::PARENT,{ "eef", "group", "ik_frame" });
//   {
//     auto generate_place=std::make_unique<mtc::stages::GeneratePlacePose>("generate place position");
//     generate_place->properties().configureInitFrom(mtc::Stage::PARENT);
//     generate_place->setObject("object");
//     geometry_msgs::msg::PoseStamped p;
//     p.header.frame_id="object";
//     p.pose.position.y=0.25;
//     p.pose.orientation.w=1.0;
//     generate_place->setPose(p);
//     generate_place->setMonitoredStage(attach_object_stage);

//     auto wrapper=std::make_unique<mtc::stages::ComputeIK>("place pose Ik",std::move(generate_place));
//     wrapper->setMaxIKSolutions(8);
//     wrapper->setMinSolutionDistance(0.5);
//     wrapper->setTimeout(2.0);
//     wrapper->setIKFrame("object");
//     wrapper->properties().configureInitFrom(mtc::Stage::PARENT,{"eef","group"});
//     wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE,{"target_pose"});
//     place->insert(std::move(wrapper));
//   }

//   {
//      auto stage=std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
//      stage->setGroup(hand_group_name);
//     stage->setGoal("gripper_open");
//     place->insert(std::move(stage));
//    }
//    {
//     auto stage=std::make_unique<mtc::stages::ModifyPlanningScene>("disable collisions");
//      stage->allowCollisions("object",
//                         task.getRobotModel()
//                            ->getJointModelGroup(hand_group_name)
//                             ->getLinkModelNamesWithCollisionGeometry(),
//                         false);
//     place->insert(std::move(stage));
//     }

// {
//   auto stage=std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
//   stage->detachObject("object","tcp");
//   place->insert(std::move(stage));
// }

// {
//   auto moveup=std::make_unique<mtc::stages::MoveRelative>("move up",cartesian_planner);
//   moveup->properties().configureInitFrom(mtc::Stage::PARENT,{"group"});
//   moveup->setMinMaxDistance(0.20,0.30);
//   moveup->setIKFrame(hand_frame);
//   geometry_msgs::msg::Vector3Stamped v;
//   v.header.frame_id="base_link";
//   v.vector.z=1.0;
//   moveup->setDirection(v);
//   place->insert(std::move(moveup));
// }
// task.add(std::move(place));
// }
// {
//   auto stage = std::make_unique<mtc::stages::MoveTo>("return home", sampling_planner);
//   stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
//   stage->setGoal("home");
//   task.add(std::move(stage));
// }

//   return task;
// }

// int main(int argc, char** argv)
// {
//   rclcpp::init(argc, argv);

//   rclcpp::NodeOptions options;
//   options.automatically_declare_parameters_from_overrides(true);

//   auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
//   rclcpp::executors::MultiThreadedExecutor executor;

//   auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
//     executor.add_node(mtc_task_node->getNodeBaseInterface());
//     executor.spin();
//     executor.remove_node(mtc_task_node->getNodeBaseInterface());
//   });

//   // ── NEW: wait for at least one detection before planning ──────────────────
//   while (rclcpp::ok() && !mtc_task_node->hasDetectedBlocks())
//   {
//     rclcpp::sleep_for(std::chrono::milliseconds(200));
//   }
//   rclcpp::sleep_for(std::chrono::seconds(1));  // let more detections accumulate
//   // ─────────────────────────────────────────────────────────────────────────

//   mtc_task_node->setupPlanningScene();
//   rclcpp::sleep_for(std::chrono::seconds(1)); 
//   mtc_task_node->doTask();
//   rclcpp::sleep_for(std::chrono::seconds(30)); 

//   spin_thread->join();
//   rclcpp::shutdown();
//   return 0;
// }











// #include <rclcpp/rclcpp.hpp>
// #include <rclcpp_action/rclcpp_action.hpp>          // ── NEW
// #include <moveit/planning_scene/planning_scene.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <moveit/task_constructor/task.h>
// #include <moveit/task_constructor/solvers.h>
// #include <moveit/task_constructor/stages.h>
// #if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #else
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #endif
// #if __has_include(<tf2_eigen/tf2_eigen.hpp>)
// #include <tf2_eigen/tf2_eigen.hpp>
// #else
// #include <tf2_eigen/tf2_eigen.h>
// #endif

// #include "perception/msg/block_pose.hpp"
// // ── NEW: replace "your_package" with your actual package name ─────────────────
// #include "mtc_tutorial/action/pick_and_place.hpp"
// // ─────────────────────────────────────────────────────────────────────────────

// #include <map>
// #include <string>
// #include <mutex>
// #include <thread>

// static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
// namespace mtc = moveit::task_constructor;

// static constexpr double PERCEPTION_X_OFFSET = 0.0;
// static constexpr double PERCEPTION_Y_OFFSET = 0.0;
// static constexpr double PERCEPTION_Z_OFFSET = 0.015;

// class MTCTaskNode
// {
// public:
//   using PickAndPlace        = mtc_tutorial::action::PickAndPlace;
//   using GoalHandlePAP       = rclcpp_action::ServerGoalHandle<PickAndPlace>;

//   MTCTaskNode(const rclcpp::NodeOptions& options);

//   rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
//   geometry_msgs::msg::Pose object_pose_;

//   bool hasDetectedBlocks() const;

// private:
//   // ── Perception ────────────────────────────────────────────────────────────
//   void blockPoseCallback(const perception::msg::BlockPose::SharedPtr msg);
//   rclcpp::Subscription<perception::msg::BlockPose>::SharedPtr block_sub_;
//   mutable std::mutex blocks_mutex_;
//   std::map<std::string, geometry_msgs::msg::PoseStamped> detected_blocks_;

//   // ── Action server ─────────────────────────────────────────────────────────
//   rclcpp_action::Server<PickAndPlace>::SharedPtr action_server_;

//   // Called immediately when a goal arrives — we accept everything
//   rclcpp_action::GoalResponse handleGoal(
//     const rclcpp_action::GoalUUID& uuid,
//     std::shared_ptr<const PickAndPlace::Goal> goal);

//   // Called if the client requests cancellation
//   rclcpp_action::CancelResponse handleCancel(
//     const std::shared_ptr<GoalHandlePAP> goal_handle);

//   // Called after goal is accepted — runs in its own thread so it doesn't
//   // block the executor (important: MTC planning is slow)
//   void handleAccepted(const std::shared_ptr<GoalHandlePAP> goal_handle);

//   // The actual pick-and-place logic, called from handleAccepted's thread
//   void execute(const std::shared_ptr<GoalHandlePAP> goal_handle);

//   // ── MTC helpers (signatures changed, stages untouched) ───────────────────
//   void setupPlanningScene(const std::string& color);
//   void doTask(const std::string& color,
//               double place_x, double place_y, double place_z,
//               const std::shared_ptr<GoalHandlePAP>& goal_handle);
//   mtc::Task createTask(const std::string& color,
//                        double place_x, double place_y, double place_z);

//   mtc::Task task_;
//   rclcpp::Node::SharedPtr node_;
// };

// // ─────────────────────────────────────────────────────────────────────────────
// MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
//   : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
// {
//   // Perception subscription
//   block_sub_ = node_->create_subscription<perception::msg::BlockPose>(
//       "/detected_blocks", 10,
//       std::bind(&MTCTaskNode::blockPoseCallback, this, std::placeholders::_1));

//   // ── NEW: action server ────────────────────────────────────────────────────
//   action_server_ = rclcpp_action::create_server<PickAndPlace>(
//       node_,
//       "pick_and_place",
//       std::bind(&MTCTaskNode::handleGoal,     this, std::placeholders::_1, std::placeholders::_2),
//       std::bind(&MTCTaskNode::handleCancel,   this, std::placeholders::_1),
//       std::bind(&MTCTaskNode::handleAccepted, this, std::placeholders::_1));
//   // ─────────────────────────────────────────────────────────────────────────

//   RCLCPP_INFO(LOGGER, "Action server ready — send goals to 'pick_and_place'.");
// }

// rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
// {
//   return node_->get_node_base_interface();
// }

// // ── Perception callback ───────────────────────────────────────────────────────
// void MTCTaskNode::blockPoseCallback(const perception::msg::BlockPose::SharedPtr msg)
// {
//   std::lock_guard<std::mutex> lock(blocks_mutex_);
//   detected_blocks_[msg->color] = msg->pose;
//   RCLCPP_INFO(LOGGER, "Detected block | color: %s | x:%.3f y:%.3f z:%.3f",
//               msg->color.c_str(),
//               msg->pose.pose.position.x,
//               msg->pose.pose.position.y,
//               msg->pose.pose.position.z);
// }

// bool MTCTaskNode::hasDetectedBlocks() const
// {
//   std::lock_guard<std::mutex> lock(blocks_mutex_);
//   return !detected_blocks_.empty();
// }

// // ── Action server callbacks ───────────────────────────────────────────────────

// // 1. Validate and accept/reject the incoming goal
// rclcpp_action::GoalResponse MTCTaskNode::handleGoal(
//   const rclcpp_action::GoalUUID& /*uuid*/,
//   std::shared_ptr<const PickAndPlace::Goal> goal)
// {
//   RCLCPP_INFO(LOGGER,
//               "Goal received | color: %s | block_id: %d | place: [%.3f, %.3f, %.3f]",
//               goal->color.c_str(),
//               static_cast<int>(goal->block_id),
//               goal->place_x, goal->place_y, goal->place_z);

//   // Reject if no detection for the requested color yet
//   std::lock_guard<std::mutex> lock(blocks_mutex_);
//   if (detected_blocks_.find(goal->color) == detected_blocks_.end())
//   {
//     RCLCPP_WARN(LOGGER, "Rejecting goal — no block detected for color '%s'",
//                 goal->color.c_str());
//     return rclcpp_action::GoalResponse::REJECT;
//   }

//   return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
// }

// // 2. Handle cancel request
// rclcpp_action::CancelResponse MTCTaskNode::handleCancel(
//   const std::shared_ptr<GoalHandlePAP> /*goal_handle*/)
// {
//   RCLCPP_INFO(LOGGER, "Cancel request received");
//   return rclcpp_action::CancelResponse::ACCEPT;
// }

// // 3. Spin off execution in a new thread so the executor isn't blocked
// void MTCTaskNode::handleAccepted(const std::shared_ptr<GoalHandlePAP> goal_handle)
// {
//   std::thread{ std::bind(&MTCTaskNode::execute, this, goal_handle) }.detach();
// }

// // 4. The actual work — runs in its own thread
// void MTCTaskNode::execute(const std::shared_ptr<GoalHandlePAP> goal_handle)
// {
//   const auto goal = goal_handle->get_goal();
//   auto feedback   = std::make_shared<PickAndPlace::Feedback>();
//   auto result     = std::make_shared<PickAndPlace::Result>();

//   // ── Feedback helper lambda ────────────────────────────────────────────────
//   auto send_feedback = [&](const std::string& status) {
//     feedback->status = status;
//     goal_handle->publish_feedback(feedback);
//     RCLCPP_INFO(LOGGER, "[Feedback] %s", status.c_str());
//   };

//   // ── Step 1: set up planning scene ─────────────────────────────────────────
//   send_feedback("Setting up planning scene");
//   setupPlanningScene(goal->color);
//   rclcpp::sleep_for(std::chrono::seconds(1));

//   // ── Step 2: run MTC task ───────────────────────────────────────────────────
//   send_feedback("Planning and executing pick and place");
//   doTask(goal->color, goal->place_x, goal->place_y, goal->place_z, goal_handle);

//   // ── Step 3: send result ────────────────────────────────────────────────────
//   if (goal_handle->is_canceling())
//   {
//     result->success = false;
//     result->message = "Goal cancelled";
//     goal_handle->canceled(result);
//     return;
//   }

//   send_feedback("Done");
//   result->success = true;
//   result->message = "Pick and place completed successfully";
//   goal_handle->succeed(result);
// }

// // ── Planning scene (color param, not hardcoded) ───────────────────────────────
// void MTCTaskNode::setupPlanningScene(const std::string& color)
// {
//   moveit_msgs::msg::CollisionObject object;
//   object.id = "object";
//   object.header.frame_id = "base_link";
//   object.primitives.resize(1);
//   object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
//   object.primitives[0].dimensions = { 0.12, 0.05, 0.1 };

//   geometry_msgs::msg::Pose pose;

//   {
//     std::lock_guard<std::mutex> lock(blocks_mutex_);
//     auto it = detected_blocks_.find(color);
//     if (it != detected_blocks_.end())
//     {
//       const auto& p = it->second.pose.position;
//       pose.position.x = p.x + PERCEPTION_X_OFFSET;
//       pose.position.y = p.y + PERCEPTION_Y_OFFSET;
//       pose.position.z = p.z + PERCEPTION_Z_OFFSET;
//       RCLCPP_INFO(LOGGER, "Using perception pose: x:%.3f y:%.3f z:%.3f",
//                   pose.position.x, pose.position.y, pose.position.z);
//     }
//     else
//     {
//       RCLCPP_WARN(LOGGER, "No perception data for '%s' — using fallback", color.c_str());
//       pose.position.x = 0.0;
//       pose.position.y = -1.2;
//       pose.position.z = 0.02;
//     }
//   }

//   pose.orientation.w = 1.0;
//   object.pose = pose;

//   moveit::planning_interface::PlanningSceneInterface psi;
//   psi.applyCollisionObject(object);
// }

// // ── doTask: passes goal_handle so execute() can check for cancellation ─────────
// void MTCTaskNode::doTask(const std::string& color,
//                          double place_x, double place_y, double place_z,
//                          const std::shared_ptr<GoalHandlePAP>& goal_handle)
// {
//   task_ = createTask(color, place_x, place_y, place_z);

//   try { task_.init(); }
//   catch (mtc::InitStageException& e)
//   {
//     RCLCPP_ERROR_STREAM(LOGGER, e);
//     return;
//   }

//   if (goal_handle->is_canceling()) return;

//   if (!task_.plan(8))
//   {
//     RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
//     return;
//   }

//   if (goal_handle->is_canceling()) return;

//   task_.introspection().publishSolution(*task_.solutions().front());

//   auto result = task_.execute(*task_.solutions().front());
//   if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
//     RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
// }

// // ── ALL STAGES COMPLETELY UNCHANGED (only place pose values updated) ──────────
// mtc::Task MTCTaskNode::createTask(const std::string& color,
//                                   double place_x, double place_y, double place_z)
// {
//   (void)color;

//   mtc::Task task;
//   task.stages()->setName("demo task");
//   task.loadRobotModel(node_);

//   const auto& arm_group_name = "arm";
//   const auto& hand_group_name = "gripper";
//   const auto& hand_frame = "tcp";

//   task.setProperty("group", arm_group_name);
//   task.setProperty("eef", "end_effector");
//   task.setProperty("ik_frame", hand_frame);

// #pragma GCC diagnostic push
// #pragma GCC diagnostic ignored "-Wunused-but-set-variable"
//   mtc::Stage* current_state_ptr = nullptr; 
//   mtc::Stage* attach_object_stage = nullptr;
// #pragma GCC diagnostic pop

//   auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
//   current_state_ptr = stage_state_current.get();
//   task.add(std::move(stage_state_current));

//   auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
//   auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

//   auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
//   cartesian_planner->setMaxVelocityScalingFactor(0.5);
//   cartesian_planner->setMaxAccelerationScalingFactor(0.5);
//   cartesian_planner->setStepSize(.01);

//   auto stage_open_hand =
//       std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
//   stage_open_hand->setGroup(hand_group_name);
//   stage_open_hand->setGoal("gripper_open");
//   task.add(std::move(stage_open_hand));

//   auto connect = std::make_unique<mtc::stages::Connect>(
//       "move to grasp",
//       mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
//   connect->setTimeout(5.0);
//   connect->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
//   task.add(std::move(connect));

// {
// auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
// task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
// grasp->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });
// {
// auto gen = std::make_unique<mtc::stages::GenerateGraspPose>("generate move towards");
// gen->properties().configureInitFrom(mtc::Stage::PARENT);
// gen->setObject("object");
// gen->setPreGraspPose("gripper_open");
// gen->setAngleDelta(M_PI/2);
// gen->setMonitoredStage(current_state_ptr);

// Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
// tf.translation() = Eigen::Vector3d(-0.25, 0.0, 0.08);
// tf.linear() = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()).toRotationMatrix();
// auto wrapper = std::make_unique<mtc::stages::ComputeIK>("compute ik", std::move(gen));
// wrapper->setIKFrame(tf, "tcp");
// wrapper->setMaxIKSolutions(8);
// wrapper->setTimeout(2.0);
// wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
// wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
// grasp->insert(std::move(wrapper));
// }
// {
// auto approach = std::make_unique<mtc::stages::MoveRelative>("approach", cartesian_planner);
// approach->setMinMaxDistance(0.20, 0.30);
// approach->properties().set("link", hand_frame);
// approach->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
// geometry_msgs::msg::Vector3Stamped vec;
// vec.header.frame_id = "base_link";
// vec.vector.z = -1.0;
// approach->setDirection(vec);
// grasp->insert(std::move(approach));
// }
// {
//   auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
//   stage->allowCollisions("object",
//                          task.getRobotModel()
//                              ->getJointModelGroup(hand_group_name)
//                              ->getLinkModelNamesWithCollisionGeometry(),
//                          true);
//   grasp->insert(std::move(stage));
// }
// {
//   auto close = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
//   close->setGroup(hand_group_name);
//   close->setGoal("gripper_close");
//   grasp->insert(std::move(close));
// }
// {
//   auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
//   stage->attachObject("object", "tcp");
//   attach_object_stage = stage.get();
//   grasp->insert(std::move(stage));
// }
// {
//   auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
//   stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
//   stage->setMinMaxDistance(0.20, 0.30);
//   stage->setIKFrame(hand_frame);
//   stage->properties().set("link", hand_frame);
//   geometry_msgs::msg::Vector3Stamped vec;
//   vec.header.frame_id = "world";
//   vec.vector.z = 1.0;
//   stage->setDirection(vec);
//   grasp->insert(std::move(stage));
// }
// task.add(std::move(grasp));
// }

// {
//   auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
//       "move to place",
//       mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
//   stage_move_to_place->setTimeout(5.0);
//   stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
//   task.add(std::move(stage_move_to_place));
// }
// {
//   auto place = std::make_unique<mtc::SerialContainer>("place object");
//   task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
//   place->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });
//   {
//     auto generate_place = std::make_unique<mtc::stages::GeneratePlacePose>("generate place position");
//     generate_place->properties().configureInitFrom(mtc::Stage::PARENT);
//     generate_place->setObject("object");

//     // ── Only change inside stages: place pose from goal instead of hardcoded ─
//     geometry_msgs::msg::PoseStamped p;
//     p.header.frame_id    = "base_link";
//     p.pose.position.x    = place_x;
//     p.pose.position.y    = place_y;
//     p.pose.position.z    = place_z;
//     p.pose.orientation.w = 1.0;
//     // ─────────────────────────────────────────────────────────────────────────

//     generate_place->setPose(p);
//     generate_place->setMonitoredStage(attach_object_stage);

//     auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place pose Ik", std::move(generate_place));
//     wrapper->setMaxIKSolutions(8);
//     wrapper->setMinSolutionDistance(0.5);
//     wrapper->setTimeout(2.0);
//     wrapper->setIKFrame("object");
//     wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
//     wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
//     place->insert(std::move(wrapper));
//   }
//   {
//     auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
//     stage->setGroup(hand_group_name);
//     stage->setGoal("gripper_open");
//     place->insert(std::move(stage));
//   }
//   {
//     auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("disable collisions");
//     stage->allowCollisions("object",
//                            task.getRobotModel()
//                                ->getJointModelGroup(hand_group_name)
//                                ->getLinkModelNamesWithCollisionGeometry(),
//                            false);
//     place->insert(std::move(stage));
//   }
//   {
//     auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
//     stage->detachObject("object", "tcp");
//     place->insert(std::move(stage));
//   }
//   {
//     auto moveup = std::make_unique<mtc::stages::MoveRelative>("move up", cartesian_planner);
//     moveup->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
//     moveup->setMinMaxDistance(0.20, 0.30);
//     moveup->setIKFrame(hand_frame);
//     geometry_msgs::msg::Vector3Stamped v;
//     v.header.frame_id = "base_link";
//     v.vector.z = 1.0;
//     moveup->setDirection(v);
//     place->insert(std::move(moveup));
//   }
//   task.add(std::move(place));
// }
// {
//   auto stage = std::make_unique<mtc::stages::MoveTo>("return home", sampling_planner);
//   stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
//   stage->setGoal("home");
//   task.add(std::move(stage));
// }

//   return task;
// }

// // ── main: just spins, execution is entirely driven by action goals ─────────────
// int main(int argc, char** argv)
// {
//   rclcpp::init(argc, argv);

//   rclcpp::NodeOptions options;
//   options.automatically_declare_parameters_from_overrides(true);

//   auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
//   rclcpp_action::Server<MTCTaskNode::PickAndPlace>::SharedPtr dummy; // suppress unused warning

//   rclcpp::executors::MultiThreadedExecutor executor;

//   auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
//     executor.add_node(mtc_task_node->getNodeBaseInterface());
//     executor.spin();
//     executor.remove_node(mtc_task_node->getNodeBaseInterface());
//   });

//   spin_thread->join();
//   rclcpp::shutdown();
//   return 0;
// }













#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>

#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();

  void setupPlanningScene();

private:
  mtc::Task createTask();

  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

void MTCTaskNode::setupPlanningScene()
{
  // ── Frame note ────────────────────────────────────────────────────────────
  // base_link is the URDF root link (no parent joint), so it sits exactly at
  // the Gazebo spawn pose from gazebo_launch.py:
  //   world -> base_link = (x=0.5, y=0.35, z=0.0), no rotation.
  // Every pose below is taken directly from empty.world and converted from
  // "world" to "base_link" by subtracting that offset, so the MTC planning
  // scene matches the simulated scene exactly.

  // ── SHELF: matches storage_shelf_unit in empty.world ─────────────────────
  // World model pose: (1.0, 0.0, 0.0)  ->  base_link: (0.5, -0.35, 0.0)
  // Two boards 0.9 x 0.4 x 0.02, at local z=0.05 (bottom) and z=0.85 (top),
  // plus 4 corner posts at local (±0.43, ±0.18), full shelf height 0.9 m,
  // post center z=0.45 (radius 0.02) — all taken verbatim from the SDF.
  const double shelf_origin_x = 0.5;   // 1.0 - 0.5
  const double shelf_origin_y = -0.35; // 0.0 - 0.35
  const double shelf_origin_z = 0.0;

  moveit_msgs::msg::CollisionObject shelf_low;
  shelf_low.id              = "shelflow";
  shelf_low.header.frame_id = "base_link";
  shelf_low.primitives.resize(1);
  shelf_low.primitives[0].type       = shape_msgs::msg::SolidPrimitive::BOX;
  shelf_low.primitives[0].dimensions = { 0.9, 0.4, 0.02 };

  geometry_msgs::msg::Pose shelf_low_pose;
  shelf_low_pose.position.x    = shelf_origin_x;
  shelf_low_pose.position.y    = shelf_origin_y;
  shelf_low_pose.position.z    = shelf_origin_z + 0.05;
  shelf_low_pose.orientation.w = 1.0;
  shelf_low.pose = shelf_low_pose;

  moveit_msgs::msg::CollisionObject shelf_up;
  shelf_up.id              = "shelfup";
  shelf_up.header.frame_id = "base_link";
  shelf_up.primitives.resize(1);
  shelf_up.primitives[0].type       = shape_msgs::msg::SolidPrimitive::BOX;
  shelf_up.primitives[0].dimensions = { 0.9, 0.4, 0.02 };

  geometry_msgs::msg::Pose shelf_up_pose;
  shelf_up_pose.position.x    = shelf_origin_x;
  shelf_up_pose.position.y    = shelf_origin_y;
  shelf_up_pose.position.z    = shelf_origin_z + 0.85;
  shelf_up_pose.orientation.w = 1.0;
  shelf_up.pose = shelf_up_pose;

  // ── PICK TARGET: shelf_block_green from empty.world ──────────────────────
  // World pose: (1.0, 0.0, 0.095) -> base_link: (0.5, -0.35, 0.095)
  // Box size 0.03 x 0.03 x 0.1 (matches the SDF collision geometry exactly).
  // Reach check: radius ~ sqrt(0.5^2 + 0.35^2) ≈ 0.61 m — at the outer edge
  // of a ~0.5-0.6 m reach arm; if IK fails consistently, this is the first
  // number to revisit.
  moveit_msgs::msg::CollisionObject object;
  object.id              = "object";
  object.header.frame_id = "base_link";
  object.primitives.resize(1);
  object.primitives[0].type       = shape_msgs::msg::SolidPrimitive::BOX;
  object.primitives[0].dimensions = { 0.03, 0.03, 0.1 };

  geometry_msgs::msg::Pose pose;
  pose.position.x    = 0.5;
  pose.position.y    = -0.3;
  pose.position.z    = 0.09;
  pose.orientation.w = 1.0;
  object.pose = pose;

  // ── Non-target blocks (red, blue): added as plain collision obstacles so
  // the planner avoids them, exactly as they sit in empty.world. ───────────
  // moveit_msgs::msg::CollisionObject block_red;
  // block_red.id              = "shelf_block_red";
  // block_red.header.frame_id = "base_link";
  // block_red.primitives.resize(1);
  // block_red.primitives[0].type       = shape_msgs::msg::SolidPrimitive::BOX;
  // block_red.primitives[0].dimensions = { 0.03, 0.03, 0.1 };
  // block_red.pose.position.x    = 0.4;   // 0.9 - 0.5
  // block_red.pose.position.y   = -0.35;  // 0.0 - 0.35
  // block_red.pose.position.z    = 0.095;
  // block_red.pose.orientation.w = 1.0;

  // moveit_msgs::msg::CollisionObject block_blue;
  // block_blue.id              = "shelf_block_blue";
  // block_blue.header.frame_id = "base_link";
  // block_blue.primitives.resize(1);
  // block_blue.primitives[0].type       = shape_msgs::msg::SolidPrimitive::BOX;
  // block_blue.primitives[0].dimensions = { 0.03, 0.03, 0.1 };
  // block_blue.pose.position.x    = 0.6;   // 1.1 - 0.5
  // block_blue.pose.position.y   = -0.35;  // 0.0 - 0.35
  // block_blue.pose.position.z    = 0.095;
  // block_blue.pose.orientation.w = 1.0;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
  psi.applyCollisionObject(shelf_low);
  psi.applyCollisionObject(shelf_up);
  // psi.applyCollisionObject(block_red);
  // psi.applyCollisionObject(block_blue);

  // ── Four corner posts, local offsets taken directly from empty.world ────
  std::vector<std::pair<std::string, std::pair<double, double>>> posts = {
    { "post_front_right", { 0.43, -0.18 } },
    { "post_front_left",  { 0.43,  0.18 } },
    { "post_back_right",  { -0.43, -0.18 } },
    { "post_back_left",   { -0.43,  0.18 } },
  };

  for (auto& [name, offset] : posts)
  {
    moveit_msgs::msg::CollisionObject post;
    post.id              = name;
    post.header.frame_id = "base_link";
    post.primitives.resize(1);
    post.primitives[0].type       = shape_msgs::msg::SolidPrimitive::CYLINDER;
    post.primitives[0].dimensions = { 0.9, 0.02 };  // height=0.9, radius=0.02
    post.pose.position.x    = shelf_origin_x + offset.first;
    post.pose.position.y    = shelf_origin_y + offset.second;
    post.pose.position.z    = shelf_origin_z + 0.45;  // 0.9 / 2
    post.pose.orientation.w = 1.0;
    psi.applyCollisionObject(post);
  }
}

void MTCTaskNode::doTask()
{
  task_ = createTask();

  // Initialise the task (validates stage connectivity and properties)
  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  // Run the planner – allow up to 8 planning attempts
  if (!task_.plan(8))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }

  // Publish the best solution so it can be visualised in RViz
  task_.introspection().publishSolution(*task_.solutions().front());

  // Execute the best solution on the real (or simulated) robot
  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }
}

mtc::Task MTCTaskNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("demo task");
  task.loadRobotModel(node_);

  // ── Planning group / frame names ─────────────────────────────────────────
  const auto& arm_group_name  = "arm";
  const auto& hand_group_name = "gripper";
  const auto& hand_frame      = "tcp";

  // Task-level properties – inherited by most stages
  task.setProperty("group",    arm_group_name);
  task.setProperty("eef",      "end_effector");
  task.setProperty("ik_frame", hand_frame);  // Frame used for IK calculations

  // Forward pointers used by later stages via setMonitoredStage()
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage* current_state_ptr   = nullptr;
  mtc::Stage* attach_object_stage = nullptr;
#pragma GCC diagnostic pop

  // ── Stage 0: Capture current robot state ─────────────────────────────────
  {
    auto stage = std::make_unique<mtc::stages::CurrentState>("current");
    current_state_ptr = stage.get();
    task.add(std::move(stage));
  }

  // ── Planner objects ───────────────────────────────────────────────────────
  // sampling_planner     : OMPL / pipeline planner for joint-space moves
  // interpolation_planner: simple joint interpolation for gripper open/close
  // cartesian_planner    : straight-line Cartesian moves (approach / retreat)
  auto sampling_planner      = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.5);
  cartesian_planner->setMaxAccelerationScalingFactor(0.5);
  cartesian_planner->setStepSize(0.01);  // 1 cm resolution along the path

  // ── Stage 1: Open gripper before approaching the object ──────────────────
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
    stage->setGroup(hand_group_name);
    stage->setGoal("gripper_open");
    task.add(std::move(stage));
  }

  // ── Stage 2: Move arm to a pre-grasp configuration ───────────────────────
  // Connect bridges the open-gripper state to the start of the grasp container.
  {
    auto stage = std::make_unique<mtc::stages::Connect>(
        "move to grasp",
        mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
    stage->setTimeout(5.0);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    task.add(std::move(stage));
  }

  // ── Stage 3: Pick serial container ───────────────────────────────────────
  // A SerialContainer groups all pick sub-stages so they are planned together.
  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");

    // Expose task-level properties to the container and its children
    task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
    grasp->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

    // ── 3a: Generate candidate grasp poses around the object ─────────────
    // Samples poses at regular angular increments (AngleDelta) around the
    // object and finds configurations where the gripper is open.
    {
      auto gen = std::make_unique<mtc::stages::GenerateGraspPose>("generate move towards");
      gen->properties().configureInitFrom(mtc::Stage::PARENT);
      gen->setObject("object");
      gen->setPreGraspPose("gripper_open");
      gen->setAngleDelta(M_PI / 2);               // Sample every 90° around the object
      gen->setMonitoredStage(current_state_ptr);  // Start from the captured state

      // Offset from TCP to the true grasp frame:
      //   -0.25 m along X : stand-off from the object centre
      //    0.08 m along Z : height adjustment
      //   Rotated 90° about Y so the gripper faces the object correctly
      Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
      tf.translation() = Eigen::Vector3d(-0.25, 0.0, 0.08);
      tf.linear() = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()).toRotationMatrix();

      // Wrap the pose generator in an IK solver
      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("compute ik", std::move(gen));
      wrapper->setIKFrame(tf, "tcp");
      wrapper->setMaxIKSolutions(8);
      wrapper->setTimeout(2.0);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      grasp->insert(std::move(wrapper));
    }

    // ── 3a-pre: Allow collision between shelf and object ────────────────
    {
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
            "allow collision (shelf,object)");
        stage->allowCollisions("object", { "shelflow" }, true);
        grasp->insert(std::move(stage));
    }

    // ── 3b: Approach the object along -Z (downward in base_link frame) ───
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("approach", cartesian_planner);
      stage->setMinMaxDistance(0.10, 0.30);  // 20–30 cm approach stroke
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "base_link";
      vec.vector.z = -1.0;  // Move downward
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    // ── 3c: Disable collision between hand links and the object ──────────
    // Required so the gripper can close around the object without MoveIt
    // reporting a collision.
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
          "allow collision (hand,object)");
      stage->allowCollisions(
          "object",
          task.getRobotModel()
              ->getJointModelGroup(hand_group_name)
              ->getLinkModelNamesWithCollisionGeometry(),
          true);
      grasp->insert(std::move(stage));
    }

    // ── 3d: Close gripper ─────────────────────────────────────────────────
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("gripper_close");
      grasp->insert(std::move(stage));
    }

    // ── 3e: Attach object to the TCP in the planning scene ───────────────
    // MoveIt now treats the object as part of the robot for collision
    // checking and IK in all subsequent stages.
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject("object", "tcp");
      attach_object_stage = stage.get();  // Save pointer – needed by place container
      grasp->insert(std::move(stage));
    }

    // {
    //   auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
    //       "disable collision (shelf,object)");
    //   stage->allowCollisions("object", { "shelflow" }, false);
    //   grasp->insert(std::move(stage));
    // }

    // ── 3f: Lift object upward along +Z ──────────────────────────────────
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.10, 0.30);  // 20–30 cm lift stroke
      stage->setIKFrame(hand_frame);
      stage->properties().set("link", hand_frame);

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;  // Move upward
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    task.add(std::move(grasp));
  }  // end pick container

  // ── Stage 4: Move arm to the place location ───────────────────────────────
  {
    auto stage = std::make_unique<mtc::stages::Connect>(
        "move to place",
        mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
    stage->setTimeout(5.0);
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage));
  }

  // ── Stage 5: Place serial container ──────────────────────────────────────
  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");

    // Expose task-level properties to the container and its children
    task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
    place->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

    // ── 5a: Generate and solve IK for the placement pose ─────────────────
    {
      auto gen = std::make_unique<mtc::stages::GeneratePlacePose>("generate place position");
      gen->properties().configureInitFrom(mtc::Stage::PARENT);
      gen->setObject("object");
      gen->setMonitoredStage(attach_object_stage);  // Object must already be attached

      // ── PLACE POSE: absolute base_link location, on the same physical
      // shelf board (shelf_row_bottom in empty.world) as the pick, shifted
      // along Y to land in the empty space toward the shelf's near edge —
      // clear of shelf_block_red (x=0.4,y=-0.35) and shelf_block_blue
      // (x=0.6,y=-0.35). Board spans y=[-0.55,-0.15] in base_link, so
      // y=-0.20 stays well inside it. Z matches the block's resting height
      // on shelf_row_bottom (top surface 0.06; object half-height 0.05).
      geometry_msgs::msg::PoseStamped p;
      p.header.frame_id    = "base_link";
      p.pose.position.x    = 0.5;
      p.pose.position.y    = -0.35;
      p.pose.position.z    = 0.095;
      p.pose.orientation.w = 1.0;  // Identity orientation
      gen->setPose(p);

      // Wrap in an IK solver using the object frame as the IK reference
      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(gen));
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(0.5);
      wrapper->setTimeout(2.0);
      wrapper->setIKFrame("object");
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      place->insert(std::move(wrapper));
    }

    // ── 5b: Open gripper to release the object ───────────────────────────
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("gripper_open");
      place->insert(std::move(stage));
    }

    // ── 5c: Re-enable collisions between hand and object ─────────────────
    // After release, the hand should no longer ignore collisions with the
    // object so the planner keeps them separated going forward.
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("disable collisions");
      stage->allowCollisions(
          "object",
          task.getRobotModel()
              ->getJointModelGroup(hand_group_name)
              ->getLinkModelNamesWithCollisionGeometry(),
          false);
      place->insert(std::move(stage));
    }

    // ── 5d: Detach object from the TCP in the planning scene ─────────────
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject("object", "tcp");
      place->insert(std::move(stage));
    }

    // ── 5e: Retreat upward after releasing the object ────────────────────
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("move up", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.10, 0.30);  // 20–30 cm retreat stroke
      stage->setIKFrame(hand_frame);

      geometry_msgs::msg::Vector3Stamped v;
      v.header.frame_id = "base_link";
      v.vector.z = 1.0;  // Move upward
      stage->setDirection(v);
      place->insert(std::move(stage));
    }

    task.add(std::move(place));
  }  // end place container

  // ── Stage 6: Return to home configuration ────────────────────────────────
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", sampling_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setGoal("home");
    task.add(std::move(stage));
  }

  return task;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  mtc_task_node->setupPlanningScene();
  rclcpp::sleep_for(std::chrono::seconds(1));
  mtc_task_node->doTask();
  rclcpp::sleep_for(std::chrono::seconds(30));

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}