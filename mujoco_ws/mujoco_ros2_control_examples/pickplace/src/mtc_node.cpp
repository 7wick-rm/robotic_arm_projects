#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>
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

static const rclcpp::Logger LOGGER = rclcpp::get_logger("pickplace");
namespace mtc = moveit::task_constructor;

static constexpr double kBlockSide = 0.05;
static constexpr double kBlockHeight = 0.024;

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  bool doTask(const std::string& object_id, double place_x, double place_y, double place_z,
              const std::vector<std::string>& support_objects);

  void setupPlanningScene();

  bool syncHandOpen();

private:
  mtc::Task createTask(const std::string& object_id, double place_x, double place_y, double place_z,
                        const std::vector<std::string>& support_objects);
  rclcpp::Node::SharedPtr node_;

  moveit::core::RobotModelPtr robot_model_;
};

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
  robot_model_loader::RobotModelLoader robot_model_loader(node_, "robot_description");
  robot_model_ = robot_model_loader.getModel();
  if (!robot_model_)
  {
    RCLCPP_FATAL(LOGGER, "Failed to load robot model from 'robot_description'");
  }
}

void MTCTaskNode::setupPlanningScene()
{
  std::vector<moveit_msgs::msg::CollisionObject> objects;

  moveit_msgs::msg::CollisionObject object1;
  object1.id = "block1";
  object1.header.frame_id = "world";
  object1.primitives.resize(1);
  object1.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  object1.primitives[0].dimensions = { kBlockSide, kBlockSide, kBlockHeight };

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.5;
  pose.position.y = 0.0;
  pose.position.z = -0.01;
  pose.orientation.w = 1.0;
  object1.pose = pose;

  objects.push_back(object1);

  moveit_msgs::msg::CollisionObject object2 = object1;
  object2.id = "block2";
  geometry_msgs::msg::Pose pose2;
  pose2.position.x = 0.5;
  pose2.position.y = 0.25;
  pose2.position.z = -0.01;
  pose2.orientation.w = 1.0;
  object2.pose = pose2;

  objects.push_back(object2);

  moveit_msgs::msg::CollisionObject object3 = object1;
  object3.id = "block3";
  geometry_msgs::msg::Pose pose3;
  pose3.position.x = 0.5;
  pose3.position.y = -0.25;
  pose3.position.z = -0.01;
  pose3.orientation.w = 1.0;
  object3.pose = pose3;

  objects.push_back(object3);

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObjects(objects);
}

bool MTCTaskNode::syncHandOpen()
{
  static const std::string hand_group_name = "hand";

  moveit::planning_interface::MoveGroupInterface hand_group(node_, hand_group_name);
  // Tight tolerance so the controller is forced to actually settle close to
  // the named target rather than reporting "reached" while still off by a
  // gap large enough to trip MTC's merge check.
  hand_group.setGoalJointTolerance(0.0002);
  hand_group.setNamedTarget("open");

  const auto result = hand_group.move();  // blocks until plan+execute finishes
  const bool ok = (result == moveit::core::MoveItErrorCode::SUCCESS);
  if (!ok)
  {
    RCLCPP_WARN_STREAM(LOGGER, "syncHandOpen: gripper did not reach 'open' within tolerance (error code "
                                    << result.val << ")");
  }

  const auto joint_names = hand_group.getActiveJoints();
  const auto joint_values = hand_group.getCurrentJointValues();
  for (size_t i = 0; i < joint_names.size() && i < joint_values.size(); ++i)
  {
    RCLCPP_INFO_STREAM(LOGGER, "  " << joint_names[i] << " = " << joint_values[i]);
  }

  return ok;
}

bool MTCTaskNode::doTask(const std::string& object_id, double place_x, double place_y, double place_z,
                          const std::vector<std::string>& support_objects)
{
  mtc::Task task = createTask(object_id, place_x, place_y, place_z, support_objects);

  try
  {
    task.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return false;
  }

  if (!task.plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed for object '" << object_id << "'");
    return false;
  }

  task.introspection().publishSolution(*task.solutions().front());
  auto result = task.execute(*task.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed for object '" << object_id << "'");
    return false;
  }

  return true;
}

mtc::Task MTCTaskNode::createTask(const std::string& object_id, double place_x, double place_y, double place_z,
                                   const std::vector<std::string>& support_objects)
{
  mtc::Task task;
  task.stages()->setName("pick_place_" + object_id);
  // Reuse the robot model loaded once in the constructor instead of calling
  // task.loadRobotModel(node_), which re-loads/re-parses the model from
  // scratch on every single call to createTask().
  task.setRobotModel(robot_model_);

  const auto& arm_group_name = "panda_arm";
  const auto& hand_group_name = "hand";
  const auto& hand_frame = "panda_hand";

  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage* current_state_ptr = nullptr;
#pragma GCC diagnostic pop

  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.05);
  cartesian_planner->setMaxAccelerationScalingFactor(0.05);
  cartesian_planner->setStepSize(.01);

  auto stage_open_hand = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("open");
  task.add(std::move(stage_open_hand));

  {
    auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
        "move to pick", mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
    stage_move_to_pick->setTimeout(5.0);
    stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage_move_to_pick));
  }

  mtc::Stage* attach_object_stage = nullptr;

  {
    auto grasp = std::make_unique<mtc::SerialContainer>("Pick Object");
    task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
    grasp->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

    {
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setObject(object_id);
      stage->setPreGraspPose("open");
      stage->setAngleDelta(M_PI / 2);
      stage->setMonitoredStage(current_state_ptr);

      Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
      grasp_frame_transform.translation().z() = 0.13;
      grasp_frame_transform.linear() = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();

      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(grasp_frame_transform, hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      grasp->insert(std::move(wrapper));
    }

    {
      // Allow the hand to touch the object it's about to grasp.
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
      stage->allowCollisions(
          object_id, task.getRobotModel()->getJointModelGroup(hand_group_name)->getLinkModelNamesWithCollisionGeometry(),
          true);
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("close");
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject(object_id, hand_frame);
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }

    {

      if (!support_objects.empty())
      {
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (object,supports) - pick");
        for (const auto& support : support_objects)
          stage->allowCollisions(object_id, support, true);
        grasp->insert(std::move(stage));
      }
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.0, 0.2);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lift_object");

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    task.add(std::move(grasp));
  }

  {
    auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
        "move to place", mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
    stage_move_to_place->setTimeout(5.0);
    stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage_move_to_place));
  }

  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
    place->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

    {
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject(object_id);

      geometry_msgs::msg::PoseStamped target_pose_msg;
      target_pose_msg.header.frame_id = "world";
      target_pose_msg.pose.position.x = place_x;
      target_pose_msg.pose.position.y = place_y;
      target_pose_msg.pose.position.z = place_z;
      target_pose_msg.pose.orientation.w = 1.0;
      stage->setPose(target_pose_msg);
      stage->setMonitoredStage(attach_object_stage);

      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(object_id);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      place->insert(std::move(wrapper));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("Approach down", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.00, 0.2);

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = -1.0;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("open");
      place->insert(std::move(stage));
    }

    {

      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
      stage->allowCollisions(
          object_id, task.getRobotModel()->getJointModelGroup(hand_group_name)->getLinkModelNamesWithCollisionGeometry(),
          false);
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject(object_id, hand_frame);
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.0, 0.1);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "retreat");

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 0.1;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

    task.add(std::move(place));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", sampling_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setGoal("ready");
    task.add(std::move(stage));
  }

  {
    auto stage_open_hand = std::make_unique<mtc::stages::MoveTo>("open hand (home)", interpolation_planner);
    stage_open_hand->setGroup(hand_group_name);
    stage_open_hand->setGoal("open");
    task.add(std::move(stage_open_hand));
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

  const std::vector<std::string> stack_order = { "block1", "block2" , "block3" };
  const double stack_x = 0.5;
  const double stack_y = 0.0;   // block3's original (x, y) position
  const double base_z = -0.01;    // block3's original z position

  double current_top_z = base_z + kBlockHeight / 2.0 + 0.05;  // top surface of the base block
  std::vector<std::string> placed_supports = { stack_order.front() };

  for (size_t i = 1; i < stack_order.size(); ++i)
  {
    const std::string& object_id = stack_order[i];
    const double place_z = current_top_z + kBlockHeight / 2.0;

    RCLCPP_INFO_STREAM(LOGGER, "Stacking '" << object_id << "' at z=" << place_z);

    if (!mtc_task_node->doTask(object_id, stack_x, stack_y, place_z, placed_supports))
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Stopping stack: failed to place '" << object_id << "'");
      break;
    }

    current_top_z = place_z + kBlockHeight / 2.0;
    placed_supports.push_back(object_id);

    RCLCPP_INFO(LOGGER, "Re-syncing gripper to 'open' before next pick...");
    mtc_task_node->syncHandOpen();
  }

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}