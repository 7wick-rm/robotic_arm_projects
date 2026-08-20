/**
 * mtc.cpp
 * MoveIt Task Constructor (MTC) pick-and-place node for the Piper robot.
 *
 * Pipeline:
 *   1. Open gripper
 *   2. Move to pre-grasp position
 *   3. Compute grasp IK → close gripper → attach object → lift
 *   4. Move to place position
 *   5. Compute place IK → open gripper → detach object → retreat
 *   6. Return home
 */

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

#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include <boost/variant.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
namespace mtc = moveit::task_constructor;

// ─────────────────────────────────────────────────────────────────────────────
// Robot / scene constants — edit these to match your setup
// ─────────────────────────────────────────────────────────────────────────────

// MoveIt planning group names (must match piper.srdf)
static constexpr char ARM_GROUP[]  = "arm";
static constexpr char HAND_GROUP[] = "gripper";

// The tool-centre-point frame (tip of the arm, used for IK)
static constexpr char HAND_FRAME[] = "link6";

// End-effector name as declared in the SRDF <end_effector> tag
static constexpr char EEF_NAME[]   = "eef";

// Named gripper states defined in the SRDF
static constexpr char GRIPPER_OPEN[]   = "open";
static constexpr char GRIPPER_CLOSED[] = "close";

// Named arm pose to return to after placing (must exist in SRDF)
static constexpr char HOME_POSE[] = "home";

// Object to pick (must match the id set in setupPlanningScene)
static constexpr char OBJECT_ID[] = "object";

// ─────────────────────────────────────────────────────────────────────────────
// MTCTaskNode class
// ─────────────────────────────────────────────────────────────────────────────
class MTCTaskNode
{
public:
  explicit MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  /// Spawn a collision box in the planning scene to represent the object
  void setupPlanningScene();

  /// Build, plan, and execute the pick-and-place task
  void doTask();

private:
  mtc::Task createTask();

  mtc::Task             task_;
  rclcpp::Node::SharedPtr node_;
};

// ─────────────────────────────────────────────────────────────────────────────
// Implementation
// ─────────────────────────────────────────────────────────────────────────────

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_(std::make_shared<rclcpp::Node>("mtc_node", options))
{}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

void MTCTaskNode::setupPlanningScene()
{
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "world";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  object.primitives[0].dimensions = { 0.0159,0.0159,0.06 };

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.25;
  pose.position.y = 0.0;
  pose.position.z=0.06;
  pose.orientation.w = 1.0;
  object.pose = pose;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
}

void MTCTaskNode::doTask()
{
  task_ = createTask();

  // Initialise — validates stage connections and the robot model
  try {
    task_.init();
  } catch (mtc::InitStageException& e) {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  // Plan up to 5 solutions; use the first (lowest cost) one
  if (!task_.plan(5)) {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }

  // Publish the solution for visualisation in RViz MTC panel
  task_.introspection().publishSolution(*task_.solutions().front());

  // Execute
  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }

  RCLCPP_INFO(LOGGER, "Pick-and-place completed successfully!");
}

mtc::Task MTCTaskNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("piper_pick_place");
  task.loadRobotModel(node_);

  // Expose top-level properties so child stages can inherit them
  task.setProperty("group",    ARM_GROUP);
  task.setProperty("eef",      EEF_NAME);
  task.setProperty("ik_frame", HAND_FRAME);

  // ── Planners ─────────────────────────────────────────────────────────────
  // sampling_planner  : OMPL pipeline planner, used for free-space motion
  // interpolation_planner : simple joint interpolation, used for gripper open/close
  // cartesian_planner : straight-line Cartesian paths, used for lift / retreat
  auto sampling_planner     = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner= std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  auto interpolation_slow=std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  interpolation_slow->setMaxAccelerationScalingFactor(0.05);
  interpolation_slow->setMaxVelocityScalingFactor(0.05);

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.1);
  cartesian_planner->setMaxAccelerationScalingFactor(0.1);
  cartesian_planner->setStepSize(0.01);  // 1 cm resolution

  // ── Stage 1: capture current robot state ─────────────────────────────────
  // This stage is referenced later by GenerateGraspPose so it knows where
  // the robot currently is.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage* current_state_ptr = nullptr;
#pragma GCC diagnostic pop

  {
    auto stage = std::make_unique<mtc::stages::CurrentState>("current state");
    current_state_ptr = stage.get();
    task.add(std::move(stage));
  }

  // ── Stage 2: open gripper ─────────────────────────────────────────────────
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper", interpolation_planner);
    stage->setGroup(HAND_GROUP);
    stage->setGoal(GRIPPER_OPEN);
    task.add(std::move(stage));
  }

  // ── Stage 3: move arm to pre-grasp position ───────────────────────────────
  {
    auto stage = std::make_unique<mtc::stages::Connect>(
        "move to pick",
        mtc::stages::Connect::GroupPlannerVector{{ ARM_GROUP, sampling_planner }});
    stage->setTimeout(10.0);
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage));
  }

  // ── Stages 4–8: Pick serial container ────────────────────────────────────
  // All pick sub-stages are grouped so MTC can reason about them together.
  mtc::Stage* attach_object_stage = nullptr;  // referenced later by place

  {
    auto pick = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(pick->properties(), { "eef", "group", "ik_frame" });
    pick->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

    // Stage 4a: generate candidate grasp poses around the object
    {
      auto grasp_generator = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      grasp_generator->properties().configureInitFrom(mtc::Stage::PARENT);
      grasp_generator->properties().set("marker_ns", "grasp_pose");
      grasp_generator->setObject(OBJECT_ID);
      grasp_generator->setPreGraspPose(GRIPPER_OPEN);
      // Sample grasp angles every 45° around the object's vertical axis
      grasp_generator->setAngleDelta(M_PI / 2);
      grasp_generator->setMonitoredStage(current_state_ptr);

      // Transform from tool_0 to the actual grasp contact point.
      // 0.13 m along Z = gripper finger length offset.
      // 180° around X = flip so fingers point downward toward the object.
      // Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
      // grasp_frame_transform.translation().z() = 0.11;
      // grasp_frame_transform.linear() =
      //     Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();

      Eigen::Isometry3d grasp_frame_transform;
      Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
                          Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
      grasp_frame_transform.linear() = q.matrix();
      grasp_frame_transform.translation().x()=-0.20;
      grasp_frame_transform.translation().z()=0.09;

      // Stage 4b: solve IK for each candidate grasp pose
      auto ik_wrapper = std::make_unique<mtc::stages::ComputeIK>(
          "grasp pose IK", std::move(grasp_generator));
      ik_wrapper->setMaxIKSolutions(32);
      // Lower threshold = accept more diverse solutions (1.0 was too strict)
      ik_wrapper->setMinSolutionDistance(0.1);
      ik_wrapper->setIKFrame(grasp_frame_transform, HAND_FRAME);
      ik_wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      ik_wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      pick->insert(std::move(ik_wrapper));
    }

    // Stage 5: allow collisions between hand links and the object
    // (needed so the gripper can close around it without triggering collisions)
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow hand-object collisions");
      stage->allowCollisions(
          OBJECT_ID,
          task.getRobotModel()
              ->getJointModelGroup(HAND_GROUP)
              ->getLinkModelNamesWithCollisionGeometry(),
          true);
      pick->insert(std::move(stage));
    }

    // Stage 6: close the gripper
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close gripper", interpolation_slow);
      stage->setGroup(HAND_GROUP);
      stage->setGoal(GRIPPER_CLOSED);
      pick->insert(std::move(stage));
    }

    // Stage 7: attach the object to the hand frame so it moves with the robot
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject(OBJECT_ID, HAND_FRAME);
      attach_object_stage = stage.get();  // save pointer for place stage
      pick->insert(std::move(stage));
    }

    // Stage 8: lift the object straight up (Cartesian motion)
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.05, 0.15);
      stage->setIKFrame(HAND_FRAME);
      stage->properties().set("marker_ns", "lift_object");

      geometry_msgs::msg::Vector3Stamped dir;
      dir.header.frame_id = "base_link";
      dir.vector.z = 1.0;  // upward
      stage->setDirection(dir);
      pick->insert(std::move(stage));
    }

    task.add(std::move(pick));
  }

  // ── Stage 9: move arm to place position ──────────────────────────────────
  {
    auto stage = std::make_unique<mtc::stages::Connect>(
        "move to place",
        mtc::stages::Connect::GroupPlannerVector{{ ARM_GROUP, sampling_planner }});
    stage->setTimeout(10.0);
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage));
  }

  // ── Stages 10–14: Place serial container ─────────────────────────────────
  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
    place->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

    // Stage 10: generate place pose and solve IK
    {
      auto place_generator = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      place_generator->properties().configureInitFrom(mtc::Stage::PARENT);
      place_generator->properties().set("marker_ns", "place_pose");
      place_generator->setObject(OBJECT_ID);

      // Target place location — directly in front of the robot at table height
      geometry_msgs::msg::PoseStamped place_pose;
      place_pose.header.frame_id    = "base_link";
      place_pose.pose.position.x    =  0.25;
      place_pose.pose.position.y    =  0.25;
      // z = half box height (0.025) so object rests on table
      place_pose.pose.position.z    =  0.012;
      place_pose.pose.orientation.w =  1.0;
      place_generator->setPose(place_pose);
      // Monitor the attach stage so MTC knows what is being placed
      place_generator->setMonitoredStage(attach_object_stage);

      auto ik_wrapper = std::make_unique<mtc::stages::ComputeIK>(
          "place pose IK", std::move(place_generator));
      ik_wrapper->setMaxIKSolutions(8);
      ik_wrapper->setMinSolutionDistance(0.1);
      // IK solved for the object frame
      ik_wrapper->setIKFrame("object");
      ik_wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      ik_wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      place->insert(std::move(ik_wrapper));
    }

    // Stage 11: open gripper to release the object
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper", interpolation_planner);
      stage->setGroup(HAND_GROUP);
      stage->setGoal(GRIPPER_OPEN);
      place->insert(std::move(stage));
    }

    // Stage 12: forbid hand-object collisions again (object is now placed)
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid hand-object collisions");
      stage->allowCollisions(
          OBJECT_ID,
          task.getRobotModel()
              ->getJointModelGroup(HAND_GROUP)
              ->getLinkModelNamesWithCollisionGeometry(),
          false);
      place->insert(std::move(stage));
    }

    // Stage 13: detach the object from the hand
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject(OBJECT_ID, HAND_FRAME);
      place->insert(std::move(stage));
    }

    // Stage 14: retreat upward after placing (Cartesian motion)
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.05, 0.15);
      stage->setIKFrame(HAND_FRAME);
      stage->properties().set("marker_ns", "retreat");

      geometry_msgs::msg::Vector3Stamped dir;
      dir.header.frame_id = "base_link";
      dir.vector.z = 1.0;  // upward
      stage->setDirection(dir);
      place->insert(std::move(stage));
    }

    task.add(std::move(place));
  }

  // Stage 15: Close gripper completely    
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("close completely", interpolation_planner);
    stage->setGroup(HAND_GROUP);
    stage->setGoal(GRIPPER_CLOSED);
    task.add(std::move(stage));
  }
  
  // ── Stage 16: return to home pose ────────────────────────────────────────
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", sampling_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setGoal(HOME_POSE);
    task.add(std::move(stage));
  }

  return task;
}

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);

  // Spin the node in a background thread so MTC callbacks are processed
  // while doTask() blocks in the main thread waiting for results.
  rclcpp::executors::MultiThreadedExecutor executor;
  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  mtc_task_node->setupPlanningScene();
  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}









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

// static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
// namespace mtc = moveit::task_constructor;

// class MTCTaskNode
// {
// public:
//   MTCTaskNode(const rclcpp::NodeOptions& options);

//   rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

//   void doTask(    const std::string& object_id,
//     double place_z);

//   void setupPlanningScene();

// private:
//   mtc::Task createTask(const std::string& object_id,double place_z);
//   mtc::Task task_;
//   rclcpp::Node::SharedPtr node_;
// };
// rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
// {
//   return node_->get_node_base_interface();
// }

// MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
//   : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
// {
// }

// void MTCTaskNode::setupPlanningScene()
// {

//   std::vector<moveit_msgs::msg::CollisionObject> objects; 

//   moveit_msgs::msg::CollisionObject object1;
//   object1.id = "block1";
//   object1.header.frame_id = "base_link";
//   object1.primitives.resize(1);
//   object1.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
//   object1.primitives[0].dimensions = { 0.05, 0.05,0.024};

//   geometry_msgs::msg::Pose pose;
//   pose.position.x = 0.25;
//   pose.position.y = 0.0;
//   pose.position.z = 0.012;   
//   pose.orientation.w = 1.0;
//   object1.pose = pose;

//   objects.push_back(object1);

//   moveit_msgs::msg::CollisionObject object2=object1;
//   object2.id="block2";
//   geometry_msgs::msg::Pose pose2;
//   pose2.position.x=0.25;
//   pose2.position.y=0.10;
//   pose2.position.z=0.012;
//   pose2.orientation.w=1.0;
//   object2.pose=pose2;

//   objects.push_back(object2);

//   moveit_msgs::msg::CollisionObject object3=object1;
//   object3.id="block3";
//   geometry_msgs::msg::Pose pose3;
//   pose3.position.x=0.25;
//   pose3.position.y=-0.10;
//   pose3.position.z=0.012;
//   pose3.orientation.w=1.0;
//   object3.pose=pose3;

//   objects.push_back(object3);



//   moveit::planning_interface::PlanningSceneInterface psi;
//   psi.applyCollisionObjects(objects);
// }

// void MTCTaskNode::doTask(const std::string& object_id,
//     double place_z)
// {
//   task_ = createTask(object_id,place_z);


//   try
//   {
//     task_.init();
//   }
//   catch (mtc::InitStageException& e)
//   {
//     RCLCPP_ERROR_STREAM(LOGGER, e);
//     return;
//   }

//   if (!task_.plan(5))
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

// mtc::Task MTCTaskNode::createTask(const std::string& object_id,double place_z){
//   mtc::Task task;
//   task.stages()->setName("demo task");
//   task.loadRobotModel(node_);

//   const auto& arm_group_name = "manipulator";
//   const auto& hand_group_name = "gripper";
//   const auto& hand_frame = "tool_0";

//   // Set task properties
//   task.setProperty("group", arm_group_name);
//   task.setProperty("eef", "eff");
//   task.setProperty("ik_frame", hand_frame);

// // Disable warnings for this line, as it's a variable that's set but not used in this example
// #pragma GCC diagnostic push
// #pragma GCC diagnostic ignored "-Wunused-but-set-variable"
//   mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
// #pragma GCC diagnostic pop

//   auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
//   current_state_ptr = stage_state_current.get();
//   task.add(std::move(stage_state_current));

//   auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
//   auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

//   auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
//   cartesian_planner->setMaxVelocityScalingFactor(0.6);
//   cartesian_planner->setMaxAccelerationScalingFactor(0.6);
//   cartesian_planner->setStepSize(.01);

//   auto stage_open_hand =
//       std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
//   stage_open_hand->setGroup(hand_group_name);
//   stage_open_hand->setGoal("open");
//   task.add(std::move(stage_open_hand));


//   {
//     auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>("move to pick",mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
//     stage_move_to_pick->setTimeout(5.0);
//     stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
//     task.add(std::move(stage_move_to_pick));
//   }

//   mtc::Stage* attach_object_stage =
//     nullptr;  // Forward attach_object_stage to place pose generator


//   {

//   auto grasp = std::make_unique<mtc::SerialContainer>("Pick Object");
//   task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
//   grasp->properties().configureInitFrom(mtc::Stage::PARENT,{ "eef", "group", "ik_frame" });
//   {

//     auto stage=std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
//     stage->properties().configureInitFrom(mtc::Stage::PARENT);
//     stage->properties().set("marker_ns", "grasp_pose");
//     stage->setObject(object_id);
//     stage->setPreGraspPose("open");
//     stage->setAngleDelta(M_PI/2);
//     stage->setMonitoredStage(current_state_ptr);

//     Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
//     grasp_frame_transform.translation().z() = 0.15;
//     grasp_frame_transform.linear()=Eigen::AngleAxisd(M_PI,Eigen::Vector3d::UnitX()).toRotationMatrix();

//   auto wrapper =
//       std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
//   wrapper->setMaxIKSolutions(8);
//   wrapper->setMinSolutionDistance(1.0);
//   wrapper->setIKFrame(grasp_frame_transform, hand_frame);
//   wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
//   wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
//   grasp->insert(std::move(wrapper));



//   }

//   {
//     auto stage=std::make_unique<mtc::stages::ModifyPlanningScene>("allow collisions");
//     stage->allowCollisions(object_id,task.getRobotModel()->getJointModelGroup(hand_group_name)->getLinkModelNamesWithCollisionGeometry(),true);
//     grasp->insert(std::move(stage));

//   }

//   {
//   auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
//   stage->setGroup(hand_group_name);
//   stage->setGoal("closed");
//   grasp->insert(std::move(stage));
// }

//   {
//   auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
//   stage->attachObject(object_id, hand_frame);
//   attach_object_stage = stage.get();
//   grasp->insert(std::move(stage));
//   }
  

//   {
//   auto stage =
//       std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
//   stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
//   stage->setMinMaxDistance(0.05, 0.3);
//   stage->setIKFrame(hand_frame);
//   stage->properties().set("marker_ns", "lift_object");

//   geometry_msgs::msg::Vector3Stamped vec;
//   vec.header.frame_id = "base_link";
//   vec.vector.z = 1.0;
//   stage->setDirection(vec);
//   grasp->insert(std::move(stage));
// }
//   task.add(std::move(grasp));
//   }

//   {
//   auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
//       "move to place",
//       mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner }
//                                                  });
//   stage_move_to_place->setTimeout(5.0);
//   stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
//   task.add(std::move(stage_move_to_place));
// }
// {
//   auto place = std::make_unique<mtc::SerialContainer>("place object");
//   task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
//   place->properties().configureInitFrom(mtc::Stage::PARENT,
//                                         { "eef", "group", "ik_frame" });

//   {
//   auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
//   stage->properties().configureInitFrom(mtc::Stage::PARENT);
//   stage->properties().set("marker_ns", "place_pose");
//   stage->setObject(object_id);

//   geometry_msgs::msg::PoseStamped target_pose_msg;
//   target_pose_msg.header.frame_id = "base_link";
//   target_pose_msg.pose.position.x=0.25;
//   target_pose_msg.pose.position.y = 0.0;
//   target_pose_msg.pose.position.z = place_z; 
//   target_pose_msg.pose.orientation.w = 1.0;
//   stage->setPose(target_pose_msg);
//   stage->setMonitoredStage(attach_object_stage); 

//   auto wrapper =
//       std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
//   wrapper->setMaxIKSolutions(8);
//   wrapper->setMinSolutionDistance(1.0);
//   wrapper->setIKFrame(object_id);
//   wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
//   wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
//   place->insert(std::move(wrapper));
// }


// {
//   auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
//   stage->setGroup(hand_group_name);
//   stage->setGoal("open");
//   place->insert(std::move(stage));
// }

// {
//   auto stage =
//       std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
//   stage->allowCollisions(object_id,
//                         task.getRobotModel()
//                             ->getJointModelGroup(hand_group_name)
//                             ->getLinkModelNamesWithCollisionGeometry(),
//                         false);
//   place->insert(std::move(stage));
// }

// {
//   auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
//   stage->detachObject(object_id, hand_frame);
//   place->insert(std::move(stage));
// }

// {
//   auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
//   stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
//   stage->setMinMaxDistance(0.01, 0.3);
//   stage->setIKFrame(hand_frame);
//   stage->properties().set("marker_ns", "retreat");

//   // Set retreat direction
//   geometry_msgs::msg::Vector3Stamped vec;
//   vec.header.frame_id = "base_link";
//   vec.vector.z = 0.1;
//   stage->setDirection(vec);
//   place->insert(std::move(stage));
// }

//   task.add(std::move(place));
// }

// {
//   auto stage = std::make_unique<mtc::stages::MoveTo>("return home", sampling_planner);
//   stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
//   stage->setGoal("forward");
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

//   mtc_task_node->setupPlanningScene();
//   // mtc_task_node->doTask("block1",0.05);
//   mtc_task_node->doTask("block2",0.038);
//   mtc_task_node->doTask("block3",0.070);

//   spin_thread->join();
//   rclcpp::shutdown();
//   return 0;
// }
