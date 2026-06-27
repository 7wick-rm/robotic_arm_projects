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

// ── Per-block configuration ───────────────────────────────────────────────────
// Each entry fully describes one pick-and-place cycle.
struct BlockConfig
{
  std::string id;          // collision object id  (must match the Gazebo model name)
  double pick_x;           // pick pose in base_link frame
  double pick_y;
  double pick_z;
  double place_x;          // place pose in base_link frame  ← YOU FILL THESE IN
  double place_y;
  double place_z;
};

// ─────────────────────────────────────────────────────────────────────────────
//  ↓↓↓  EDIT THE place_x / place_y / place_z VALUES BELOW  ↓↓↓
// ─────────────────────────────────────────────────────────────────────────────
static const std::vector<BlockConfig> BLOCKS = {
  // RED block  — 0.2 m offset in X from green
  { "object_red",    0.40, -0.44, 0.09,    0.5,  -0.86,   0.2   },
  
  // GREEN block  (original, pick position unchanged)
  { "object_green",  0.50, -0.40, 0.09,    0.50, -0.76, 0.20  },

  // BLUE block — 0.4 m offset in X from green (another 0.2 m from red)
  { "object_blue",   0.60, -0.48, 0.09,    0.5,  -0.96,   0.2   },
};
// ─────────────────────────────────────────────────────────────────────────────

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();

  void setupPlanningScene();

private:
  // Creates one complete pick-and-place task for a single block.
  // remaining_objects: ids of blocks not yet picked – registered as obstacles.
  mtc::Task createTask(const BlockConfig& block,
                       const std::vector<std::string>& remaining_objects);

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
  moveit::planning_interface::PlanningSceneInterface psi;

  // ── Shelf geometry (unchanged) ────────────────────────────────────────────
  const double shelf_origin_x = 0.5;
  const double shelf_origin_y = -0.35;
  const double shelf_origin_z = 0.0;

  auto make_shelf_panel = [&](const std::string& id, double z) {
    moveit_msgs::msg::CollisionObject obj;
    obj.id              = id;
    obj.header.frame_id = "base_link";
    obj.primitives.resize(1);
    obj.primitives[0].type       = shape_msgs::msg::SolidPrimitive::BOX;
    obj.primitives[0].dimensions = { 0.9, 0.4, 0.02 };
    obj.pose.position.x    = shelf_origin_x;
    obj.pose.position.y    = shelf_origin_y;
    obj.pose.position.z    = z;
    obj.pose.orientation.w = 1.0;
    psi.applyCollisionObject(obj);
  };

  make_shelf_panel("shelflow", shelf_origin_z + 0.05);
  make_shelf_panel("shelfup",  shelf_origin_z + 0.85);

  // ── Four corner posts ─────────────────────────────────────────────────────
  std::vector<std::pair<std::string, std::pair<double, double>>> posts = {
    { "post_front_right", { 0.43, -0.18 } },
    { "post_front_left",  { 0.43,  0.18 } },
    { "post_back_right",  { -0.43, -0.18 } },
    { "post_back_left",   { -0.43,  0.18 } },
  };
  for (auto& [name, off] : posts)
  {
    moveit_msgs::msg::CollisionObject post;
    post.id              = name;
    post.header.frame_id = "base_link";
    post.primitives.resize(1);
    post.primitives[0].type       = shape_msgs::msg::SolidPrimitive::CYLINDER;
    post.primitives[0].dimensions = { 0.9, 0.02 };
    post.pose.position.x    = shelf_origin_x + off.first;
    post.pose.position.y    = shelf_origin_y + off.second;
    post.pose.position.z    = shelf_origin_z + 0.45;
    post.pose.orientation.w = 1.0;
    psi.applyCollisionObject(post);
  }

  // ── Register ALL three blocks in the planning scene ───────────────────────
  for (const auto& blk : BLOCKS)
  {
    moveit_msgs::msg::CollisionObject obj;
    obj.id              = blk.id;
    obj.header.frame_id = "base_link";
    obj.primitives.resize(1);
    obj.primitives[0].type       = shape_msgs::msg::SolidPrimitive::BOX;
    obj.primitives[0].dimensions = { 0.03, 0.03, 0.1 };
    obj.pose.position.x    = blk.pick_x;
    obj.pose.position.y    = blk.pick_y;
    obj.pose.position.z    = blk.pick_z;
    obj.pose.orientation.w = 1.0;
    psi.applyCollisionObject(obj);
    RCLCPP_INFO(LOGGER, "Registered block '%s' at (%.2f, %.2f, %.2f)",
                blk.id.c_str(), blk.pick_x, blk.pick_y, blk.pick_z);
  }
}

void MTCTaskNode::doTask()
{
  // Execute one pick-and-place cycle per block, in order.
  // After each successful execution the block is no longer in its pick
  // position, so it is removed from the "remaining obstacles" list for the
  // next cycle.
  std::vector<std::string> remaining;
  for (const auto& blk : BLOCKS)
    remaining.push_back(blk.id);

  for (std::size_t i = 0; i < BLOCKS.size(); ++i)
  {
    const auto& blk = BLOCKS[i];

    // Build remaining-obstacles list: every block that has NOT been picked yet
    // (excluding the one we are about to pick).
    std::vector<std::string> obstacles;
    for (std::size_t j = i + 1; j < BLOCKS.size(); ++j)
      obstacles.push_back(BLOCKS[j].id);

    RCLCPP_INFO(LOGGER, "─── Planning pick-and-place for block '%s' ───", blk.id.c_str());

    task_ = createTask(blk, obstacles);

    try
    {
      task_.init();
    }
    catch (mtc::InitStageException& e)
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Init failed for block '" << blk.id << "': " << e);
      return;
    }

    if (!task_.plan(8))
    {
      RCLCPP_ERROR(LOGGER, "Planning failed for block '%s'", blk.id.c_str());
      return;
    }

    task_.introspection().publishSolution(*task_.solutions().front());

    auto result = task_.execute(*task_.solutions().front());
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      RCLCPP_ERROR(LOGGER, "Execution failed for block '%s'", blk.id.c_str());
      return;
    }

    RCLCPP_INFO(LOGGER, "✓ Block '%s' placed successfully.", blk.id.c_str());

    // Brief pause between cycles so the planning scene settles
    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }

  RCLCPP_INFO(LOGGER, "✓✓✓  All blocks picked and placed.");
}

// ─────────────────────────────────────────────────────────────────────────────
// createTask  –  builds a full pick-and-place MTC task for ONE block.
//
// remaining_objects: ids of blocks not yet picked; they are treated as static
//                    obstacles that the arm must not collide with.
// ─────────────────────────────────────────────────────────────────────────────
mtc::Task MTCTaskNode::createTask(const BlockConfig& block,
                                  const std::vector<std::string>& remaining_objects)
{
  mtc::Task task;
  task.stages()->setName("pick_place_" + block.id);
  task.loadRobotModel(node_);

  const auto& arm_group_name  = "arm";
  const auto& hand_group_name = "gripper";
  const auto& hand_frame      = "tcp";

  task.setProperty("group",    arm_group_name);
  task.setProperty("eef",      "end_effector");
  task.setProperty("ik_frame", hand_frame);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage* current_state_ptr   = nullptr;
  mtc::Stage* attach_object_stage = nullptr;
#pragma GCC diagnostic pop

  // ── Planners ──────────────────────────────────────────────────────────────
  auto sampling_planner      = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.5);
  cartesian_planner->setMaxAccelerationScalingFactor(0.5);
  cartesian_planner->setStepSize(0.01);

  // ── Stage 0: Current state ────────────────────────────────────────────────
  {
    auto stage = std::make_unique<mtc::stages::CurrentState>("current");
    current_state_ptr = stage.get();
    task.add(std::move(stage));
  }

  // ── Stage 1: Open gripper ─────────────────────────────────────────────────
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
    stage->setGroup(hand_group_name);
    stage->setGoal("gripper_open");
    task.add(std::move(stage));
  }

  // ── Stage 2: Move to pre-grasp ────────────────────────────────────────────
  {
    auto stage = std::make_unique<mtc::stages::Connect>(
        "move to grasp",
        mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
    stage->setTimeout(5.0);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    task.add(std::move(stage));
  }

  // ── Stage 3: Pick serial container ───────────────────────────────────────
  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick " + block.id);

    task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
    grasp->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

    // ── 3a: Generate grasp poses ──────────────────────────────────────────
    {
      auto gen = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp");
      gen->properties().configureInitFrom(mtc::Stage::PARENT);
      gen->setObject(block.id);
      gen->setPreGraspPose("gripper_open");
      gen->setAngleDelta(M_PI / 2);
      gen->setMonitoredStage(current_state_ptr);

      Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
      tf.translation() = Eigen::Vector3d(0.0, 0.0, 0.1);
      tf.linear() = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()).toRotationMatrix();

      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("compute ik", std::move(gen));
      wrapper->setIKFrame(tf, "tcp");
      wrapper->setMaxIKSolutions(8);
      wrapper->setTimeout(2.0);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      grasp->insert(std::move(wrapper));
    }

    // ── 3a-pre: Allow collision between shelf and THIS block ──────────────
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
          "allow collision (shelf," + block.id + ")");
      stage->allowCollisions(block.id, { "shelflow" }, true);
      grasp->insert(std::move(stage));
    }

    // ── 3a-obs: Explicitly allow gripper to coexist near remaining blocks ─
    // The planner must not abort because of predicted proximity with blocks
    // that are still sitting on the shelf — they will be picked later.
    if (!remaining_objects.empty())
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
          "allow collision (hand, remaining blocks)");
      for (const auto& obs_id : remaining_objects)
        stage->allowCollisions(
            obs_id,
            task.getRobotModel()
                ->getJointModelGroup(hand_group_name)
                ->getLinkModelNamesWithCollisionGeometry(),
            true);
      grasp->insert(std::move(stage));
    }

    // ── 3b: Approach along -Z ─────────────────────────────────────────────
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("approach", cartesian_planner);
      stage->setMinMaxDistance(0.01, 0.30);
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "base_link";
      vec.vector.z = -1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    // ── 3c: Allow collision hand ↔ target block ───────────────────────────
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
          "allow collision (hand," + block.id + ")");
      stage->allowCollisions(
          block.id,
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

    // ── 3e: Attach object ────────────────────────────────────────────────
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach " + block.id);
      stage->attachObject(block.id, "tcp");
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }

    // ── 3e-hold: Re-issue close to make controller hold the grip ─────────
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("hold grip", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("gripper_close");
      grasp->insert(std::move(stage));
    }

    // ── 3f: Lift object ───────────────────────────────────────────────────
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lift " + block.id, cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.01, 0.30);
      stage->setIKFrame(hand_frame);
      stage->properties().set("link", hand_frame);

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    task.add(std::move(grasp));
  }  // end pick container

  // ── Stage 4: Move to place ────────────────────────────────────────────────
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
    auto place = std::make_unique<mtc::SerialContainer>("place " + block.id);

    task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
    place->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

    // ── 5a: Generate and solve IK for placement pose ──────────────────────
    {
      auto gen = std::make_unique<mtc::stages::GeneratePlacePose>("generate place position");
      gen->properties().configureInitFrom(mtc::Stage::PARENT);
      gen->setObject(block.id);
      gen->setMonitoredStage(attach_object_stage);

      geometry_msgs::msg::PoseStamped p;
      p.header.frame_id    = "base_link";
      p.pose.position.x    = block.place_x;
      p.pose.position.y    = block.place_y;
      p.pose.position.z    = block.place_z;
      p.pose.orientation.w = 1.0;
      gen->setPose(p);

      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(gen));
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(0.5);
      wrapper->setTimeout(2.0);
      wrapper->setIKFrame(block.id);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      place->insert(std::move(wrapper));
    }

    // ── 5b: Lower to surface before releasing ─────────────────────────────
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lower to place", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.0, 0.06);
      stage->setIKFrame(hand_frame);
      stage->properties().set("link", hand_frame);

      geometry_msgs::msg::Vector3Stamped lower_vec;
      lower_vec.header.frame_id = "world";
      lower_vec.vector.z = -1.0;
      stage->setDirection(lower_vec);
      place->insert(std::move(stage));
    }

    // ── 5c: Open gripper ──────────────────────────────────────────────────
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("gripper_open");
      place->insert(std::move(stage));
    }

    // ── 5d: Re-enable collisions hand ↔ block ─────────────────────────────
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("disable collisions");
      stage->allowCollisions(
          block.id,
          task.getRobotModel()
              ->getJointModelGroup(hand_group_name)
              ->getLinkModelNamesWithCollisionGeometry(),
          false);
      place->insert(std::move(stage));
    }

    // ── 5e: Detach block from TCP ─────────────────────────────────────────
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach " + block.id);
      stage->detachObject(block.id, "tcp");
      place->insert(std::move(stage));
    }

    // ── 5f: Retreat upward ────────────────────────────────────────────────
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("move up", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.10, 0.30);
      stage->setIKFrame(hand_frame);

      geometry_msgs::msg::Vector3Stamped v;
      v.header.frame_id = "base_link";
      v.vector.z = 1.0;
      stage->setDirection(v);
      place->insert(std::move(stage));
    }

    task.add(std::move(place));
  }  // end place container

  // ── Stage 6: Return home ──────────────────────────────────────────────────
  // {
  //   auto stage = std::make_unique<mtc::stages::MoveTo>("return home", sampling_planner);
  //   stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  //   stage->setGoal("home");
  //   task.add(std::move(stage));
  // }

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