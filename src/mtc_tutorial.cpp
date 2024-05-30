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
  // Compose an MTC task from a series of stages.
  mtc::Task createTask();
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
};

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
}

void MTCTaskNode::setupPlanningScene()
{
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "world";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = { 0.1, 0.02 };

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.45;
  pose.position.y = -0.35;
  pose.position.z = 0.25;
  pose.orientation.w = 1.0;
  object.pose = pose;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
  moveit_msgs::msg::CollisionObject table;
  table.id = "table";
  table.header.frame_id = "world";

  // Define the table dimensions and pose
  shape_msgs::msg::SolidPrimitive table_shape;
  table_shape.type = shape_msgs::msg::SolidPrimitive::BOX;
  table_shape.dimensions.resize(3);
  table_shape.dimensions[0] = 0.9; // Table length
  table_shape.dimensions[1] = 0.6; // Table width
  table_shape.dimensions[2] = 0.2; // Table height

  geometry_msgs::msg::Pose table_pose;
  table_pose.position.x = 0.45;
  table_pose.position.y = -0.6;
  table_pose.position.z = 0.1; // Half the height of the table, so the top is at z=0.75
  table_pose.orientation.w = 1.0;

  table.primitives.push_back(table_shape);
  table.primitive_poses.push_back(table_pose);
  table.operation = table.ADD;

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObject(table);
}

void MTCTaskNode::doTask()
{
  task_ = createTask();

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(20))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }
  task_.introspection().publishSolution(*task_.solutions().front());

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }

  return;
}

/* mtc::Task MTCTaskNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("demo task");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "ur5e_arm";
  const auto& hand_group_name = "gripper";
  const auto& hand_frame = "flange";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.01);

  // Current state
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  task.add(std::move(stage_state_current));

  // Close hand
  auto stage_open_hand = std::make_unique<mtc::stages::MoveTo>("open hand", sampling_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("open");
  task.add(std::move(stage_open_hand));

  // Define the target pose
  auto const target_pose = []{
	  geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "world";
	  
    // Define roll, pitch, and yaw angles in radians
	  double roll = M_PI/2.0;  // Rotation around the x-axis
	  double pitch = 0.0;  // Rotation around the y-axis (90 degrees)
	  double yaw = 0.0;  // Rotation around the z-axis
	  // Convert roll, pitch, yaw to quaternion
	  tf2::Quaternion q;
	  q.setRPY(roll, pitch, yaw);

	  msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    msg.pose.orientation.w = q.w();
	  msg.pose.position.x = 0.45;
	  msg.pose.position.y = -0.10;  //to 10mm slide move to got to grasp the object
	  msg.pose.position.z = 0.25;
	  return msg;
	}();
  
  // Move to target pose
  auto move_to_target = std::make_unique<mtc::stages::MoveTo>("move to target", sampling_planner);
  move_to_target->setGroup(arm_group_name);
  move_to_target->setGoal(target_pose);
  task.add(std::move(move_to_target));


  // Define the slide pose
  auto const slide_pose = []{
	  geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "world";
	  
    // Define roll, pitch, and yaw angles in radians
	  double roll = M_PI/2.0;  // Rotation around the x-axis
	  double pitch = 0.0;  // Rotation around the y-axis (90 degrees)
	  double yaw = 0.0;  // Rotation around the z-axis
	  // Convert roll, pitch, yaw to quaternion
	  tf2::Quaternion q;
	  q.setRPY(roll, pitch, yaw);

	  msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    msg.pose.orientation.w = q.w();
	  msg.pose.position.x = 0.45;
	  msg.pose.position.y = -0.18;  //to 8cm slide move
	  msg.pose.position.z = 0.25;
	  return msg;
	}();
  
  // Move to slide pose
  auto move_to_slide = std::make_unique<mtc::stages::MoveTo>("move to slide", cartesian_planner);
  move_to_slide->setGroup(arm_group_name);
  move_to_slide->setGoal(slide_pose);
  task.add(std::move(move_to_slide));

  // Close hand
  auto stage_close_hand = std::make_unique<mtc::stages::MoveTo>("close hand", sampling_planner);
  stage_close_hand->setGroup(hand_group_name);
  stage_close_hand->setGoal("close");
  stage_close_hand->setProperty("allow_collision", true); // Disable collision checking
  task.add(std::move(stage_close_hand));
  
  return task;
} */
mtc::Task MTCTaskNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("demo task");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "ur5e_arm";
  const auto& hand_group_name = "gripper";
  const auto& hand_frame = "flange";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.01);

  // Current state
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  task.add(std::move(stage_state_current));

  // Open hand
  auto stage_open_hand = std::make_unique<mtc::stages::MoveTo>("open hand", sampling_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("open");
  task.add(std::move(stage_open_hand));

  // Define the target pose
  auto const target_pose = []{
	  geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "world";
	  
    // Define roll, pitch, and yaw angles in radians
	  double roll = M_PI/2.0;  // Rotation around the x-axis
	  double pitch = 0.0;  // Rotation around the y-axis (90 degrees)
	  double yaw = 0.0;  // Rotation around the z-axis
	  // Convert roll, pitch, yaw to quaternion
	  tf2::Quaternion q;
	  q.setRPY(roll, pitch, yaw);

	  msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    msg.pose.orientation.w = q.w();
	  msg.pose.position.x = 0.45;
	  msg.pose.position.y = -0.10;  //to 10mm slide move to got to grasp the object
	  msg.pose.position.z = 0.25;
	  return msg;
	}();
  
  // Move to target pose
  auto move_to_target = std::make_unique<mtc::stages::MoveTo>("move to target", sampling_planner);
  move_to_target->setGroup(arm_group_name);
  move_to_target->setGoal(target_pose);
  task.add(std::move(move_to_target));

  // Define the slide pose
  auto const slide_pose = []{
	  geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "world";
	  
    // Define roll, pitch, and yaw angles in radians
	  double roll = M_PI/2.0;  // Rotation around the x-axis
	  double pitch = 0.0;  // Rotation around the y-axis (90 degrees)
	  double yaw = 0.0;  // Rotation around the z-axis
	  // Convert roll, pitch, yaw to quaternion
	  tf2::Quaternion q;
	  q.setRPY(roll, pitch, yaw);

	  msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    msg.pose.orientation.w = q.w();
	  msg.pose.position.x = 0.45;
	  msg.pose.position.y = -0.18;  //to 8cm slide move
	  msg.pose.position.z = 0.25;
	  return msg;
	}();
  
  // Move to slide pose
  auto move_to_slide = std::make_unique<mtc::stages::MoveTo>("move to slide", cartesian_planner);
  move_to_slide->setGroup(arm_group_name);
  move_to_slide->setGoal(slide_pose);
  task.add(std::move(move_to_slide));

  // Allow collision (hand, object) temporarily
  auto allow_collision = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand, object)");
  allow_collision->allowCollisions("object",
                                   task.getRobotModel()
                                       ->getJointModelGroup(hand_group_name)
                                       ->getLinkModelNamesWithCollisionGeometry(),
                                   true);
  task.add(std::move(allow_collision));

  // Allow collision (object, table) temporarily
  auto allow_collision_object_table = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (object, table)");
  allow_collision_object_table->allowCollisions("object", {"table"}, true);
  task.add(std::move(allow_collision_object_table));

  // Close hand
  auto stage_close_hand = std::make_unique<mtc::stages::MoveTo>("close hand", sampling_planner);
  stage_close_hand->setGroup(hand_group_name);
  stage_close_hand->setGoal("close");
  task.add(std::move(stage_close_hand));

  // Attach object to the gripper
  auto attach_object = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
  attach_object->attachObject("object", hand_frame);
  task.add(std::move(attach_object));

  // Lift object slightly to avoid collision with the table
  auto lift_object = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
  lift_object->properties().set("marker_ns", "lift_object");
  lift_object->properties().set("link", hand_frame);
  lift_object->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  lift_object->setMinMaxDistance(0.1, 0.15); // Increase lift height

  // Set lift direction
  geometry_msgs::msg::Vector3Stamped lift_direction;
  lift_direction.header.frame_id = "world";
  lift_direction.vector.z = 1.0; // Lift upwards
  lift_object->setDirection(lift_direction);
  task.add(std::move(lift_object));

  

  // Re-enable collision (object, table) after lifting
  auto reenable_collision_object_table = std::make_unique<mtc::stages::ModifyPlanningScene>("reenable collision (object, table)");
  reenable_collision_object_table->allowCollisions("object", {"table"}, false);
  task.add(std::move(reenable_collision_object_table));

  /* // Re-enable collision checking after grasping and lifting
  auto reenable_collision = std::make_unique<mtc::stages::ModifyPlanningScene>("reenable collision (hand, object)");
  reenable_collision->allowCollisions("object",
                                      task.getRobotModel()
                                          ->getJointModelGroup(hand_group_name)
                                          ->getLinkModelNamesWithCollisionGeometry(),
                                      false);
  task.add(std::move(reenable_collision)); */
  
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
  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
  /* // Move wrist joint back to its initial position
  std::map<std::string, double> initial_joint_positions;
  initial_joint_positions["wrist_3_joint"] = 3.14; // Adjust this value based on the initial position of wrist_3_joint
  auto move_to_initial = std::make_unique<mtc::stages::MoveTo>("reset wrist joint", sampling_planner);
  move_to_initial->setGroup(arm_group_name);
  move_to_initial->setGoal(initial_joint_positions);
  task.add(std::move(move_to_initial));
 */