// I tried to use a cartesian trajectory
// the robot perform large motion of the whole arm and significantly change the state of the robot
// even if the required cartesian space motion is small



#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <thread>  // <---- add this to the set of includes at the top
#include <moveit/planning_scene_interface/planning_scene_interface.h>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Next step goes here
  // Spin up a SingleThreadedExecutor for MoveItVisualTools to interact with ROS
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");

  // Set the planner to use Cartesian planner
  //move_group_interface.setPlannerId("TRRT"); // Set the planner to TRRT (Trajectory Rollout and Retraction)
  move_group_interface.setPlannerId("RRT");

  // Set the planning time
  move_group_interface.setPlanningTime(10.0); // Set maximum time for planning

  // Construct and initialize MoveItVisualTools
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
      node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
      move_group_interface.getRobotModel()};
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  
  auto const prompt = [&moveit_visual_tools](auto text) {
    moveit_visual_tools.prompt(text);
  };
 



  // Function to print current robot pose
    auto print_current_pose = [&move_group_interface, &logger]() {
        auto current_pose = move_group_interface.getCurrentPose();
        RCLCPP_INFO(logger, "Current Robot Pose:");
        RCLCPP_INFO(logger, "Position: x=%f, y=%f, z=%f",
                    current_pose.pose.position.x,
                    current_pose.pose.position.y,
                    current_pose.pose.position.z);
        RCLCPP_INFO(logger, "Orientation: x=%f, y=%f, z=%f, w=%f",
                    current_pose.pose.orientation.x,
                    current_pose.pose.orientation.y,
                    current_pose.pose.orientation.z,
                    current_pose.pose.orientation.w);

        auto current_joint_values = move_group_interface.getCurrentJointValues();
        RCLCPP_INFO(logger, "Current Joint Positions:");
        for (size_t i = 0; i < current_joint_values.size(); ++i) {
            RCLCPP_INFO(logger, "Joint %zu: %f", i+1, current_joint_values[i]);
        }
    };
    // Print current robot pose
    print_current_pose();



  // Set a target Pose
  auto const target_pose = [&move_group_interface] {
    geometry_msgs::msg::Pose msg;
    auto current_pose = move_group_interface.getCurrentPose();
    // msg.orientation.y = 0;
    // msg.orientation.x = 1;
    // msg.orientation.w = 0;
    // msg.position.x = 0.1;
    // msg.position.y = -0.4;
    // msg.position.z = 0.5;
    msg.orientation.x = current_pose.pose.orientation.x;
    msg.orientation.y = current_pose.pose.orientation.y;
    msg.orientation.z = current_pose.pose.orientation.z;
    msg.orientation.w = current_pose.pose.orientation.w;
    msg.position.x = current_pose.pose.position.x;
    msg.position.y = current_pose.pose.position.y;
    msg.position.z = current_pose.pose.position.z+0.1;
  return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);


  //create cartesian trajectory
  //TODO --> chatgpt
  // Define waypoints for the Cartesian path
  std::vector<geometry_msgs::msg::Pose> waypoints;
  //set start pose (current pose)
  auto const start_pose = [&move_group_interface] {
    geometry_msgs::msg::Pose msg;
    auto current_pose = move_group_interface.getCurrentPose();
    msg.orientation.x = current_pose.pose.orientation.x;
    msg.orientation.y = current_pose.pose.orientation.y;
    msg.orientation.z = current_pose.pose.orientation.z;
    msg.orientation.w = current_pose.pose.orientation.w;
    msg.position.x = current_pose.pose.position.x;
    msg.position.y = current_pose.pose.position.y;
    msg.position.z = current_pose.pose.position.z;
  return msg;
  }();
  waypoints.push_back(start_pose); // Starting pose
  waypoints.push_back(target_pose); // target pose



  // Compute Cartesian path
  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = move_group_interface.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
  RCLCPP_INFO(logger,"%f", fraction);


  // Create a plan to that target pose
  prompt("Press 'Next' in the RvizVisualToolsGui window to plan");
  //draw_title("Planning");
  moveit_visual_tools.trigger();
  auto const [success, plan] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if (success) {
    // Print final joint positions
        auto final_joint_values = plan.trajectory_.joint_trajectory.points.back().positions;
        RCLCPP_INFO(logger, "Final Joint Positions:");
        for (size_t i = 0; i < final_joint_values.size(); ++i) {
            RCLCPP_INFO(logger, "Joint %zu: %f", i+1, final_joint_values[i]);
        }

    //draw_trajectory_tool_path(plan.trajectory_);
    moveit_visual_tools.trigger();
    prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
    moveit_visual_tools.trigger();
    move_group_interface.execute(plan);
  } else {
    moveit_visual_tools.trigger();
    RCLCPP_ERROR(logger, "Planning failed!");
  }



  

  // Shutdown ROS
  rclcpp::shutdown();  // <--- This will cause the spin function in the thread to return
  spinner.join();  // <--- Join the thread before exiting
  return 0;
}
