#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <thread>  // <---- add this to the set of includes at the top
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include "moveit/moveit_cpp/moveit_cpp.h"
#include "moveit/moveit_cpp/planning_component.h"
#include "moveit_msgs/msg/cartesian_trajectory.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include <geometry_msgs/msg/pose_array.hpp>

//write to file
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <boost/archive/binary_oarchive.hpp>


// Define a global variable to store PoseArray messages
std::vector<geometry_msgs::msg::Pose> pose_array_messages;

// Define a callback function for the PoseArray subscriber
void poseArrayCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    pose_array_messages = msg->poses;
    std::cout << "Poses received from the topic" << std::endl;
}

void printRobotTrajectory(const moveit_msgs::msg::RobotTrajectory &trajectory) {
    RCLCPP_INFO(rclcpp::get_logger("print_trajectory"), "Trajectory points:");
    for (size_t i = 0; i < trajectory.joint_trajectory.points.size(); ++i) {
        const auto &point = trajectory.joint_trajectory.points[i];
        RCLCPP_INFO(rclcpp::get_logger("print_trajectory"), "Point %zu:", i + 1);
        RCLCPP_INFO(rclcpp::get_logger("print_trajectory"), "Positions:");
        for (size_t j = 0; j < point.positions.size(); ++j) {
            RCLCPP_INFO(rclcpp::get_logger("print_trajectory"), "  Joint %zu: %f", j + 1, point.positions[j]);
        }
    }
}

void writeTrajectoryToPickle(const moveit_msgs::msg::RobotTrajectory &trajectory){
  // Convert RobotTrajectory message to a dictionary
  YAML::Node trajectory_dict;
  trajectory_dict["joint_names"] = trajectory.joint_trajectory.joint_names;
  trajectory_dict["points"] = YAML::Load("[]");

  for (size_t i = 0; i < trajectory.joint_trajectory.points.size(); ++i)
  {
    YAML::Node point;
    point["positions"] = trajectory.joint_trajectory.points[i].positions;
    point["velocities"] = trajectory.joint_trajectory.points[i].velocities;
    point["accelerations"] = trajectory.joint_trajectory.points[i].accelerations;
    point["effort"] = trajectory.joint_trajectory.points[i].effort;
    // point["time_from_start"] = trajectory.joint_trajectory.points[i].time_from_start.toSec();
    trajectory_dict["points"].push_back(point);
  }

  // Write dictionary to .pkl file
  std::ofstream file("trajectory.pkl", std::ios::binary);
  // boost::archive::binary_oarchive oa(file);
  file << trajectory_dict;
  file.close();

  RCLCPP_INFO(rclcpp::get_logger("save_trajectory"), "Trajectory written to trajectory.pkl"); 

}



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
  move_group_interface.setPlannerId("TRRT");

  // Set the planning time
  move_group_interface.setPlanningTime(10.0); // Set maximum time for planning

  // move_group_interface.setGoalTimeout(30.0); //set maximum time for execution

  //move_group_interface.setNumPlanningAttempts(10);

  // Construct and initialize MoveItVisualTools
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
      node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
      move_group_interface.getRobotModel()};
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  
  auto const prompt = [&moveit_visual_tools](auto text) {
    moveit_visual_tools.prompt(text);
  };
 
   // Subscriber to PoseArray topic
  auto pose_array_subscriber = node->create_subscription<geometry_msgs::msg::PoseArray>(
      "/hole_wrt_base_link", 10, poseArrayCallback);

//ask the user to press enter as soon as the poseArray is received
prompt("Press 'Next' as soon as the pose array is received ");
moveit_visual_tools.trigger();


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



  // // Set a target Pose
  // auto const target_pose = [&move_group_interface] {
  //   geometry_msgs::msg::Pose msg;
  //   auto current_pose = move_group_interface.getCurrentPose();
  //   // msg.orientation.x = current_pose.pose.orientation.x;
  //   // msg.orientation.y = current_pose.pose.orientation.y;
  //   // msg.orientation.z = current_pose.pose.orientation.z;
  //   // msg.orientation.w = current_pose.pose.orientation.w;
  //   // msg.position.x = current_pose.pose.position.x;
  //   // msg.position.y = current_pose.pose.position.y;
  //   // msg.position.z = current_pose.pose.position.z+0.1;

  //   //target pose manually taken from the visual process
  //   msg.orientation.x = 1.0;
  //   msg.orientation.y = 0.0;
  //   msg.orientation.z = 0.0;
  //   msg.orientation.w = 0.0;
  //   msg.position.x =  -0.00978492;
  //   msg.position.y = -0.34217947;
  //   msg.position.z = 0.10849668 + 0.005;

  // return msg;
  // }();


  // //create cartesian trajectory
  // //TODO --> chatgpt
  // // Define waypoints for the Cartesian path
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

  // //add points to the waypoints vector
  // waypoints.push_back(start_pose); // Starting pose
  // waypoints.push_back(target_pose); // target pose
  // waypoints.push_back(start_pose); //return to Starting pose




  
  //create the waypoints vector
  
  waypoints.push_back(start_pose);

  for (size_t i = 0; i < pose_array_messages.size(); ++i) {
    // waypoints.push_back(start_pose);
    geometry_msgs::msg::Pose msg;
    geometry_msgs::msg::Pose msg_up;
    geometry_msgs::msg::Pose msg_up_1;
    msg.orientation.x = pose_array_messages[i].orientation.x;
    msg.orientation.y = pose_array_messages[i].orientation.y;
    msg.orientation.z = pose_array_messages[i].orientation.z;
    msg.orientation.w = pose_array_messages[i].orientation.w;
    msg.position.x = pose_array_messages[i].position.x;
    msg.position.y = pose_array_messages[i].position.y;
    msg.position.z = pose_array_messages[i].position.z;

    msg_up.orientation.x = pose_array_messages[i].orientation.x;
    msg_up.orientation.y = pose_array_messages[i].orientation.y;
    msg_up.orientation.z = pose_array_messages[i].orientation.z;
    msg_up.orientation.w = pose_array_messages[i].orientation.w;
    msg_up.position.x = pose_array_messages[i].position.x;
    msg_up.position.y = pose_array_messages[i].position.y;
    msg_up.position.z = pose_array_messages[i].position.z+0.1;

    msg_up_1.orientation.x = pose_array_messages[i].orientation.x;
    msg_up_1.orientation.y = pose_array_messages[i].orientation.y;
    msg_up_1.orientation.z = pose_array_messages[i].orientation.z;
    msg_up_1.orientation.w = pose_array_messages[i].orientation.w;
    msg_up_1.position.x = pose_array_messages[i].position.x+0.1;
    msg_up_1.position.y = pose_array_messages[i].position.y;
    msg_up_1.position.z = pose_array_messages[i].position.z+0.1;

    
    // waypoints.push_back(msg);
    // waypoints.push_back(msg);
    // waypoints.push_back(msg_up);
    // waypoints.push_back(msg_up_1);
    waypoints.push_back(msg_up);
    waypoints.push_back(msg);
    waypoints.push_back(msg_up);
  }
  waypoints.push_back(start_pose);

    // Print the waypoints vector -->visual debug
  std::cout << "Waypoints:" << std::endl;
  for (size_t i = 0; i < waypoints.size(); ++i) {
    const geometry_msgs::msg::Pose& pose = waypoints[i];
    std::cout << "  - Position: (" << pose.position.x << ", "
              << pose.position.y << ", " << pose.position.z << ")" << std::endl;

    std::cout << "    - Orientation (quaternion):" << std::endl;
    std::cout << "      - w: " << pose.orientation.w << std::endl;
    std::cout << "      - x: " << pose.orientation.x << std::endl;
    std::cout << "      - y: " << pose.orientation.y << std::endl;
    std::cout << "      - z: " << pose.orientation.z << std::endl;
  }

  

  // Compute Cartesian path
  prompt("Press 'Next' in the RvizVisualToolsGui window to plan a cartesian trajectory");
  moveit_visual_tools.trigger();
  moveit_msgs::msg::RobotTrajectory trajectory;
  //collision avoidance is ensured by a non mandatory input (avoid_collision = true)
  double fraction = move_group_interface.computeCartesianPath(waypoints, 0.005, 0, trajectory);
  RCLCPP_INFO(logger,"Cartesian frajectory fraction value: %f", fraction);


  // // Scale the velocity of the trajectory (adjust as needed)
  // double scale_factor = 0.5; // Scale the velocity to 50%
  // for (auto& point : trajectory.joint_trajectory.points) {
  //     for (auto& velocity : point.velocities) {
  //         velocity *= scale_factor;
  //     }
  // }




  // Call print function
  // printRobotTrajectory(trajectory);
  // Call write to pickle function
  writeTrajectoryToPickle(trajectory);


  
  if(fraction > 0){
    //ask the user to confirm and execute the planned trajectory
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;

    // print Final Joint Positions
    auto final_joint_values = plan.trajectory_.joint_trajectory.points.back().positions;  
    RCLCPP_INFO(logger, "Final Joint Positions:");
    for (size_t i = 0; i < final_joint_values.size(); ++i) {
        RCLCPP_INFO(logger, "Joint %zu: %f", i+1, final_joint_values[i]);
    }

    prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
    moveit_visual_tools.trigger();
    move_group_interface.execute(plan);

  }
  else{
    //trajectory planner failed !
    moveit_visual_tools.trigger();
    RCLCPP_ERROR(logger, "Planning failed!");
  }




  

  // Shutdown ROS
  rclcpp::shutdown();  // <--- This will cause the spin function in the thread to return
  spinner.join();  // <--- Join the thread before exiting
  return 0;
}