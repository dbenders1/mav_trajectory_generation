/*
 * Simple example that shows a trajectory planner using
 *  mav_trajectory_generation.
 *
 *
 * Launch via
 *   roslaunch mav_trajectory_generation_example firefly_custom_traj_control.launch
 *
 * Wait for console to run through all gazebo/rviz messages and then
 * you should see the example below
 *  - After Enter, it receives the current uav position
 *  - After second enter, publishes trajectory information
 *  - After third enter, executes trajectory (sends it to the sampler)
 */
#include <mav_trajectory_generation_example/custom_planner.h>

int main(int argc, char** argv) {

  ros::init(argc, argv, "custom_planner");

  ros::NodeHandle n;
  CustomPlanner planner(n);
  ROS_WARN_STREAM("SLEEPING FOR 5s TO WAIT FOR CLEAR CONSOLE");
  ros::Duration(5.0).sleep();
  ROS_WARN_STREAM("WARNING: CONSOLE INPUT/OUTPUT ONLY FOR DEMONSTRATION!");

  // define set point
  Eigen::Vector3d position, velocity;
  position << 0.0, 1.0, 2.0;
  velocity << 0.0, 0.0, 0.0;

  // THIS SHOULD NORMALLY RUN INSIDE ROS::SPIN!!! JUST FOR DEMO PURPOSES LIKE THIS.
  ROS_WARN_STREAM("PRESS ENTER TO UPDATE CURRENT POSITION AND SEND TRAJECTORY");
  std::cin.get();
  for (int i = 0; i < 10; i++) {
    ros::spinOnce();  // process a few messages in the background - causes the uavPoseCallback to happen
  }

  ros::V_string args;
  ros::removeROSArgs(argc, argv, args);

  if (args.size() != 2 && args.size() != 3) {
    ROS_ERROR("Usage: mav_trajectory_generation_custom <waypoint_file>."
        "\nThe waypoint file should be structured as: space separated: wait_time [s] x[m] y[m] z[m] yaw[deg]).");
    return -1;
  }

  
  std::string wp_file_name;
  std::vector<Waypoint> waypoints;
  planner.readWaypoints(args.at(1).c_str(), &waypoints);


  mav_trajectory_generation::Trajectory trajectory;
  planner.planTrajectory(position, velocity, &trajectory);
  planner.publishTrajectory(trajectory);

  
  ROS_WARN_STREAM("DONE. GOODBYE.");

  return 0;
}
