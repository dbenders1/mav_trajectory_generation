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
  // Initialize ROS node
  ros::init(argc, argv, "custom_planner");
  ros::NodeHandle n;

  CustomPlanner planner(n);

  ROS_WARN_STREAM("SLEEPING FOR 5s TO WAIT FOR CLEAR CONSOLE");
  ros::Duration(5.0).sleep();
  ROS_WARN_STREAM("WARNING: CONSOLE INPUT/OUTPUT ONLY FOR DEMONSTRATION!");


  // Define waypoints
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

  // Manually 6D
  // Eigen::VectorXd pose, twist;
  // pose.resize(6);
  // twist.resize(6);
  // Eigen::Vector3d position, rotation_vec;
  // Eigen::Matrix3d rotation_mat;
  // Eigen::Vector3d unitx = Eigen::Vector3d::UnitX();
  // Eigen::Vector3d unity = Eigen::Vector3d::UnitY();
  // Eigen::Vector3d unitz = Eigen::Vector3d::UnitZ();
  // Eigen::AngleAxisd test = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
  // rotation_mat = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX())
  //            * Eigen::AngleAxisd(M_PI / 2.0,  Eigen::Vector3d::UnitY())
  //            * Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ());
  // mav_msgs::vectorFromRotationMatrix(rotation_mat, &rotation_vec);
  // ROS_INFO_STREAM("unitx: " << unitx);
  // ROS_INFO_STREAM("unity: " << unity);
  // ROS_INFO_STREAM("unitz: " << unitz);
  // ROS_INFO_STREAM("rotation_vec: " << rotation_vec);
  // ROS_INFO_STREAM("rotation_mat: " << rotation_mat);
  // position << 0.0, 1.0, 2.0;
  // pose << position, rotation_vec;
  // twist << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  //  Manually 4D
  // Eigen::VectorXd pos_yaw, vel_yawrate;
  // pos_yaw.resize(4);
  // vel_yawrate.resize(4);
  // pos_yaw << 0.0, 0.0, 2.0, M_PI;
  // vel_yawrate << 0.0, 0.0, 0.0, 0.0;


  // From file
  std::string wp_file_name;
  std::vector<Waypoint> waypoints;
  planner.read4DWaypoints(args.at(1).c_str(), &waypoints);


  // Plan trajectory and publish
  mav_trajectory_generation::Trajectory trajectory;
  planner.plan4DTrajectory(&waypoints, &trajectory);
  // planner.planTrajectory(pos_yaw, vel_yawrate, &trajectory);
  // planner.planTrajectory(pose, twist, &trajectory);
  planner.publishTrajectory(trajectory);


  ROS_WARN_STREAM("DONE. GOODBYE.");

  return 0;
}
