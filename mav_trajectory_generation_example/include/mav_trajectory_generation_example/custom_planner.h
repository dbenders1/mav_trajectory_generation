#ifndef MAV_TRAJECTORY_GENERATION_CUSTOM_PLANNER_H
#define MAV_TRAJECTORY_GENERATION_CUSTOM_PLANNER_H

#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <eigen_conversions/eigen_msg.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>

class Waypoint {
 public:
  Waypoint()
      : waiting_time(0.01), yaw(0.0) {
  }

  Waypoint(double t, float x, float y, float z, float _yaw)
      : position(x, y, z), yaw(_yaw), waiting_time(t) {
  }

  Eigen::Vector3d position;
  double yaw;
  double waiting_time;
};

class CustomPlanner {
 public:
  CustomPlanner(ros::NodeHandle& nh);

  void uavOdomCallback(const nav_msgs::Odometry::ConstPtr& pose);

  void setMaxSpeed(double max_v);

  // Reads waypoints from a text file
  // bool readWaypoints(std::string wp_file_name);
  bool readWaypoints(std::string wp_file_name, std::vector<Waypoint>* waypoints);

  // Plans a trajectory to take off from the current position and
  // fly to the given altitude (while maintaining x,y, and yaw).
  bool planTrajectory(const Eigen::VectorXd& goal_pos,
                      const Eigen::VectorXd& goal_vel,
                      mav_trajectory_generation::Trajectory* trajectory);
                      
  bool planTrajectory(const Eigen::VectorXd& goal_pos,
                      const Eigen::VectorXd& goal_vel,
                      const Eigen::VectorXd& start_pos,
                      const Eigen::VectorXd& start_vel,
                      double v_max, double a_max,
                      mav_trajectory_generation::Trajectory* trajectory);
                      
  bool publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory);

 private:
  ros::Publisher pub_markers_;
  ros::Publisher pub_trajectory_;
  ros::Subscriber sub_odom_;

  ros::NodeHandle& nh_;
  Eigen::Affine3d current_pose_;
  Eigen::Vector3d current_velocity_;
  Eigen::Vector3d current_angular_velocity_;
  double max_v_; // m/s
  double max_a_; // m/s^2
  double max_ang_v_;
  double max_ang_a_;

  // Debug information
  bool verbose_;

};

#endif // MAV_TRAJECTORY_GENERATION_CUSTOM_PLANNER_H
