#include <mav_trajectory_generation_example/custom_planner.h>

CustomPlanner::CustomPlanner(ros::NodeHandle& nh) :
    nh_(nh),
    verbose_(false),
    max_v_(2.0),
    max_a_(2.0),
    current_velocity_(Eigen::Vector3d::Zero()),
    current_pose_(Eigen::Affine3d::Identity()) {
    
  // Load params
  if (!nh_.getParam(ros::this_node::getName() + "/verbose", verbose_)){
    ROS_WARN("[custom_planner] param verbose not found");
  }
  if (!nh_.getParam(ros::this_node::getName() + "/max_v", max_v_)){
    ROS_WARN("[custom_planner] param max_v not found");
  }
  if (!nh_.getParam(ros::this_node::getName() + "/max_a", max_a_)){
    ROS_WARN("[custom_planner] param max_a not found");
  }

  // create publisher for RVIZ markers
  pub_markers_ =
      nh.advertise<visualization_msgs::MarkerArray>("trajectory_markers", 0);

  pub_trajectory_ =
      nh.advertise<mav_planning_msgs::PolynomialTrajectory4D>("trajectory",
                                                              0);

  // subscriber for Odometry
  sub_odom_ =
      nh.subscribe("uav_pose", 1, &CustomPlanner::uavOdomCallback, this);
}

// Callback to get current Pose of UAV
void CustomPlanner::uavOdomCallback(const nav_msgs::Odometry::ConstPtr& odom) {

  // store current position in our planner
  tf::poseMsgToEigen(odom->pose.pose, current_pose_);

  // store current vleocity
  tf::vectorMsgToEigen(odom->twist.twist.linear, current_velocity_);
}

// Method to set maximum speed.
void CustomPlanner::setMaxSpeed(const double max_v) {
  max_v_ = max_v;
}

// Reads waypoints from a text file
bool CustomPlanner::readWaypoints(std::string wp_file_name, std::vector<Waypoint>* waypoints) {
  ROS_INFO_STREAM("Called CustomPlanner::readWaypoints with filename " << wp_file_name << ".");

  const float DEG_2_RAD = M_PI / 180.0;

  // Open file and read waypoints
  std::ifstream wp_file(wp_file_name);
  if (wp_file.is_open()) {
    double t, x, y, z, yaw;
    // Only read complete waypoints.
    while (wp_file >> t >> x >> y >> z >> yaw) {
      waypoints->push_back(Waypoint(t, x, y, z, yaw * DEG_2_RAD));
    }
    wp_file.close();
    ROS_INFO("Read %d waypoints.", (int) waypoints->size());
  } else {
    ROS_ERROR_STREAM("Unable to open poses file: " << wp_file_name << ".");
    return -1;
  }

  // Print waypoints to console in verbose mode
  if (verbose_) {
    for (size_t i = 0; i < waypoints->size(); ++i) {
      ROS_INFO_STREAM("Waypoint " << i+1 << " read [t x y z yaw]: " << (*waypoints)[i].waiting_time << " " << (*waypoints)[i].position[0] << " " << (*waypoints)[i].position[1] << " " << (*waypoints)[i].position[2] << " " << (*waypoints)[i].yaw << ".");
    }
  }

  return 0;
}

// Plans a trajectory from the current position to the a goal position and velocity
// we neglect attitude here for simplicity
bool CustomPlanner::planTrajectory(const Eigen::VectorXd& goal_pos,
                                    const Eigen::VectorXd& goal_vel,
                                    mav_trajectory_generation::Trajectory* trajectory) {


  // 3 Dimensional trajectory => through cartesian space, no orientation
  const int dimension = 3;

  // Array for all waypoints and their constrains
  mav_trajectory_generation::Vertex::Vector vertices;

  // Optimize up to 4th order derivative (SNAP)
  const int derivative_to_optimize =
      mav_trajectory_generation::derivative_order::SNAP;

  // we have 2 vertices:
  // Start = current position
  // end = desired position and velocity
  mav_trajectory_generation::Vertex start(dimension), end(dimension);


  /******* Configure start point *******/
  // set start point constraints to current position and set all derivatives to zero
  start.makeStartOrEnd(current_pose_.translation(),
                       derivative_to_optimize);

  // set start point's velocity to be constrained to current velocity
  start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                      current_velocity_);

  // add waypoint to list
  vertices.push_back(start);


  /******* Configure end point *******/
  // set end point constraints to desired position and set all derivatives to zero
  end.makeStartOrEnd(goal_pos,
                     derivative_to_optimize);

  // set start point's velocity to be constrained to current velocity
  end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                    goal_vel);

  // add waypoint to list
  vertices.push_back(end);

  // estimate initial segment times
  std::vector<double> segment_times;
  segment_times = estimateSegmentTimes(vertices, max_v_, max_a_);

  // Set up polynomial solver with default params
  mav_trajectory_generation::NonlinearOptimizationParameters parameters;

  // set up optimization problem
  const int N = 10;
  mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
  opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

  // constrain velocity and acceleration
  opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, max_v_);
  opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, max_a_);

  // solve trajectory
  opt.optimize();

  // get trajectory as polynomial parameters
  opt.getTrajectory(&(*trajectory));

  return true;
}

bool CustomPlanner::publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory){
  // send trajectory as markers to display them in RVIZ
  visualization_msgs::MarkerArray markers;
  double distance =
      0.2; // Distance by which to separate additional markers. Set 0.0 to disable.
  std::string frame_id = "world";

  mav_trajectory_generation::drawMavTrajectory(trajectory,
                                               distance,
                                               frame_id,
                                               &markers);
  pub_markers_.publish(markers);

  // send trajectory to be executed on UAV
  mav_planning_msgs::PolynomialTrajectory msg;
  mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory,
                                                                 &msg);
  msg.header.frame_id = "world";
  pub_trajectory_.publish(msg);

  return true;
}
