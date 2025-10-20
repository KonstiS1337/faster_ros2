/* ----------------------------------------------------------------------------
 * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include "faster_ros.hpp"

// This object is created in the faster_ros_node
FasterRos::FasterRos() : Node("faster_ros_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
  auto node_ptr = shared_from_this();
  param_success_ = safeGetParam(node_ptr, "use_ff", par_.use_ff);
  param_success_ = safeGetParam(node_ptr, "visual", par_.visual);
  param_success_ = safeGetParam(node_ptr, "dc", par_.dc);
  param_success_ = safeGetParam(node_ptr, "goal_radius", par_.goal_radius);
  param_success_ = safeGetParam(node_ptr, "drone_radius", par_.drone_radius);
  param_success_ = safeGetParam(node_ptr, "force_goal_height", par_.force_goal_height);
  param_success_ = safeGetParam(node_ptr, "goal_height", par_.goal_height);
  param_success_ = safeGetParam(node_ptr, "N_safe", par_.N_safe);
  param_success_ = safeGetParam(node_ptr, "N_whole", par_.N_whole);
  param_success_ = safeGetParam(node_ptr, "Ra", par_.Ra);
  param_success_ = safeGetParam(node_ptr, "w_max", par_.w_max);
  param_success_ = safeGetParam(node_ptr, "alpha_filter_dyaw", par_.alpha_filter_dyaw);
  param_success_ = safeGetParam(node_ptr, "z_ground", par_.z_ground);
  param_success_ = safeGetParam(node_ptr, "z_max", par_.z_max);
  param_success_ = safeGetParam(node_ptr, "inflation_jps", par_.inflation_jps);
  param_success_ = safeGetParam(node_ptr, "factor_jps", par_.factor_jps);
  param_success_ = safeGetParam(node_ptr, "v_max", par_.v_max);
  param_success_ = safeGetParam(node_ptr, "a_max", par_.a_max);
  param_success_ = safeGetParam(node_ptr, "j_max", par_.j_max);
  param_success_ = safeGetParam(node_ptr, "gamma_whole", par_.gamma_whole);
  param_success_ = safeGetParam(node_ptr, "gammap_whole", par_.gammap_whole);
  param_success_ = safeGetParam(node_ptr, "increment_whole", par_.increment_whole);
  param_success_ = safeGetParam(node_ptr, "gamma_safe", par_.gamma_safe);
  param_success_ = safeGetParam(node_ptr, "gammap_safe", par_.gammap_safe);
  param_success_ = safeGetParam(node_ptr, "increment_safe", par_.increment_safe);
  param_success_ = safeGetParam(node_ptr, "delta_a", par_.delta_a);
  param_success_ = safeGetParam(node_ptr, "delta_H", par_.delta_H);
  param_success_ = safeGetParam(node_ptr, "max_poly_whole", par_.max_poly_whole);
  param_success_ = safeGetParam(node_ptr, "max_poly_safe", par_.max_poly_safe);
  param_success_ = safeGetParam(node_ptr, "dist_max_vertexes", par_.dist_max_vertexes);
  param_success_ = safeGetParam(node_ptr, "gurobi_threads", par_.gurobi_threads);
  param_success_ = safeGetParam(node_ptr, "gurobi_verbose", par_.gurobi_verbose);
  param_success_ = safeGetParam(node_ptr, "use_faster", par_.use_faster);
  param_success_ = safeGetParam(node_ptr, "is_ground_robot", par_.is_ground_robot);

  // And now obtain the parameters from the mapper
  std::vector<double> world_dimensions;
  param_success_ = safeGetParam(node_ptr, "mapper/world_dimensions", world_dimensions);
  param_success_ = safeGetParam(node_ptr, "mapper/resolution", par_.res);

  par_.wdx = world_dimensions[0];
  par_.wdy = world_dimensions[1];
  par_.wdz = world_dimensions[2];
  if(!param_success_) return;

  RCLCPP_INFO_STREAM(this->get_logger(),bold << green << "world_dimensions=" << world_dimensions << reset << std::endl);
  RCLCPP_INFO_STREAM(this->get_logger(),bold << green << "resolution=" << par_.res << reset << std::endl);
  RCLCPP_INFO_STREAM(this->get_logger(),"Parameters obtained" << std::endl);

  if (par_.N_safe <= par_.max_poly_safe + 2)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(),bold << red << "Needed: N_safe>=max_poly+ 2 at least" << reset << std::endl);  // To decrease the probability of not finding a solution
    rclcpp::shutdown();
    return;
  }
  if (par_.N_whole <= par_.max_poly_whole + 2)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(),bold << red << "Needed: N_whole>=max_poly + 2 at least" << reset
              << std::endl);  // To decrease the probability of not finding a solution
    rclcpp::shutdown();
    return;
  }

  if (par_.factor_jps * par_.res / 2.0 > par_.inflation_jps)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), bold << red << "Needed: par_.factor_jps * par_.res / 2 <= par_.inflation_jps" << reset
              << std::endl);  // If not JPS will find a solution between the voxels.
    rclcpp::shutdown();
    return;
  }



  // Initialize FASTER
  faster_ptr_ = std::unique_ptr<Faster>(new Faster(par_));
  RCLCPP_INFO(this->get_logger(),"Planner initialized");

  // Publishers
  rclcpp::QoS qos_latched(1);
  qos_latched.transient_local();  // Like latch in ROS 1
  pub_goal_ = this->create_publisher<faster_msgs::msg::Goal>("goal", 1);
  pub_traj_whole_ = this->create_publisher<nav_msgs::msg::Path>("traj_whole", 1);
  pub_traj_safe_ = this->create_publisher<nav_msgs::msg::Path>("traj_safe", 1);
  pub_setpoint_ = this->create_publisher<visualization_msgs::msg::Marker>("setpoint", 1);
  pub_intersectionI_ = this->create_publisher<visualization_msgs::msg::Marker>("intersection_I", 1);
  pub_point_G_ = this->create_publisher<geometry_msgs::msg::PointStamped>("point_G", 1);
  pub_point_G_term_ = this->create_publisher<geometry_msgs::msg::PointStamped>("point_G_term", 1);
  pub_point_E_ = this->create_publisher<visualization_msgs::msg::Marker>("point_E", 1);
  pub_point_R_ = this->create_publisher<visualization_msgs::msg::Marker>("point_R", 1);
  pub_point_M_ = this->create_publisher<visualization_msgs::msg::Marker>("point_M", 1);
  pub_point_H_ = this->create_publisher<visualization_msgs::msg::Marker>("point_H", 1);
  pub_point_A_ = this->create_publisher<visualization_msgs::msg::Marker>("point_A", 1);
  pub_actual_traj_ = this->create_publisher<visualization_msgs::msg::Marker>("actual_traj", 1);
  pub_path_jps1_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("path_jps1", 1);
  pub_path_jps2_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("path_jps2", 1);
  pub_path_jps_whole_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("path_jps_whole", 1);
  pub_path_jps_safe_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("path_jps_safe", 1);
  poly_whole_pub_ = this->create_publisher<faster_msgs::msg::PolyhedronArray>("poly_whole",qos_latched); // Latching not needed
  poly_safe_pub_ = this->create_publisher<faster_msgs::msg::PolyhedronArray>("poly_safe", qos_latched);
  pub_jps_inters_ = this->create_publisher<geometry_msgs::msg::PointStamped>("jps_intersection", 1);
  pub_traj_committed_colored_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("traj_committed_colored", 1);
  pub_traj_whole_colored_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("traj_whole_colored", 1);
  pub_traj_safe_colored_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("traj_safe_colored", 1);

  // Subscribers
  occup_grid_sub_.subscribe(this, "occupancy_grid", rmw_qos_profile_sensor_data);
  unknown_grid_sub_.subscribe(this, "unknown_grid",   rmw_qos_profile_sensor_data);

  sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(MySyncPolicy(10), occup_grid_sub_, unknown_grid_sub_);
  sync_->registerCallback(std::bind(&FasterRos::mapCB, this, std::placeholders::_1, std::placeholders::_2));

  sub_goal_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("term_goal", 1, std::bind(&FasterRos::terminalGoalCB, this, std::placeholders::_1));
  sub_mode_ = this->create_subscription<faster_msgs::msg::Mode>("mode", 1, std::bind(&FasterRos::modeCB, this, std::placeholders::_1));
  sub_state_ = this->create_subscription<faster_msgs::msg::State>("state", 1, std::bind(&FasterRos::stateCB, this, std::placeholders::_1));

  // Timers
  pubCBTimer_ = this->create_wall_timer(std::chrono::duration<double>(par_.dc),std::bind(&FasterRos::pubCB, this));
  replanCBTimer_ = this->create_wall_timer(std::chrono::duration<double>(par_.dc),std::bind(&FasterRos::replanCB, this));


  // For now stop all these subscribers/timers until we receive GO
 
  occup_grid_sub_.unsubscribe();
  unknown_grid_sub_.unsubscribe();
  pubCBTimer_->cancel();
  replanCBTimer_->cancel();

  // Markers
  setpoint_ = getMarkerSphere(0.35, ORANGE_TRANS);
  R_ = getMarkerSphere(0.35, ORANGE_TRANS);
  I_ = getMarkerSphere(0.35, YELLOW);
  E_ = getMarkerSphere(0.35, RED);
  M_ = getMarkerSphere(0.35, BLUE);
  H_ = getMarkerSphere(0.35, GREEN);
  A_ = getMarkerSphere(0.35, RED);

  // If you want another thread for the replanCB: replanCBTimer_ = nh_.createTimer(ros::Duration(par_.dc),
  // &FasterRos::replanCB, this);
  // Init tft2 Buffer
  clearMarkerActualTraj();

  init_ = true;
}

FasterRos::~FasterRos() {
  RCLCPP_INFO(this->get_logger(),"Destroying node.");
}

void FasterRos::replanCB()
{
  if (rclcpp::ok())
  {
    vec_Vecf<3> JPS_safe;
    vec_Vecf<3> JPS_whole;
    vec_E<Polyhedron<3>> poly_safe;
    vec_E<Polyhedron<3>> poly_whole;
    std::vector<state> X_safe;
    std::vector<state> X_whole;

    faster_ptr_->replan(JPS_safe, JPS_whole, poly_safe, poly_whole, X_safe, X_whole);
    clearJPSPathVisualization(2);
    publishJPSPath(JPS_safe, JPS_SAFE);
    publishJPSPath(JPS_whole, JPS_WHOLE);

    publishPoly(poly_safe, SAFE);
    publishPoly(poly_whole, WHOLE);
    pubTraj(X_safe, SAFE_COLORED);
    pubTraj(X_whole, WHOLE_COLORED);
  }
}

void FasterRos::publishPoly(const vec_E<Polyhedron<3>>& poly, int type)
{
  faster_msgs::msg::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros_faster(poly);
  poly_msg.header.frame_id = world_name_;

  switch (type)
  {
    case SAFE:
      poly_safe_pub_->publish(poly_msg);
      break;
    case WHOLE:
      poly_whole_pub_->publish(poly_msg);
      break;
  }
}

void FasterRos::stateCB(const faster_msgs::msg::State::ConstPtr& msg)
{
  state state_tmp;
  state_tmp.setPos(msg->pos.x, msg->pos.y, msg->pos.z);
  state_tmp.setVel(msg->vel.x, msg->vel.y, msg->vel.z);
  state_tmp.setAccel(0.0, 0.0, 0.0);
  double roll, pitch, yaw;
  quaternion2Euler(msg->quat, roll, pitch, yaw);
  state_tmp.setYaw(yaw);
  faster_ptr_->updateState(state_tmp);
}

void FasterRos::modeCB(const faster_msgs::msg::Mode& msg)
{
  // faster_ptr_->changeMode(msg.mode);

  if (msg.mode != msg.GO)
  {  // FASTER DOES NOTHING
    occup_grid_sub_.unsubscribe();
    unknown_grid_sub_.unsubscribe();
    pubCBTimer_->cancel();
    replanCBTimer_->cancel();
    faster_ptr_->resetInitialization();
  }
  else
  {  // The mode changed to GO
    occup_grid_sub_.subscribe();
    unknown_grid_sub_.subscribe();


    pubCBTimer_->reset();
    replanCBTimer_->reset();
  }
}

void FasterRos::pubCB()
{
  state next_goal;
  if (faster_ptr_->getNextGoal(next_goal))
  {
    faster_msgs::msg::Goal quadGoal;
    // visualization_msgs::Marker setpoint;
    // Pub setpoint maker.  setpoint_ is the last quadGoal sent to the drone

    // printf("Publicando Goal=%f, %f, %f\n", quadGoal_.pos.x, quadGoal_.pos.y, quadGoal_.pos.z);

    const auto v = eigen2rospoint(next_goal.pos);
    quadGoal.p.x = v.x;
    quadGoal.p.y = v.y;
    quadGoal.p.z = v.z;
    quadGoal.dyaw = next_goal.dyaw;
    quadGoal.yaw = next_goal.yaw;
    quadGoal.header.stamp = this->now();
    quadGoal.header.frame_id = world_name_;

    pub_goal_->publish(quadGoal);

    setpoint_.header.stamp = this->now();
    setpoint_.pose.position.x = quadGoal.p.x;
    setpoint_.pose.position.y = quadGoal.p.y;
    setpoint_.pose.position.z = quadGoal.p.z;

    pub_setpoint_->publish(setpoint_);
  }
}

void FasterRos::clearJPSPathVisualization(int i)
{
  switch (i)
  {
    case JPSk_NORMAL:
      clearMarkerArray(std::make_shared<visualization_msgs::msg::MarkerArray>(path_jps1_),i);
      break;
    case JPS2_NORMAL:
      clearMarkerArray(std::make_shared<visualization_msgs::msg::MarkerArray>(path_jps2_), i);
      break;
    case JPS_WHOLE:
      clearMarkerArray(std::make_shared<visualization_msgs::msg::MarkerArray>(path_jps_whole_), i);
      break;
    case JPS_SAFE:
      clearMarkerArray(std::make_shared<visualization_msgs::msg::MarkerArray>(path_jps_safe_), i);
      break;
    case WHOLE_COLORED:
      clearMarkerArray(std::make_shared<visualization_msgs::msg::MarkerArray>(traj_whole_colored_), i);
      break;
    case COMMITTED_COLORED:
      clearMarkerArray(std::make_shared<visualization_msgs::msg::MarkerArray>(traj_committed_colored_), i);
      break;
    case SAFE_COLORED:
      clearMarkerArray(std::make_shared<visualization_msgs::msg::MarkerArray>(traj_safe_colored_), i);
      break;
  }
}

void FasterRos::clearMarkerArray(visualization_msgs::msg::MarkerArray::SharedPtr tmp, int c)
{
  if (tmp->markers.size() == 0)
  {
    return;
  }
  int id_begin = tmp->markers[0].id;
  // int id_end = (*path).markers[markers.size() - 1].id;

  for (int i = 0; i < (*tmp).markers.size(); i++)
  {
    visualization_msgs::msg::Marker m;
    m.type = visualization_msgs::msg::Marker::ARROW;
    m.action = visualization_msgs::msg::Marker::DELETE;
    m.id = i + id_begin;
    (*tmp).markers[i] = m;
  }
  switch(c) {
    case JPSk_NORMAL:
      pub_path_jps1_->publish(*tmp);
      break;
    case JPS2_NORMAL:
      pub_path_jps2_->publish(*tmp);
      break;
    case JPS_WHOLE:
      pub_path_jps_whole_->publish(*tmp);
      break;
    case JPS_SAFE:
      pub_path_jps_safe_->publish(*tmp);
      break;
    case WHOLE_COLORED:
      pub_traj_whole_colored_->publish(*tmp);
      break;
    case COMMITTED_COLORED:
      pub_traj_committed_colored_->publish(*tmp);
      break;
    case SAFE_COLORED:
      pub_traj_safe_colored_->publish(*tmp);
      break;
    
  }
  (*tmp).markers.clear();
}

void FasterRos::publishJPSPath(vec_Vecf<3>& path, int i)
{
  /*vec_Vecf<3> traj, visualization_msgs::MarkerArray* m_array*/
  clearJPSPathVisualization(i);
  switch (i)
  {
    case JPSk_NORMAL:
      vectorOfVectors2MarkerArray(path, &path_jps1_, color(BLUE));
      pub_path_jps1_->publish(path_jps1_);
      break;

    case JPS2_NORMAL:
      vectorOfVectors2MarkerArray(path, &path_jps2_, color(RED));
      pub_path_jps2_->publish(path_jps2_);
      break;
    case JPS_WHOLE:
      vectorOfVectors2MarkerArray(path, &path_jps_whole_, color(GREEN));
      pub_path_jps_whole_->publish(path_jps_whole_);
      break;
    case JPS_SAFE:
      vectorOfVectors2MarkerArray(path, &path_jps_safe_, color(YELLOW));
      pub_path_jps_safe_->publish(path_jps_safe_);
      break;
  }
}

void FasterRos::pubTraj(const std::vector<state>& data, int type)
{
  // Trajectory
  nav_msgs::msg::Path traj;
  traj.poses.clear();
  traj.header.stamp = this->now();
  traj.header.frame_id = world_name_;

  geometry_msgs::msg::PoseStamped temp_path;

  for (int i = 0; i < data.size(); i = i + 8)
  {
    temp_path.pose.position.x = data[i].pos(0);
    temp_path.pose.position.y = data[i].pos(0);
    temp_path.pose.position.z = data[i].pos(0);
    temp_path.pose.orientation.w = 1;
    temp_path.pose.orientation.x = 0;
    temp_path.pose.orientation.y = 0;
    temp_path.pose.orientation.z = 0;
    traj.poses.push_back(temp_path);
  }

  if (type == WHOLE)
  {
    pub_traj_whole_->publish(traj);
  }

  if (type == SAFE)
  {
    pub_traj_safe_->publish(traj);
  }

  clearMarkerColoredTraj();
  clearMarkerArray(std::make_shared<visualization_msgs::msg::MarkerArray>(traj_committed_colored_), COMMITTED_COLORED);
  clearMarkerArray(std::make_shared<visualization_msgs::msg::MarkerArray>(traj_whole_colored_), WHOLE_COLORED);
  clearMarkerArray(std::make_shared<visualization_msgs::msg::MarkerArray>(traj_safe_colored_), SAFE_COLORED);

  if (type == COMMITTED_COLORED)
  {
    traj_committed_colored_ = stateVector2ColoredMarkerArray(data, type, par_.v_max);
    pub_traj_committed_colored_->publish(traj_committed_colored_);
  }

  if (type == WHOLE_COLORED)
  {
    traj_whole_colored_ = stateVector2ColoredMarkerArray(data, type, par_.v_max);
    pub_traj_whole_colored_->publish(traj_whole_colored_);
  }

  if (type == SAFE_COLORED)
  {
    traj_safe_colored_ = stateVector2ColoredMarkerArray(data, type, par_.v_max);
    pub_traj_safe_colored_->publish(traj_safe_colored_);
  }
}

void FasterRos::pubJPSIntersection(Eigen::Vector3d& inters)
{
  geometry_msgs::msg::PointStamped p;
  p.header.frame_id = world_name_;
  p.point = eigen2point(inters);
  pub_jps_inters_->publish(p);
}

void FasterRos::pubActualTraj()
{
  static geometry_msgs::msg::Point p_last = pointOrigin();

  state current_state;
  faster_ptr_->getState(current_state);
  Eigen::Vector3d act_pos = current_state.pos;

  visualization_msgs::msg::Marker m;
  m.type = visualization_msgs::msg::Marker::ARROW;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.id = actual_trajID_ % 3000;  // Start the id again after 300 points published (if not RVIZ goes very slow)
  actual_trajID_++;
  m.color = color(RED);
  m.scale.x = 0.15;
  m.scale.y = 0;
  m.scale.z = 0;
  m.header.stamp = this->now();
  m.header.frame_id = world_name_;

  geometry_msgs::msg::Point p;
  p = eigen2point(act_pos);
  m.points.push_back(p_last);
  m.points.push_back(p);
  pub_actual_traj_->publish(m);
  p_last = p;
}

void FasterRos::clearMarkerActualTraj()
{
  // printf("In clearMarkerActualTraj\n");

  visualization_msgs::msg::Marker m;
  m.type = visualization_msgs::msg::Marker::ARROW;
  m.action = visualization_msgs::msg::Marker::DELETEALL;
  m.id = 0;
  m.scale.x = 0.02;
  m.scale.y = 0.04;
  m.scale.z = 1;
  pub_actual_traj_->publish(m);
  actual_trajID_ = 0;
}

void FasterRos::clearMarkerColoredTraj()
{
  visualization_msgs::msg::Marker m;
  m.type = visualization_msgs::msg::Marker::ARROW;
  m.action = visualization_msgs::msg::Marker::DELETEALL;
  m.id = 0;
  m.scale.x = 1;
  m.scale.y = 1;
  m.scale.z = 1;
  pub_actual_traj_->publish(m);
  // actual_trajID_ = 0;
}

// Occupied CB
void FasterRos::mapCB(const sensor_msgs::msg::PointCloud2::ConstPtr& pcl2ptr_map_ros,
                      const sensor_msgs::msg::PointCloud2::ConstPtr& pcl2ptr_unk_ros)
{
  // Occupied Space Point Cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclptr_map(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*pcl2ptr_map_ros, *pclptr_map);
  // Unknown Space Point Cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclptr_unk(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*pcl2ptr_unk_ros, *pclptr_unk);

  faster_ptr_->updateMap(pclptr_map, pclptr_unk);
}

void FasterRos::pubState(const state& data, int c)
{
  geometry_msgs::msg::PointStamped p;
  p.header.frame_id = world_name_;
  p.point = eigen2point(data.pos);
  switch(c)
  {
    case G_TERM:
      pub_point_G_term_->publish(p);
      break;
    case G_NORMAL:
      pub_point_G_->publish(p);
      break;
  }
}

void FasterRos::terminalGoalCB(const geometry_msgs::msg::PoseStamped& msg)
{
  state G_term;

  double height;
  if (par_.is_ground_robot)
  {
    height = 0.2;
  }
  else if (par_.force_goal_height)
  {
    height = par_.goal_height;
  }
  else
  {
    height = msg.pose.position.z;
  }

  // const double height = (par_.is_ground_robot) ? 0.2 : goal_height_;
  G_term.setPos(msg.pose.position.x, msg.pose.position.y, height);
  faster_ptr_->setTerminalGoal(G_term);

  state G;  // projected goal
  faster_ptr_->getG(G);

  pubState(G_term, G_TERM);
  pubState(G, G_NORMAL);

  clearMarkerActualTraj();
  // std::cout << "Exiting from goalCB\n";
}
