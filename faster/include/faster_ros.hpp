/* ----------------------------------------------------------------------------
 * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#pragma once

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <rclcpp/rclcpp.hpp>

#include "visualization_msgs/msg/marker.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

//#include <atomic>

#include <Eigen/Dense>
#include <faster_msgs/msg/goal.hpp>
#include <faster_msgs/msg/state.hpp>
#include <faster_msgs/msg/mode.hpp>
#include <faster_msgs/msg/polyhedron_array.hpp>
#include <nav_msgs/msg/path.hpp>

// TimeSynchronizer includes
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "utils.hpp"

#include "faster.hpp"
#include "faster_types.hpp"

#define WHOLE 1  // Whole trajectory (part of which is planned on unkonwn space)
#define SAFE 2   // Safe path
#define COMMITTED_COLORED 3
#define WHOLE_COLORED 4
#define SAFE_COLORED 5

#define JPSk_NORMAL 6
#define JPS2_NORMAL 7
#define JPS_WHOLE 8
#define JPS_SAFE 9

#define G_TERM 10
#define G_NORMAL 11


//####Class FasterRos
class FasterRos : public rclcpp::Node
{
public:
  FasterRos();
  ~FasterRos();
  bool successful_init() {return init_;};
private:
  std::unique_ptr<Faster> faster_ptr_;

  // class methods
  void pubTraj(const std::vector<state>& data, int type);
  void terminalGoalCB(const geometry_msgs::msg::PoseStamped& msg);
  void pubState(const state& msg,int c);
  void stateCB(const faster_msgs::msg::State::ConstPtr& msg);
  // void odomCB(const nav_msgs::Odometry& odom_ptr);
  void modeCB(const faster_msgs::msg::Mode& msg);
  void pubCB();
  void replanCB();

  visualization_msgs::msg::Marker createMarkerLineStrip(Eigen::MatrixXd X);

  void clearMarkerActualTraj();
  void clearMarkerColoredTraj();
  void mapCB(const sensor_msgs::msg::PointCloud2::ConstPtr& pcl2ptr_msg,
             const sensor_msgs::msg::PointCloud2::ConstPtr& pcl2ptr_msg2);  // Callback for the occupancy pcloud
  void unkCB(const sensor_msgs::msg::PointCloud2::ConstPtr& pcl2ptr_msg);     // Callback for the unkown pcloud
  void pclCB(const sensor_msgs::msg::PointCloud2::ConstPtr& pcl2ptr_msg);
  void frontierCB(const sensor_msgs::msg::PointCloud2::ConstPtr& pcl2ptr_msg);

  void pubActualTraj();
  visualization_msgs::msg::MarkerArray clearArrows();

  void updateInitialCond(int i);
  void yaw(double diff, faster_msgs::msg::Goal& quad_goal);

  void clearMarkerArray(visualization_msgs::msg::MarkerArray::SharedPtr tmp, int c);
  void publishJPSPath(vec_Vecf<3>& path, int i);
  void clearJPSPathVisualization(int i);

  void pubJPSIntersection(Eigen::Vector3d& inters);
  Eigen::Vector3d getFirstCollisionJPS(vec_Vecf<3>& path, bool* thereIsIntersection, int map = MAP,
                                       int type_return = RETURN_LAST_VERTEX);
  Eigen::Vector3d projectClickedGoal(Eigen::Vector3d& P1);

  void publishJPS2handIntersection(vec_Vecf<3> JPS2_fix, Eigen::Vector3d& inter1, Eigen::Vector3d& inter2,
                                   bool solvedFix);

  void createMoreVertexes(vec_Vecf<3>& path, double d);

  bool ARisInFreeSpace(int index);

  int findIndexR(int indexH);
  int findIndexH(bool& needToComputeSafePath);

  void publishPoly(const vec_E<Polyhedron<3>>& poly, int type);

  std::string world_name_ = "world";
  bool param_success_ = true;
  bool init_ = false;

  visualization_msgs::msg::Marker R_;
  visualization_msgs::msg::Marker I_;
  visualization_msgs::msg::Marker E_;
  visualization_msgs::msg::Marker M_;
  visualization_msgs::msg::Marker H_;
  visualization_msgs::msg::Marker A_;
  visualization_msgs::msg::Marker setpoint_;
// Ros stuff

  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_point_G_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_point_G_term_;
  rclcpp::Publisher<faster_msgs::msg::Goal>::SharedPtr pub_goal_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_traj_whole_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_traj_safe_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_setpoint_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_actual_traj_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_path_jps1_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_path_jps2_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_path_jps_safe_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_path_jps_whole_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_intersectionI_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_point_R_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_point_M_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_point_E_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_point_H_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_point_A_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_traj_committed_colored_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_traj_whole_colored_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_traj_safe_colored_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_jps_inters_;
  rclcpp::Publisher<faster_msgs::msg::PolyhedronArray>::SharedPtr poly_whole_pub_;
  rclcpp::Publisher<faster_msgs::msg::PolyhedronArray>::SharedPtr poly_safe_pub_;

  // ros::Publisher cvx_decomp_poly_uo_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
  rclcpp::Subscription<faster_msgs::msg::State>::SharedPtr sub_state_;
  rclcpp::Subscription<faster_msgs::msg::Mode>::SharedPtr sub_mode_;

  // Eigen::Vector3d accel_vicon_;

  rclcpp::TimerBase::SharedPtr pubCBTimer_;
  rclcpp::TimerBase::SharedPtr replanCBTimer_;

  parameters par_;  // where all the parameters are
  // snapstack_msgs::Cvx log_;  // to log all the data
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::string name_drone_;

  visualization_msgs::msg::MarkerArray trajs_sphere_;  // all the trajectories generated in the sphere
  visualization_msgs::msg::MarkerArray path_jps1_;
  visualization_msgs::msg::MarkerArray path_jps2_;
  visualization_msgs::msg::MarkerArray path_jps2_fix_;
  visualization_msgs::msg::MarkerArray path_jps_safe_;
  visualization_msgs::msg::MarkerArray path_jps_whole_;
  visualization_msgs::msg::MarkerArray traj_committed_colored_;
  visualization_msgs::msg::MarkerArray traj_whole_colored_;
  visualization_msgs::msg::MarkerArray traj_safe_colored_;

  visualization_msgs::msg::MarkerArray intersec_points_;
  visualization_msgs::msg::MarkerArray samples_safe_path_;

  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> occup_grid_sub_; //ehemalig PointCloud2
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> unknown_grid_sub_; //ehemalig PointCloud2
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2> //ehemalig PointCloud2
      MySyncPolicy;
  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  std::shared_ptr<Sync> sync_;

  int actual_trajID_ = 0;
  // faster_msgs::Mode mode_;
};
