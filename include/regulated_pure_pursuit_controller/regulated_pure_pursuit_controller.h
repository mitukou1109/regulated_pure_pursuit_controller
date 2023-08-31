#pragma once

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <nav_msgs/Odometry.h>

#include <regulated_pure_pursuit_controller/RegulatedPurePursuitControllerConfig.h>

namespace regulated_pure_pursuit_controller
{

  class RegulatedPurePursuitController : public nav_core::BaseLocalPlanner
  {
  public:
    RegulatedPurePursuitController();

    RegulatedPurePursuitController(
        std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros);

    ~RegulatedPurePursuitController() override;

    bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel) override;

    bool isGoalReached() override { return has_reached_goal_; }

    bool setPlan(const std::vector<geometry_msgs::PoseStamped> &plan) override;

    void initialize(std::string name, tf2_ros::Buffer *tf,
                    costmap_2d::Costmap2DROS *costmap_ros) override;

  private:
    double getPathLength(const std::vector<tf2::Transform>::const_iterator &start,
                         const std::vector<tf2::Transform>::const_iterator &end);

    void odomCallback(const nav_msgs::Odometry &odom) { odom_ = odom; }

    void reconfigureCB(RegulatedPurePursuitControllerConfig &config, uint32_t level);

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    std::shared_ptr<costmap_2d::Costmap2DROS> costmap_ros_;

    ros::Publisher local_plan_pub_;

    ros::Subscriber odom_sub_;

    std::shared_ptr<dynamic_reconfigure::Server<RegulatedPurePursuitControllerConfig>>
        reconfigure_server_;

    std::string global_frame_;

    std::vector<tf2::Transform> global_plan_;

    nav_msgs::Odometry odom_;

    double acc_lim_linear_;
    double min_vel_linear_;
    double max_vel_linear_;

    double acc_lim_angular_;
    double min_vel_angular_;
    double max_vel_angular_;

    bool is_omnidirectional_;
    bool follow_viapoint_orientation_;
    bool use_rotate_to_heading_;
    double rotate_to_heading_angular_vel_;

    double xy_goal_tolerance_;
    double yaw_goal_tolerance_;

    bool use_velocity_scaled_lookahead_dist_;
    double fixed_lookahead_dist_;
    double min_lookahead_dist_;
    double max_lookahead_dist_;
    double lookahead_time_;

    bool use_curvature_heuristic_;
    double curvature_gain_;
    double curvature_threshold_;

    bool use_proximity_heuristic_;
    double proximity_gain_;
    double proximity_threshold_;

    double transform_tolerance_;

    double control_period_;

    bool is_initialized_;

    bool has_reached_goal_;

    bool is_rotating_to_goal_orientation_;

    bool should_prerotate_;
  };

} // namespace regulated_pure_pursuit_controller