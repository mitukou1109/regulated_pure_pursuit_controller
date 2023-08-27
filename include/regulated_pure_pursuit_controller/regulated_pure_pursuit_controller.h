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

    bool isGoalReached() override { return is_goal_reached_; }

    bool setPlan(const std::vector<geometry_msgs::PoseStamped> &plan) override;

    void initialize(std::string name, tf2_ros::Buffer *tf,
                    costmap_2d::Costmap2DROS *costmap_ros) override;

  private:
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

    double xy_goal_tolerance_;
    double yaw_goal_tolerance_;

    bool use_velocity_scaled_lookahead_dist_;
    double fixed_lookahead_dist_;
    double min_lookahead_dist_;
    double max_lookahead_dist_;
    double lookahead_time_;

    bool use_curvature_heuristic_;
    double curvature_threshold_;

    bool use_proximity_heuristic_;
    double proximity_gain_;
    double proximity_threshold_;

    double transform_tolerance_;

    double controller_frequency_;

    bool is_initialized_;

    bool is_goal_reached_;
  };

} // namespace regulated_pure_pursuit_controller