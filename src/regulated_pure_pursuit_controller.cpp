#include "regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.h"

#include <numeric>

#include <nav_msgs/Path.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>

PLUGINLIB_EXPORT_CLASS(regulated_pure_pursuit_controller::RegulatedPurePursuitController,
                       nav_core::BaseLocalPlanner)

namespace regulated_pure_pursuit_controller
{
  RegulatedPurePursuitController::RegulatedPurePursuitController()
      : is_initialized_(false), has_reached_goal_(false)
  {
  }

  RegulatedPurePursuitController::RegulatedPurePursuitController(
      std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros)
      : RegulatedPurePursuitController()
  {
    initialize(name, tf, costmap_ros);
  }

  RegulatedPurePursuitController::~RegulatedPurePursuitController() {}

  bool RegulatedPurePursuitController::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
  {
    if (!is_initialized_)
    {
      ROS_ERROR("This planner has not been initialized, "
                "please call initialize() before using this planner");
      return false;
    }

    tf2::Transform robot_base_to_global_tf;
    try
    {
      tf2::fromMsg(tf_buffer_->lookupTransform(costmap_ros_->getBaseFrameID(),
                                               global_frame_,
                                               ros::Time(0),
                                               ros::Duration(transform_tolerance_))
                       .transform,
                   robot_base_to_global_tf);
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
      return false;
    }

    auto closest_point = std::min_element(
        global_plan_.begin(), global_plan_.end(),
        [&](const tf2::Transform &a, const tf2::Transform &b)
        {
          return (robot_base_to_global_tf * a).getOrigin().length() <
                 (robot_base_to_global_tf * b).getOrigin().length();
        });

    double current_vel_linear = std::hypot(odom_.twist.twist.linear.x, odom_.twist.twist.linear.y);
    double current_vel_angular = odom_.twist.twist.angular.z;

    double lookahead_dist;
    if (use_velocity_scaled_lookahead_dist_)
    {
      lookahead_dist = std::clamp(current_vel_linear * lookahead_time_,
                                  min_lookahead_dist_, max_lookahead_dist_);
    }
    else
    {
      lookahead_dist = fixed_lookahead_dist_;
    }

    double approach_velocity_scaling_dist =
        (std::pow(max_vel_linear_, 2) - std::pow(min_vel_linear_, 2)) / (2 * acc_lim_linear_);

    std::vector<tf2::Transform> transformed_path;
    tf2::Transform lookahead_tf;
    for (auto itr = closest_point; itr != global_plan_.end(); itr++)
    {
      const auto &pose = *itr;
      auto transformed_pose = robot_base_to_global_tf * pose;
      transformed_path.push_back(transformed_pose);

      double dist_from_robot_base = transformed_pose.getOrigin().length();
      if (dist_from_robot_base >= lookahead_dist)
      {
        lookahead_tf = transformed_pose;
        break;
      }
    }

    auto goal_error = robot_base_to_global_tf * global_plan_.back();
    double vel_linear = 0;

    if (goal_error.getOrigin().length() <= approach_velocity_scaling_dist)
    {
      if (goal_error.getOrigin().length() <= xy_goal_tolerance_ &&
          std::abs(tf2::getYaw(goal_error.getRotation())) <= yaw_goal_tolerance_)
      {
        has_reached_goal_ = true;
        return true;
      }
      else
      {
        vel_linear = std::max(current_vel_linear - acc_lim_linear_ * control_period_,
                              min_vel_linear_);
      }
    }
    else
    {
      vel_linear = std::min(current_vel_linear + acc_lim_linear_ * control_period_,
                            max_vel_linear_);
    }

    tf2::Vector3 cmd_vel_linear = vel_linear * lookahead_tf.getOrigin().normalized();

    double desired_vel_angular = tf2::getYaw(lookahead_tf.getRotation()) / control_period_;
    double vel_angular_error = desired_vel_angular - current_vel_angular;
    double cmd_vel_angular =
        current_vel_angular + std::copysign(std::min(std::abs(vel_angular_error),
                                                     acc_lim_angular_ * control_period_),
                                            vel_angular_error);
    cmd_vel_angular = std::copysign(std::clamp(std::abs(cmd_vel_angular),
                                               min_vel_angular_, max_vel_angular_),
                                    cmd_vel_angular);

    cmd_vel.linear = tf2::toMsg(cmd_vel_linear);
    cmd_vel.angular.z = cmd_vel_angular;

    nav_msgs::Path local_plan;
    local_plan.header.frame_id = global_frame_;
    local_plan.header.stamp = ros::Time::now();

    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = global_frame_;
    pose.header.stamp = ros::Time::now();
    tf2::toMsg(robot_base_to_global_tf.inverse(), pose.pose);
    local_plan.poses.push_back(pose);

    tf2::toMsg(robot_base_to_global_tf.inverse() * lookahead_tf, pose.pose);
    local_plan.poses.push_back(pose);

    local_plan_pub_.publish(local_plan);

    return true;
  }

  bool RegulatedPurePursuitController::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan)
  {
    if (!is_initialized_)
    {
      ROS_ERROR("This planner has not been initialized, "
                "please call initialize() before using this planner");
      return false;
    }

    global_frame_ = plan.front().header.frame_id;

    global_plan_.clear();
    for (const auto &pose_msg : plan)
    {
      tf2::Transform pose;
      tf2::fromMsg(pose_msg.pose, pose);
      global_plan_.push_back(pose);
    }

    has_reached_goal_ = false;

    return true;
  }

  void RegulatedPurePursuitController::initialize(std::string name, tf2_ros::Buffer *tf,
                                                  costmap_2d::Costmap2DROS *costmap_ros)
  {
    if (!is_initialized_)
    {
      tf_buffer_.reset(tf);
      costmap_ros_.reset(costmap_ros);

      ros::NodeHandle pnh("~/" + name);

      local_plan_pub_ = pnh.advertise<nav_msgs::Path>("local_plan", 1);

      pnh.param("acc_lim_linear", acc_lim_linear_, 2.5);
      pnh.param("min_vel_linear", min_vel_linear_, 0.1);
      pnh.param("max_vel_linear", max_vel_linear_, 0.5);

      pnh.param("acc_lim_angular", acc_lim_angular_, 3.2);
      pnh.param("min_vel_angular", min_vel_angular_, 0.2);
      pnh.param("max_vel_angular", max_vel_angular_, 1.0);

      pnh.param("is_omnidirectional", is_omnidirectional_, true);

      pnh.param("xy_goal_tolerance", xy_goal_tolerance_, 0.25);
      pnh.param("yaw_goal_tolerance", yaw_goal_tolerance_, 0.25);

      pnh.param("use_velocity_scaled_lookahead_dist", use_velocity_scaled_lookahead_dist_, true);
      pnh.param("lookahead_dist", fixed_lookahead_dist_, 0.6);
      pnh.param("min_lookahead_dist", min_lookahead_dist_, 0.3);
      pnh.param("max_lookahead_dist", max_lookahead_dist_, 0.9);
      pnh.param("lookahead_time", lookahead_time_, 1.5);

      pnh.param("use_curvature_heuristic", use_curvature_heuristic_, true);
      pnh.param("curvature_threshold", curvature_threshold_, 1.1);

      pnh.param("use_proximity_heuristic", use_proximity_heuristic_, true);
      pnh.param("proximity_gain", proximity_gain_, 0.5);
      pnh.param("proximity_threshold", proximity_threshold_, 0.5);

      pnh.param("transform_tolerance", transform_tolerance_, 0.1);

      ros::NodeHandle nh;
      odom_sub_ = nh.subscribe("odom", 1, &RegulatedPurePursuitController::odomCallback, this);

      ros::NodeHandle move_base_nh("~");
      double controller_frequency;
      move_base_nh.param("controller_frequency", controller_frequency, 20.0);
      control_period_ = 1 / controller_frequency;

      reconfigure_server_ = std::make_shared<decltype(reconfigure_server_)::element_type>(pnh);
      reconfigure_server_->setCallback(
          boost::bind(&RegulatedPurePursuitController::reconfigureCB, this, _1, _2));

      is_initialized_ = true;
    }
    else
    {
      ROS_WARN("This planner has already been initialized, doing nothing");
    }
  }

  void RegulatedPurePursuitController::reconfigureCB(RegulatedPurePursuitControllerConfig &config,
                                                     uint32_t level)
  {
    acc_lim_linear_ = config.acc_lim_linear;
    min_vel_linear_ = config.min_vel_linear;
    max_vel_linear_ = config.max_vel_linear;

    acc_lim_angular_ = config.acc_lim_angular;
    min_vel_angular_ = config.min_vel_angular;
    max_vel_angular_ = config.max_vel_angular;

    is_omnidirectional_ = config.is_omnidirectional;

    xy_goal_tolerance_ = config.xy_goal_tolerance;
    yaw_goal_tolerance_ = config.yaw_goal_tolerance;

    use_velocity_scaled_lookahead_dist_ = config.use_velocity_scaled_lookahead_dist;
    fixed_lookahead_dist_ = config.lookahead_dist;
    min_lookahead_dist_ = config.min_lookahead_dist;
    max_lookahead_dist_ = config.max_lookahead_dist;
    lookahead_time_ = config.lookahead_time;

    use_curvature_heuristic_ = config.use_curvature_heuristic;
    curvature_threshold_ = config.curvature_threshold;

    use_proximity_heuristic_ = config.use_proximity_heuristic;
    proximity_gain_ = config.proximity_gain;
    proximity_threshold_ = config.proximity_threshold;

    transform_tolerance_ = config.transform_tolerance;
  }
}