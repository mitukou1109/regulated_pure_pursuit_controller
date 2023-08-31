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
      : is_initialized_(false),
        has_reached_goal_(false),
        is_rotating_to_goal_orientation_(false),
        should_prerotate_(false) {}

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

    tf2::Vector3 current_vel_linear;
    tf2::fromMsg(odom_.twist.twist.linear, current_vel_linear);
    double current_vel_angular = odom_.twist.twist.angular.z;

    double lookahead_dist;
    if (use_velocity_scaled_lookahead_dist_)
    {
      lookahead_dist = std::clamp(current_vel_linear.length() * lookahead_time_,
                                  min_lookahead_dist_, max_lookahead_dist_);
    }
    else
    {
      lookahead_dist = fixed_lookahead_dist_;
    }

    double approach_velocity_scaling_dist =
        (std::pow(max_vel_linear_, 2) - std::pow(min_vel_linear_, 2)) / (2 * acc_lim_linear_);

    std::vector<tf2::Transform> transformed_path;
    auto goal_tf = robot_base_to_global_tf * global_plan_.back();
    auto lookahead_tf = goal_tf;
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

    auto lookahead_point_wrt_moving_direction =
        lookahead_tf.getOrigin().rotate({0, 0, 1}, current_vel_linear.angle({1, 0, 0}));
    double curvature =
        2 * lookahead_point_wrt_moving_direction.y() / std::pow(lookahead_dist, 2);

    double vel_linear;
    if (getPathLength(closest_point, global_plan_.cend()) <= approach_velocity_scaling_dist)
    {
      vel_linear = current_vel_linear.length() - acc_lim_linear_ * control_period_;
      if (goal_tf.getOrigin().length() <= xy_goal_tolerance_)
      {
        if (is_omnidirectional_ || (!is_omnidirectional_ && use_rotate_to_heading_))
        {
          if (std::abs(tf2::getYaw(goal_tf.getRotation())) <= yaw_goal_tolerance_)
          {
            has_reached_goal_ = true;
            return true;
          }
          else
          {
            is_rotating_to_goal_orientation_ = true;
          }
        }
        else
        {
          has_reached_goal_ = true;
          return true;
        }
      }
    }
    else
    {
      vel_linear = current_vel_linear.length() + acc_lim_linear_ * control_period_;

      if (use_curvature_heuristic_ || use_proximity_heuristic_)
      {
        double vel_linear_curvature = vel_linear;
        double vel_linear_proximity = vel_linear;

        if (use_curvature_heuristic_)
        {
          if (std::abs(curvature) >= curvature_threshold_)
          {
            vel_linear_curvature =
                vel_linear * curvature_gain_ / (std::abs(curvature) / curvature_threshold_);
          }
        }
        else
        {
          vel_linear_curvature = 0;
        }

        if (use_proximity_heuristic_)
        {
          using CostmapPoints = std::vector<costmap_2d::MapLocation>;

          auto costmap = costmap_ros_->getCostmap();

          CostmapPoints footprint;
          std::vector<geometry_msgs::Point> raw_footprint;
          costmap_ros_->getOrientedFootprint(raw_footprint);
          for (const auto &raw_point : raw_footprint)
          {
            costmap_2d::MapLocation point;
            if (costmap->worldToMap(raw_point.x, raw_point.y, point.x, point.y))
            {
              footprint.push_back(point);
            }
          }

          CostmapPoints cells_in_footprint;
          costmap->convexFillCells(footprint, cells_in_footprint);

          if (!cells_in_footprint.empty())
          {
            double distance_to_obstacle = INFINITY;
            for (unsigned int mx = 0; mx < costmap->getSizeInCellsX(); mx++)
            {
              for (unsigned int my = 0; my < costmap->getSizeInCellsY(); my++)
              {
                if (std::none_of(cells_in_footprint.cbegin(), cells_in_footprint.cend(),
                                 [&](const costmap_2d::MapLocation &cell)
                                 { return cell.x == mx && cell.y == my; }))
                {
                  if (costmap->getCost(mx, my) >= costmap_2d::LETHAL_OBSTACLE)
                  {
                    auto closest_point_to_obstacle = *std::min_element(
                        cells_in_footprint.cbegin(), cells_in_footprint.cend(),
                        [&](const costmap_2d::MapLocation &a, const costmap_2d::MapLocation &b)
                        { return std::hypot(static_cast<double>(a.x) - mx,
                                            static_cast<double>(a.y) - my) <
                                 std::hypot(static_cast<double>(b.x) - mx,
                                            static_cast<double>(b.y) - my); });

                    distance_to_obstacle =
                        std::min(
                            distance_to_obstacle,
                            std::hypot(static_cast<double>(closest_point_to_obstacle.x) - mx,
                                       static_cast<double>(closest_point_to_obstacle.y) - my) *
                                costmap->getResolution());
                  }
                }
              }
            }

            if (distance_to_obstacle <= proximity_threshold_)
            {
              vel_linear_proximity =
                  vel_linear * proximity_gain_ * distance_to_obstacle / proximity_threshold_;
            }
          }
        }
        else
        {
          vel_linear_proximity = 0;
        }

        vel_linear = std::max(vel_linear_curvature, vel_linear_proximity);
      }
    }

    tf2::Vector3 cmd_vel_linear;
    double cmd_vel_angular;

    nav_msgs::Path local_plan;
    local_plan.header.frame_id = global_frame_;
    local_plan.header.stamp = ros::Time::now();

    if (should_prerotate_)
    {
      auto path_direction_wrt_moving_direction =
          transformed_path.at(1).getOrigin() - transformed_path.at(0).getOrigin();
      double yaw_difference = std::atan2(path_direction_wrt_moving_direction.y(),
                                         path_direction_wrt_moving_direction.x());

      if (std::abs(yaw_difference) <= M_PI / 12)
      {
        should_prerotate_ = false;
      }
      else
      {
        cmd_vel_linear = {0, 0, 0};
        cmd_vel_angular = std::copysign(rotate_to_heading_angular_vel_, yaw_difference);
      }
    }
    else if (is_rotating_to_goal_orientation_)
    {
      cmd_vel_linear = {0, 0, 0};
      cmd_vel_angular = std::copysign(rotate_to_heading_angular_vel_,
                                      tf2::getYaw(goal_tf.getRotation()));
    }
    else if (is_omnidirectional_)
    {
      cmd_vel_linear = std::clamp(vel_linear, min_vel_linear_, max_vel_linear_) *
                       lookahead_tf.getOrigin().normalized();

      double desired_vel_angular;
      if (follow_viapoint_orientation_)
      {
        desired_vel_angular = tf2::getYaw(lookahead_tf.getRotation()) / control_period_;
      }
      else
      {
        desired_vel_angular = tf2::getYaw(goal_tf.getRotation()) *
                              cmd_vel_linear.length() * control_period_ /
                              getPathLength(closest_point, global_plan_.cend());
      }

      double vel_angular_error = desired_vel_angular - current_vel_angular;
      cmd_vel_angular = current_vel_angular +
                        std::copysign(std::min(std::abs(vel_angular_error),
                                               acc_lim_angular_ * control_period_),
                                      vel_angular_error);
      cmd_vel_angular = std::copysign(std::clamp(std::abs(cmd_vel_angular),
                                                 min_vel_angular_, max_vel_angular_),
                                      cmd_vel_angular);

      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = global_frame_;
      pose.header.stamp = ros::Time::now();

      tf2::toMsg(robot_base_to_global_tf.inverse(), pose.pose);
      local_plan.poses.push_back(pose);

      tf2::toMsg(robot_base_to_global_tf.inverse() * lookahead_tf, pose.pose);
      local_plan.poses.push_back(pose);
    }
    else
    {
      cmd_vel_linear.setX(std::clamp(vel_linear, min_vel_linear_, max_vel_linear_));
      cmd_vel_angular = vel_linear * curvature;
      cmd_vel_angular = std::copysign(std::clamp(std::abs(cmd_vel_angular),
                                                 min_vel_angular_, max_vel_angular_),
                                      cmd_vel_angular);

      double arc_radius = std::abs(1 / curvature);
      auto arc_origin =
          robot_base_to_global_tf.inverse() *
          tf2::Transform({{0, 0, 1}, -M_PI_2},
                         transformed_path.front().getOrigin() + tf2::Vector3(0, arc_radius, 0));
      auto arc_end_vector =
          (arc_origin.inverse() * robot_base_to_global_tf.inverse() * lookahead_tf)
              .getOrigin();
      double arc_center_angle = std::atan2(arc_end_vector.y(), arc_end_vector.x());

      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = global_frame_;
      pose.header.stamp = ros::Time::now();

      for (int i = 0; i * M_PI / 12 <= std::abs(arc_center_angle); i++)
      {
        double theta = std::copysign(i * M_PI / 12, arc_center_angle);
        auto arc_pose = arc_origin * tf2::Transform({{0, 0, 1}, theta},
                                                    {arc_radius * std::cos(theta),
                                                     arc_radius * std::sin(theta),
                                                     0});
        tf2::toMsg(arc_pose, pose.pose);
        local_plan.poses.push_back(pose);
      }
      tf2::toMsg(robot_base_to_global_tf.inverse() * lookahead_tf, pose.pose);
      local_plan.poses.push_back(pose);
    }

    cmd_vel.linear = tf2::toMsg(cmd_vel_linear);
    cmd_vel.angular.z = cmd_vel_angular;

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
    is_rotating_to_goal_orientation_ = false;

    auto start_direction = (global_plan_.at(0).inverse() * global_plan_.at(1)).getOrigin();
    double start_yaw = std::atan2(start_direction.y(), start_direction.x());
    should_prerotate_ =
        (!is_omnidirectional_ && use_rotate_to_heading_ &&
         (std::abs(start_yaw - tf2::getYaw(odom_.pose.pose.orientation)) >= M_PI_4));

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
      pnh.param("follow_viapoint_orientation", follow_viapoint_orientation_, true);
      pnh.param("use_rotate_to_heading", use_rotate_to_heading_, true);
      pnh.param("rotate_to_heading_angular_vel", rotate_to_heading_angular_vel_, 0.5);

      pnh.param("xy_goal_tolerance", xy_goal_tolerance_, 0.25);
      pnh.param("yaw_goal_tolerance", yaw_goal_tolerance_, 0.25);

      pnh.param("use_velocity_scaled_lookahead_dist", use_velocity_scaled_lookahead_dist_, true);
      pnh.param("lookahead_dist", fixed_lookahead_dist_, 0.6);
      pnh.param("min_lookahead_dist", min_lookahead_dist_, 0.3);
      pnh.param("max_lookahead_dist", max_lookahead_dist_, 0.9);
      pnh.param("lookahead_time", lookahead_time_, 1.5);

      pnh.param("use_curvature_heuristic", use_curvature_heuristic_, true);
      pnh.param("curvature_gain", curvature_gain_, 1.0);
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

  double RegulatedPurePursuitController::getPathLength(
      const std::vector<tf2::Transform>::const_iterator &start,
      const std::vector<tf2::Transform>::const_iterator &end)
  {
    double length = 0;
    for (auto itr = std::next(start); itr != end; itr++)
    {
      length += std::prev(itr)->getOrigin().distance(itr->getOrigin());
    }
    return length;
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
    follow_viapoint_orientation_ = config.follow_viapoint_orientation;
    use_rotate_to_heading_ = config.use_rotate_to_heading;
    rotate_to_heading_angular_vel_ = config.rotate_to_heading_angular_vel;

    xy_goal_tolerance_ = config.xy_goal_tolerance;
    yaw_goal_tolerance_ = config.yaw_goal_tolerance;

    use_velocity_scaled_lookahead_dist_ = config.use_velocity_scaled_lookahead_dist;
    fixed_lookahead_dist_ = config.lookahead_dist;
    min_lookahead_dist_ = config.min_lookahead_dist;
    max_lookahead_dist_ = config.max_lookahead_dist;
    lookahead_time_ = config.lookahead_time;

    use_curvature_heuristic_ = config.use_curvature_heuristic;
    curvature_gain_ = config.curvature_gain;
    curvature_threshold_ = config.curvature_threshold;

    use_proximity_heuristic_ = config.use_proximity_heuristic;
    proximity_gain_ = config.proximity_gain;
    proximity_threshold_ = config.proximity_threshold;

    transform_tolerance_ = config.transform_tolerance;
  }
}