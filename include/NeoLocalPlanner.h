/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Neobotix GmbH
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Neobotix nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef INCLUDE_NEOLOCALPLANNER_H_
#define INCLUDE_NEOLOCALPLANNER_H_

#include <tf2_ros/buffer.h>
// #include <dynamic_reconfigure/server.h>
#include <angles/angles.h>
#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>

#include "nav2_core/controller.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"


namespace neo_local_planner {

class NeoLocalPlanner : public nav2_core::Controller {

public:
  /**
   * @brief Constructor for nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
   */
  NeoLocalPlanner() = default;

  /**
   * @brief Destrructor for nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
   */
  ~NeoLocalPlanner() override = default;

  /**
   * @brief Configure controller state machine
   * @param parent WeakPtr to node
   * @param name Name of plugin
   * @param tf TF buffer
   * @param costmap_ros Costmap2DROS object of environment
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Cleanup controller state machine
   */
  void cleanup() override;

  /**
   * @brief Activate controller state machine
   */
  void activate() override;

  /**
   * @brief Deactivate controller state machine
   */
  void deactivate() override;

  /**
   * @brief Compute the best command given the current pose and velocity, with possible debug information
   *
   * Same as above computeVelocityCommands, but with debug results.
   * If the results pointer is not null, additional information about the twists
   * evaluated will be in results after the call.
   *
   * @param pose      Current robot pose
   * @param velocity  Current robot velocity
   * @param results   Output param, if not NULL, will be filled in with full evaluation results
   * @return          Best command
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & speed,
    nav2_core::GoalChecker * goal_checker) override;

  /**
   * @brief nav2_core setPlan - Sets the global plan
   * @param path The global plan
   */
  void setPlan(const nav_msgs::msg::Path & path) override;

  /**
   * @brief Limits the maximum linear speed of the robot.
   * @param speed_limit expressed in absolute value (in m/s)
   * or in percentage from maximum robot speed.
   * @param percentage Setting speed limit in percentage if true
   * or in absolute values in false case.
   */
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  /**
   * @brief Callback executed when a parameter change is detected
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

	void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

	bool reset_lastvel(nav_msgs::msg::Path m_global_plan, nav_msgs::msg::Path plan);

private:
	std::shared_ptr<tf2_ros::Buffer> tf_;
	std::string plugin_name_;
	std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
	nav2_costmap_2d::Costmap2D * costmap_;
	rclcpp::Logger logger_ {rclcpp::get_logger("NeoLocalPlanner")};
	tf2_ros::Buffer* m_tf = 0;
	nav2_costmap_2d::Costmap2DROS* m_cost_map;
	nav_msgs::msg::Path m_global_plan;
	rclcpp::Clock::SharedPtr clock_;

	std::mutex m_mutex;
	nav_msgs::msg::Odometry::SharedPtr m_odometry;

	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_sub;
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
	rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> m_local_plan_pub;

	std::string m_global_frame = "map";
	std::string m_local_frame;
	std::string m_base_frame;

	enum state_t {
		STATE_IDLE,
		STATE_TRANSLATING,
		STATE_ROTATING,
		STATE_ADJUSTING,
		STATE_TURNING,
		STATE_STUCK
	};

	state_t m_state = state_t::STATE_IDLE;

	rclcpp::Time m_last_time;
	rclcpp::Time m_first_goal_reached_time;

	bool m_is_goal_reached = false;
	uint64_t m_update_counter = 0;
	double m_last_control_values[3] = {};
	geometry_msgs::msg::Twist m_last_cmd_vel;

protected:
	double acc_lim_x = 0;
	double acc_lim_y = 0;
	double acc_lim_theta = 0;
	double acc_lim_trans = 0;
	double min_vel_x = 0;
	double max_vel_x = 0;
	double min_vel_y = 0;
	double max_vel_y = 0;
	double min_rot_vel = 0;
	double max_rot_vel = 0;
	double min_trans_vel = 0;
	double max_trans_vel = 0;
	double min_vel_theta = 0;
	double max_vel_theta = 0;
	double min_vel_trans = 0;
	double max_vel_trans = 0;
	double rot_stopped_vel = 0; 
	double theta_stopped_vel = 0;
	double trans_stopped_vel = 0;
	double yaw_goal_tolerance = 0;
	double xy_goal_tolerance = 0;
	double goal_tune_time = 0;
	bool differential_drive = false;
	bool constrain_final = false;
	double start_yaw_error = 0.0;
	double lookahead_time = 0.0;
	double m_lookahead_dist = 0.0;
	double pos_x_gain = 0.0;
	double pos_y_gain = 0.0;
	double pos_y_yaw_gain = 0.0;
	double yaw_gain = 0.0;
	double static_yaw_gain = 0.0;
	double cost_x_gain = 0.0;
	double cost_y_gain = 0.0;
	double cost_y_yaw_gain = 0.0;
	double m_cost_y_lookahead_dist = 0.0;
	double cost_y_lookahead_time = 0.0;
	double cost_yaw_gain = 0.0;
	double low_pass_gain = 0.0;
	double max_cost = 0.0;
	double max_curve_vel = 0.0;
	double max_goal_dist = 0.0;
	double max_backup_dist = 0.0;
	double min_stop_dist = 0.0;
	double emergency_acc_lim_x = 0.0;
	bool enable_software_stop = false;
	bool m_reset_lastvel = false;
	bool m_allow_reversing = false;
	double m_robot_direction = 1.0;
	std::string odom_topic = "odom";
	std::string local_plan_topic = "local_plan";
	int count = 0;
	
};


} // neo_local_planner

#endif /* INCLUDE_NEOLOCALPLANNER_H_ */
