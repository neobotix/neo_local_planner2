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

#include "../include/NeoLocalPlanner.h"

#include <tf2/utils.h>
#include "nav2_util/node_utils.hpp"
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include "nav2_util/line_iterator.hpp"
#include "nav2_core/goal_checker.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <algorithm>
#include <tf2_eigen/tf2_eigen.hpp>

using rcl_interfaces::msg::ParameterType;
namespace neo_local_planner {

tf2::Quaternion createQuaternionFromYaw(double yaw)
{
	tf2::Quaternion q;
	q.setRPY(0, 0, yaw);
	return q;
}

std::vector<tf2::Transform>::const_iterator find_closest_point(	std::vector<tf2::Transform>::const_iterator begin,
															std::vector<tf2::Transform>::const_iterator end,
															const tf2::Vector3& pos,
															double* actual_dist = 0)
{
	auto iter_short = begin;
	double dist_short = std::numeric_limits<double>::infinity();

	for(auto iter = iter_short; iter != end; ++iter)
	{
		const double dist = (iter->getOrigin() - pos).length();
		if(dist < dist_short)
		{
			dist_short = dist;
			iter_short = iter;
		}
	}
	if(actual_dist) {
		*actual_dist = dist_short;
	}
	return iter_short;
}

std::vector<tf2::Transform>::const_iterator move_along_path(	std::vector<tf2::Transform>::const_iterator begin,
														std::vector<tf2::Transform>::const_iterator end,
														const double dist, double* actual_dist = 0)
{
	auto iter = begin;
	auto iter_prev = iter;
	double dist_left = dist;

	while(iter != end)
	{
		const double dist = (iter->getOrigin() - iter_prev->getOrigin()).length();
		dist_left -= dist;
		if(dist_left <= 0) {
			break;
		}
		iter_prev = iter;
		iter++;
	}
	if(iter == end) {
		iter = iter_prev;		// targeting final pose
	}
	if(actual_dist) {
		*actual_dist = dist - dist_left;
	}
	return iter;
}

std::vector<std::pair <int,int> > get_line_cells(
								nav2_costmap_2d::Costmap2D* cost_map,
								const tf2::Vector3& world_pos_0,
								const tf2::Vector3& world_pos_1)
{
	int coords[2][2] = {};
	cost_map->worldToMapEnforceBounds(world_pos_0.x(), world_pos_0.y(), coords[0][0], coords[0][1]);
	cost_map->worldToMapEnforceBounds(world_pos_1.x(), world_pos_1.y(), coords[1][0], coords[1][1]);

	// Creating a vector for storing the value of the cells
	
	std::vector< std::pair <int,int> > cells;

	// Line iterator for determining the cells between two points
 	for (nav2_util::LineIterator line(coords[0][0], coords[0][1], coords[1][0], coords[1][1]); line.isValid(); line.advance())
 	{
        cells.push_back( std::make_pair(line.getX(),line.getY()) );
    }
    // Mandatory reversal
    // std::reverse( cells.begin(), cells.end() );

	return cells;
}

double get_cost(nav2_costmap_2d::Costmap2D* cost_map_, const tf2::Vector3& world_pos)
{


	int coords[2] = {};
	cost_map_->worldToMapEnforceBounds(world_pos.x(), world_pos.y(), coords[0], coords[1]);

	return cost_map_->getCost(coords[0], coords[1]) / 255.;

}

double compute_avg_line_cost(	nav2_costmap_2d::Costmap2D* cost_map_,
								const tf2::Vector3& world_pos_0,
								const tf2::Vector3& world_pos_1)
{
	const std::vector< std::pair<int, int> > cells = get_line_cells(cost_map_, world_pos_0, world_pos_1);

	double avg_cost = 0;
	for(auto cell : cells) {

		avg_cost += (double)cost_map_->getCost(cell.first, cell.second) / 255.;
	}

	return avg_cost / cells.size();
}

double compute_max_line_cost(	nav2_costmap_2d::Costmap2D* cost_map_,
								const tf2::Vector3& world_pos_0,
								const tf2::Vector3& world_pos_1)
{
	const std::vector< std::pair<int, int> > cells = get_line_cells(cost_map_, world_pos_0, world_pos_1);

	int max_cost = 0;
	for(auto cell : cells) {
		max_cost = std::max(max_cost, int(cost_map_->getCost(cell.first, cell.second)));
	}
	return max_cost / 255.;
}

geometry_msgs::msg::TwistStamped NeoLocalPlanner::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & position,
  const geometry_msgs::msg::Twist & speed,
  nav2_core::GoalChecker * goal_checker)
{
	std::lock_guard<std::mutex> lock_reinit(m_mutex);

	geometry_msgs::msg::Twist cmd_vel;
	geometry_msgs::msg::TwistStamped cmd_vel_final;

	if(m_global_plan.poses.empty())
	{
		RCLCPP_WARN_THROTTLE(logger_, *clock_, 1.0, 
			"Global Plan is empty");
	}

	// compute delta time
	const rclcpp::Time time_now = rclcpp::Clock().now();
	const double dt = fmax(fmin((time_now - m_last_time).seconds(), 0.1), 0);

	// get latest global to local transform (map to odom)
	tf2::Stamped<tf2::Transform> global_to_local;
	try {
		geometry_msgs::msg::TransformStamped msg = tf_->lookupTransform(m_local_frame, m_global_frame, tf2::TimePointZero);
		tf2::fromMsg(msg, global_to_local);
	} catch(...) {
		RCLCPP_WARN_THROTTLE(logger_, *clock_, 1.0, 
			"lookupTransform(m_local_frame, m_global_frame) failed");
	}

	// get latest global to local transform (map to robot)
	tf2::Stamped<tf2::Transform> global_to_robot;
	try {
		geometry_msgs::msg::TransformStamped msg = tf_->lookupTransform(m_base_frame, m_global_frame, tf2::TimePointZero);
		tf2::fromMsg(msg, global_to_robot);
	} catch(...) {
		RCLCPP_WARN_THROTTLE(logger_, *clock_, 1.0, 
			"lookupTransform(m_base_frame, m_global_frame) failed");
	}

	// transform plan to local frame (odom)
	std::vector<tf2::Transform> local_plan;
	std::vector<tf2::Transform> transformed_plan;

	for(const auto& pose : m_global_plan.poses)
	{
		tf2::Transform pose_;
		tf2::Transform robot_pose_;
		tf2::fromMsg(pose.pose, pose_);
		tf2::fromMsg(pose.pose, robot_pose_);
		local_plan.push_back(global_to_local * pose_);
		transformed_plan.push_back(global_to_robot * robot_pose_);
	}

	if(m_allow_reversing and count<=1) {
		auto point_1 = tf2::toMsg(transformed_plan[5]);

		// Estimate if the robot has travelled and then determine if the path is reversed! ToDo
		// Just checks if the goal is in the rear end of the robot
		auto reverse_path = point_1.translation.x;

		m_robot_direction = reverse_path >= 0.0 ? 1.0 : -1.0;
		count++;

		if(m_robot_direction == -1.0 && acc_lim_x > 0.0) {
			acc_lim_x = -1.0 * acc_lim_x;
		} 

		if(m_robot_direction == 1.0 && acc_lim_x < 0.0) {
			acc_lim_x = -1.0 * acc_lim_x;
		} 
	}

	// get latest local pose
	tf2::Transform local_pose;
	tf2::fromMsg(position.pose, local_pose);

	const double start_yaw = tf2::getYaw(local_pose.getRotation());
	const double start_vel_x = speed.linear.x;
	const double start_vel_y = speed.linear.y;
	const double start_yawrate = speed.angular.z;

	double lookahead_dist = 0.0;
	double cost_y_lookahead_dist = 0.0;

	// calc dynamic lookahead distances
	lookahead_dist = m_lookahead_dist + fmax(fabs(start_vel_x), 0) * lookahead_time;
	cost_y_lookahead_dist = m_cost_y_lookahead_dist + fmax(start_vel_x, 0) * cost_y_lookahead_time;

	// predict future pose (using second order midpoint method)
	tf2::Vector3 actual_pos;
	double actual_yaw = 0;
	{
		const double midpoint_yaw = start_yaw + start_yawrate * lookahead_time / 2;
		actual_pos = local_pose.getOrigin() + tf2::Matrix3x3(createQuaternionFromYaw(midpoint_yaw))
												* tf2::Vector3(start_vel_x, start_vel_y, 0) * lookahead_time;
		actual_yaw = start_yaw + start_yawrate * lookahead_time;
	}

	const tf2::Transform actual_pose = tf2::Transform(createQuaternionFromYaw(actual_yaw), actual_pos);

	// compute cost gradients
	const double delta_x = 0.3;
	const double delta_y = 0.2;
	const double delta_yaw = 0.1;

	const double center_cost = get_cost(costmap_, actual_pos);
	const double delta_cost_x = (
		compute_avg_line_cost(costmap_, actual_pos, actual_pose * tf2::Vector3(delta_x, 0, 0)) -
		compute_avg_line_cost(costmap_, actual_pos, actual_pose * tf2::Vector3(-delta_x, 0, 0)))
		/ delta_x;

	const double delta_cost_y = (
		compute_avg_line_cost(costmap_, actual_pos, actual_pose * tf2::Vector3(cost_y_lookahead_dist, delta_y, 0)) -
		compute_avg_line_cost(costmap_, actual_pos, actual_pose * tf2::Vector3(cost_y_lookahead_dist, -delta_y, 0)))
		/ delta_y;

	const double delta_cost_yaw = (
		(
			compute_avg_line_cost(costmap_,	actual_pose * (tf2::Matrix3x3(createQuaternionFromYaw(delta_yaw)) * tf2::Vector3(delta_x, 0, 0)),
												actual_pose * (tf2::Matrix3x3(createQuaternionFromYaw(delta_yaw)) * tf2::Vector3(-delta_x, 0, 0)))
		) - (
			compute_avg_line_cost(costmap_,	actual_pose * (tf2::Matrix3x3(createQuaternionFromYaw(-delta_yaw)) * tf2::Vector3(delta_x, 0, 0)),
												actual_pose * (tf2::Matrix3x3(createQuaternionFromYaw(-delta_yaw)) * tf2::Vector3(-delta_x, 0, 0)))
		)) / (2 * delta_yaw);

	// fill local plan later
	nav_msgs::msg::Path local_path;
	local_path.header.frame_id = m_local_frame;
	local_path.header.stamp = m_odometry->header.stamp;

	// compute obstacle distance
	bool have_obstacle = false;
	double obstacle_dist = 0;
	double obstacle_cost = 0;
	{
		const double delta_move = 0.05;
		const double delta_time = fabs(start_vel_x) > trans_stopped_vel ? (delta_move / fabs(start_vel_x)) : 0;

		tf2::Transform pose = actual_pose;
		tf2::Transform last_pose = pose;

		while(obstacle_dist < 10)
		{
			const double cost = compute_max_line_cost(costmap_, last_pose.getOrigin(), pose.getOrigin());

			bool is_contained = false;
			{
				unsigned int dummy[2] = {};
				is_contained = costmap_->worldToMap(pose.getOrigin().x(), pose.getOrigin().y(), dummy[0], dummy[1]);
			}
			have_obstacle = cost >= max_cost;
			obstacle_cost = fmax(obstacle_cost, cost);

			{
				geometry_msgs::msg::PoseStamped tmp;
				auto tmp1 = tf2::toMsg(pose);
				tmp.header = position.header;
				tmp.pose.position.x = tmp1.translation.x;
				tmp.pose.position.y = tmp1.translation.y;
				tmp.pose.position.z = tmp1.translation.z;
				tmp.pose.orientation.x = tmp1.rotation.x;
				tmp.pose.orientation.y = tmp1.rotation.y;
				tmp.pose.orientation.z = tmp1.rotation.z;
				tmp.pose.orientation.w = tmp1.rotation.w;
				local_path.poses.push_back(tmp);
			}
			if(!is_contained || have_obstacle) {
				break;
			}

			last_pose = pose;
			if(!m_allow_reversing) {
				pose = tf2::Transform(createQuaternionFromYaw(tf2::getYaw(pose.getRotation()) + start_yawrate * delta_time),
						pose * tf2::Vector3(delta_move, 0, 0));	
			}	else {
				pose = tf2::Transform(createQuaternionFromYaw(tf2::getYaw(pose.getRotation()) + start_yawrate * delta_time),
						pose * tf2::Vector3(m_robot_direction * delta_move, 0, 0));	
			}

			obstacle_dist += delta_move;
		}
	}
	m_local_plan_pub->publish(local_path);

	obstacle_dist -= min_stop_dist;

	// publish local plan

	// compute situational max velocities
	const double max_trans_vel = fmax(max_vel_trans * (max_cost - center_cost) / max_cost, min_vel_trans);
	const double max_rot_vel = fmax(max_vel_theta * (max_cost - center_cost) / max_cost, min_vel_theta);

	// find closest point on path to future position
	auto iter_target = find_closest_point(local_plan.cbegin(), local_plan.cend(), actual_pos);

	// check if goal target
	bool is_goal_target = false;
	{
		// check if goal is within reach
		auto iter_next = move_along_path(iter_target, local_plan.cend(), max_goal_dist);
		is_goal_target = iter_next + 1 >= local_plan.cend();

		if(is_goal_target)
		{
			// go straight to goal
			iter_target = iter_next;
		}
	}
	// figure out target orientation
	double target_yaw = 0;
	if(is_goal_target)
	{
		// take goal orientation
		target_yaw = tf2::getYaw(iter_target->getRotation());
	}
	else
	{
		// compute path based target orientation
		auto iter_next = move_along_path(iter_target, local_plan.cend(), lookahead_dist);
		target_yaw = ::atan2(	iter_next->getOrigin().y() - iter_target->getOrigin().y(),
								iter_next->getOrigin().x() - iter_target->getOrigin().x());
	}

	// get target position
	const tf2::Vector3 target_pos = iter_target->getOrigin();
	double yaw_error = 0.0;

	if(m_robot_direction==1 or is_goal_target) {
		yaw_error = angles::shortest_angular_distance(actual_yaw, target_yaw);
	} else {
		yaw_error = angles::shortest_angular_distance(actual_yaw + 3.14, target_yaw); 
	}

	// Condition to check for a spotaneous change in the goal
	if (m_reset_lastvel && speed.linear.x != 0.0 && 
		speed.linear.y != 0.0 && speed.angular.z != 0.0 &&
		fabs(yaw_error) > M_PI / 6) {
		cmd_vel_final.twist = m_last_cmd_vel;
		m_reset_lastvel = false;
		return cmd_vel_final;
	}

	// compute errors
	const double goal_dist = (local_plan.back().getOrigin() - actual_pos).length();
	const tf2::Vector3 pos_error = tf2::Transform(createQuaternionFromYaw(actual_yaw), actual_pos).inverse() * target_pos;

	// compute control values
	bool is_emergency_brake = false;
	double control_vel_x = 0;
	double control_vel_y = 0;
	double control_yawrate = 0;

	if(is_goal_target)
	{
		// use term for final stopping position
		control_vel_x = pos_error.x() * pos_x_gain;
	}
	else
	{
		control_vel_x = m_robot_direction * max_trans_vel;

		// wait to start moving
		if(m_state != state_t::STATE_TRANSLATING && fabs(yaw_error) > start_yaw_error)
		{
			control_vel_x = 0;
		}

		// limit curve velocity
		{
			const double max_vel_x = max_curve_vel * (lookahead_dist / fabs(yaw_error));
			if(m_robot_direction == -1.0) {
				control_vel_x = m_robot_direction * fmin(fabs(control_vel_x), max_vel_x);	
			} else {
				control_vel_x = fmin(control_vel_x, max_vel_x);
			}
		}

		// limit velocity when approaching goal position
		if(fabs(start_vel_x) > 0)	{
			const double stop_accel = 0.8 * acc_lim_x;
			const double stop_time = sqrt(2 * fmax(fabs(goal_dist), 0) / fabs(stop_accel));

			if(m_robot_direction == -1.0) {
				const double max_vel_x = m_robot_direction * fmax(fabs(stop_accel) * stop_time, min_vel_trans);
				control_vel_x = m_robot_direction * fmin(fabs(control_vel_x), fabs(max_vel_x));	
			} else {
				const double max_vel_x = fmax(stop_accel * stop_time, min_vel_trans);
				control_vel_x = fmin(control_vel_x, max_vel_x);
				
			}
		}

		// limit velocity when approaching an obstacle
		if(have_obstacle && fabs(start_vel_x) > 0)
		{
			const double stop_accel = 0.9 * acc_lim_x;
			const double stop_time = sqrt(2 * fmax(obstacle_dist, 0) / stop_accel);
			const double max_vel_x = stop_accel * stop_time;

			// check if it's much lower than current velocity
			if(fabs(max_vel_x) < 0.5 * fabs(start_vel_x)) {
				is_emergency_brake = true;
			}

			if(m_robot_direction == -1.0) {
				control_vel_x = m_robot_direction * fmin(fabs(control_vel_x), max_vel_x);	
			} else {
				control_vel_x = fmin(control_vel_x, max_vel_x);
			}
		}

		// stop before hitting obstacle
		if(have_obstacle && obstacle_dist <= 0)
		{
			control_vel_x = 0;
		}

		// only allow forward velocity depending on the parameter setting
		if(!m_allow_reversing) {
			control_vel_x = fmax(control_vel_x, 0);
		}
		
	}
	// limit backing up
	if(is_goal_target	 && max_backup_dist > 0
		&& fabs(pos_error.x()) < (m_state == state_t::STATE_TURNING ? 0 : -1 * max_backup_dist))
	{
		control_vel_x = 0;
		m_state = state_t::STATE_TURNING;
	}
	else if(m_state == state_t::STATE_TURNING)
	{
		m_state = state_t::STATE_IDLE;
	}
	if(differential_drive)
	{
		if(fabs(start_vel_x) > (m_state == state_t::STATE_TRANSLATING ?
								trans_stopped_vel : 2 * trans_stopped_vel))
		{
			// we are translating, use term for lane keeping
			control_yawrate = pos_error.y() / start_vel_x * pos_y_yaw_gain;

			if(!is_goal_target)
			{
				// additional term for lane keeping
				control_yawrate += yaw_error * yaw_gain;

				// add cost terms
				control_yawrate -= delta_cost_y / start_vel_x * cost_y_yaw_gain;
				control_yawrate -= delta_cost_yaw * cost_yaw_gain;
			}

			m_state = state_t::STATE_TRANSLATING;
		}
		else if(m_state == state_t::STATE_TURNING)
		{
			// continue on current yawrate
			control_yawrate = (start_yawrate > 0 ? 1 : -1) * max_rot_vel;
		}
		else if(is_goal_target
				&& (m_state == state_t::STATE_ADJUSTING || fabs(yaw_error) < M_PI / 6)
				&& fabs(pos_error.y()) > (m_state == state_t::STATE_ADJUSTING ?
					0.25 * xy_goal_tolerance : 0.5 * xy_goal_tolerance))
		{
			// we are not translating, but we have too large y error
			control_yawrate = (pos_error.y() > 0 ? 1 : -1) * max_rot_vel;

			m_state = state_t::STATE_ADJUSTING;
		}
		else
		{
			// use term for static target orientation
			control_yawrate = yaw_error * static_yaw_gain;

			m_state = state_t::STATE_ROTATING;
		}
	}
	else
	{
		// simply correct y with holonomic drive
		control_vel_y = pos_error.y() * pos_y_gain;

		if(m_state == state_t::STATE_TURNING)
		{
			// continue on current yawrate
			control_yawrate = (start_yawrate > 0 ? 1 : -1) * max_rot_vel;
		}
		else
		{
			// use term for static target orientation
			control_yawrate = yaw_error * static_yaw_gain;

			if(fabs(start_vel_x) > trans_stopped_vel) {
				m_state = state_t::STATE_TRANSLATING;
			} else {
				m_state = state_t::STATE_ROTATING;
			}
		}

		// apply x cost term only when rotating
		if(m_state == state_t::STATE_ROTATING && fabs(yaw_error) > M_PI / 6)
		{
			control_vel_x -= delta_cost_x * cost_x_gain;
		}

		// apply y cost term when not approaching goal or if we are rotating
		if(!is_goal_target || (m_state == state_t::STATE_ROTATING && fabs(yaw_error) > M_PI / 6))
		{
			control_vel_y -= delta_cost_y * cost_y_gain;
		}

		// apply yaw cost term when not approaching goal
		if(!is_goal_target)
		{
			control_yawrate -= delta_cost_yaw * cost_yaw_gain;
		}
	}
	// check if we are stuck
	if(have_obstacle && obstacle_dist <= 0 && delta_cost_x > 0
		&& m_state == state_t::STATE_ROTATING && fabs(yaw_error) < M_PI / 6)
	{
		// we are stuck
		m_state = state_t::STATE_STUCK;

		RCLCPP_WARN_THROTTLE(logger_, *clock_, 1.0, 
			"We are stuck: yaw_error=%f, obstacle_dist=%f, obstacle_cost=%f, delta_cost_x=%f",
						yaw_error, obstacle_dist, obstacle_cost, delta_cost_x);

		geometry_msgs::msg::TwistStamped cmd_vel_stuck;
  	cmd_vel_stuck.header.stamp = clock_->now();
		cmd_vel_stuck.header.frame_id = position.header.frame_id;
  	cmd_vel_stuck.twist.linear.x = 0;
  	cmd_vel_stuck.twist.angular.z = 0;

  return cmd_vel_stuck;
	}

	// logic check
	is_emergency_brake = is_emergency_brake && fabs(control_vel_x) >= 0;

	// apply low pass filter

	control_vel_x = control_vel_x * low_pass_gain + m_last_control_values[0] * (1 - low_pass_gain);
	control_vel_y = control_vel_y * low_pass_gain + m_last_control_values[1] * (1 - low_pass_gain);
	control_yawrate = control_yawrate * low_pass_gain + m_last_control_values[2] * (1 - low_pass_gain);

	// apply acceleration limits

	if(m_robot_direction == -1.0) {
		if(!is_goal_target) {
			control_vel_x = fmin(fabs(control_vel_x), fabs(m_last_cmd_vel.linear.x + acc_lim_x * dt));
			control_vel_x = m_robot_direction * fmax(fabs(control_vel_x), fabs(m_last_cmd_vel.linear.x - 
			(is_emergency_brake ?  m_robot_direction * emergency_acc_lim_x : acc_lim_x) * dt));
		}
	} else {
		control_vel_x = fmax(fmin(control_vel_x, m_last_cmd_vel.linear.x + acc_lim_x * dt),
							m_last_cmd_vel.linear.x - (is_emergency_brake ? emergency_acc_lim_x : acc_lim_x) * dt);
	}

	// Calculate vel_y
	control_vel_y = fmin(control_vel_y, m_last_cmd_vel.linear.y + acc_lim_y * dt);
	control_vel_y = fmax(control_vel_y, m_last_cmd_vel.linear.y - acc_lim_y * dt);

	// Calculate vel_yaw
	control_yawrate = fmin(control_yawrate, m_last_cmd_vel.angular.z + acc_lim_theta * dt);
	control_yawrate = fmax(control_yawrate, m_last_cmd_vel.angular.z - acc_lim_theta * dt);

	// fill return data
	if (m_robot_direction == -1.0) {
		cmd_vel.linear.x = fmax(fmin(control_vel_x, m_robot_direction * min_vel_x),
		 m_robot_direction * max_vel_x);
	}
	else {
		cmd_vel.linear.x = fmin(fmax(control_vel_x, min_vel_x),
		 max_vel_x);
	}

	cmd_vel.linear.y = fmin(fmax(control_vel_y, min_vel_y), max_vel_y);
	cmd_vel.linear.z = 0;
	cmd_vel.angular.x = 0;
	cmd_vel.angular.y = 0;
	cmd_vel.angular.z = fmin(fmax(control_yawrate, -max_vel_theta), max_vel_theta);

	m_last_time = time_now;
	m_last_control_values[0] = control_vel_x;
	m_last_control_values[1] = control_vel_y;
	m_last_control_values[2] = control_yawrate;
	m_last_cmd_vel = cmd_vel;

	m_update_counter++;
	cmd_vel_final.header.stamp = clock_->now();
	cmd_vel_final.header.frame_id = position.header.frame_id;
	cmd_vel_final.twist.linear = cmd_vel.linear;
	cmd_vel_final.twist.angular = cmd_vel.angular;

  return cmd_vel_final;
}

void NeoLocalPlanner::cleanup()
{
	m_local_plan_pub.reset();
}

void NeoLocalPlanner::activate()
{
	m_local_plan_pub->on_activate();
  // Add callback for dynamic parameters
  auto node = node_.lock();
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &NeoLocalPlanner::dynamicParametersCallback,
      this, std::placeholders::_1));
}

void NeoLocalPlanner::deactivate()
{
	m_local_plan_pub->on_deactivate();
	dyn_params_handler_.reset();
}

rcl_interfaces::msg::SetParametersResult
NeoLocalPlanner::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
	std::lock_guard<std::mutex> lock_reinit(m_mutex);
	rcl_interfaces::msg::SetParametersResult result;

  for (auto parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();

    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (param_name == plugin_name_ + ".acc_lim_x") {
        acc_lim_x = parameter.as_double();
      } else if (param_name == plugin_name_ + ".acc_lim_y") {
        acc_lim_y = parameter.as_double();
      } else if (param_name == plugin_name_ + ".acc_lim_theta") {
        acc_lim_theta = parameter.as_double();
      } else if (param_name == plugin_name_ + ".min_vel_x") {
        min_vel_x = parameter.as_double();
      } else if (param_name == plugin_name_ + ".max_vel_x") {
        max_vel_x = parameter.as_double();
      } else if (param_name == plugin_name_ + ".min_vel_y") {
        min_vel_y = parameter.as_double();
      } else if (param_name == plugin_name_ + ".max_vel_y") {
        max_vel_y = parameter.as_double();
      } else if (param_name == plugin_name_ + ".min_rot_vel") {
        min_vel_theta = parameter.as_double();
      } else if (param_name == plugin_name_ + ".max_rot_vel") {
        max_vel_theta = parameter.as_double();
      } else if (param_name == plugin_name_ + ".min_vel_trans") {
        min_vel_trans = parameter.as_double();
      } else if (param_name == plugin_name_ + ".max_vel_trans") {
        max_vel_trans = parameter.as_double();
      } else if (param_name == plugin_name_ + ".rot_stopped_vel") {
        theta_stopped_vel = parameter.as_double();
      } else if (param_name == plugin_name_ + ".trans_stopped_vel") {
        trans_stopped_vel = parameter.as_double();
      } else if (param_name == plugin_name_ + ".yaw_goal_tolerance") {
        yaw_goal_tolerance = parameter.as_double();
      } else if (param_name == plugin_name_ + ".xy_goal_tolerance") {
        xy_goal_tolerance = parameter.as_double();
      } else if (param_name == plugin_name_ + ".goal_tune_time") {
        goal_tune_time = parameter.as_double();
      } else if (param_name == plugin_name_ + ".lookahead_time") {
        lookahead_time = parameter.as_double();
      } else if (param_name == plugin_name_ + ".lookahead_dist") {
        m_lookahead_dist = parameter.as_double();
      } else if (param_name == plugin_name_ + ".start_yaw_error") {
        start_yaw_error = parameter.as_double();
      } else if (param_name == plugin_name_ + ".pos_x_gain") {
        pos_x_gain = parameter.as_double();
      } else if (param_name == plugin_name_ + ".pos_y_gain") {
        pos_y_gain = parameter.as_double();
      } else if (param_name == plugin_name_ + ".pos_y_yaw_gain") {
        pos_y_yaw_gain = parameter.as_double();
      } else if (param_name == plugin_name_ + ".yaw_gain") {
        yaw_gain = parameter.as_double();
      } else if (param_name == plugin_name_ + ".static_yaw_gain") {
        static_yaw_gain = parameter.as_double();
      } else if (param_name == plugin_name_ + ".cost_x_gain") {
        cost_x_gain = parameter.as_double();
      } else if (param_name == plugin_name_ + ".cost_y_gain") {
        cost_y_gain = parameter.as_double();
      } else if (param_name == plugin_name_ + ".cost_y_yaw_gain") {
        cost_y_yaw_gain = parameter.as_double();
      } else if (param_name == plugin_name_ + ".cost_y_lookahead_time") {
        cost_y_lookahead_time = parameter.as_double();
      } else if (param_name == plugin_name_ + ".cost_yaw_gain") {
        cost_yaw_gain = parameter.as_double();
      } else if (param_name == plugin_name_ + ".low_pass_gain") {
        low_pass_gain = parameter.as_double();
      } else if (param_name == plugin_name_ + ".max_cost") {
        max_cost = parameter.as_double();
      } else if (param_name == plugin_name_ + ".max_curve_vel") {
        max_curve_vel = parameter.as_double();
      } else if (param_name == plugin_name_ + ".max_goal_dist") {
        max_goal_dist = parameter.as_double();
      } else if (param_name == plugin_name_ + ".max_backup_dist") {
        max_backup_dist = parameter.as_double();
      } else if (param_name == plugin_name_ + ".min_stop_dist") {
        min_stop_dist = parameter.as_double();
      } else if (param_name == plugin_name_ + ".emergency_acc_lim_x") {
        emergency_acc_lim_x = parameter.as_double(); 
      }
    }
  }
  result.successful = true;
  return result;
}

bool NeoLocalPlanner::reset_lastvel(nav_msgs::msg::Path m_global_plan, nav_msgs::msg::Path plan)
{

	if(m_global_plan.poses.empty() || plan.poses.back().pose == m_global_plan.poses.back().pose)
	{
		return false;
	}
	else 
	{
		m_last_control_values[0] = 0;
		m_last_control_values[1] = 0;
		m_last_control_values[2] = 0;
		m_last_cmd_vel = geometry_msgs::msg::Twist();
		count = 0;
		return true;
	}
	
}

void NeoLocalPlanner::setPlan(const nav_msgs::msg::Path & plan)
{
	m_reset_lastvel = reset_lastvel(m_global_plan, plan);
	m_global_plan = plan;
}

void NeoLocalPlanner::setSpeedLimit(
  const double & speed_limit,
  const bool & percentage)
{
}

void NeoLocalPlanner::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,  std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
	auto node = parent.lock();
	node_ = parent;
	plugin_name_ = name;
	clock_ = node->get_clock();
	nav2_util::declare_parameter_if_not_declared(node,plugin_name_ + ".acc_lim_x",rclcpp::ParameterValue(0.2));
	nav2_util::declare_parameter_if_not_declared(node,plugin_name_ + ".acc_lim_y",rclcpp::ParameterValue(0.2));
	nav2_util::declare_parameter_if_not_declared(node,plugin_name_ + ".acc_lim_theta",rclcpp::ParameterValue(0.2));
	nav2_util::declare_parameter_if_not_declared(node,plugin_name_ + ".min_vel_x",rclcpp::ParameterValue(0.2));
	nav2_util::declare_parameter_if_not_declared(node,plugin_name_ + ".max_vel_x",rclcpp::ParameterValue(0.2));
	nav2_util::declare_parameter_if_not_declared(node,plugin_name_ + ".min_vel_y",rclcpp::ParameterValue(0.2));
	nav2_util::declare_parameter_if_not_declared(node,plugin_name_ + ".max_vel_y",rclcpp::ParameterValue(0.2));
	nav2_util::declare_parameter_if_not_declared(node,plugin_name_ + ".min_rot_vel",rclcpp::ParameterValue(0.2));
	nav2_util::declare_parameter_if_not_declared(node,plugin_name_ + ".max_rot_vel",rclcpp::ParameterValue(0.2));
	nav2_util::declare_parameter_if_not_declared(node,plugin_name_ + ".min_vel_trans",rclcpp::ParameterValue(0.2));
	nav2_util::declare_parameter_if_not_declared(node,plugin_name_ + ".max_vel_trans",rclcpp::ParameterValue(0.2));
	nav2_util::declare_parameter_if_not_declared(node,plugin_name_ + ".rot_stopped_vel", rclcpp::ParameterValue(0.2));
	nav2_util::declare_parameter_if_not_declared(node,plugin_name_ + ".trans_stopped_vel",rclcpp::ParameterValue(0.2));
	nav2_util::declare_parameter_if_not_declared(node,plugin_name_ + ".yaw_goal_tolerance",rclcpp::ParameterValue(0.2));
	nav2_util::declare_parameter_if_not_declared(node,plugin_name_ + ".xy_goal_tolerance",rclcpp::ParameterValue(0.2));

	nav2_util::declare_parameter_if_not_declared(node,plugin_name_ + ".goal_tune_time",rclcpp::ParameterValue(0.2));
	nav2_util::declare_parameter_if_not_declared(node,plugin_name_ + ".lookahead_time",rclcpp::ParameterValue(0.2));
	nav2_util::declare_parameter_if_not_declared(node,plugin_name_ + ".lookahead_dist",rclcpp::ParameterValue(0.2));
	nav2_util::declare_parameter_if_not_declared(node,plugin_name_ + ".start_yaw_error",rclcpp::ParameterValue(0.2));
	nav2_util::declare_parameter_if_not_declared(node,plugin_name_ + ".pos_x_gain",rclcpp::ParameterValue(0.2));
	nav2_util::declare_parameter_if_not_declared(node,plugin_name_ + ".pos_y_gain",rclcpp::ParameterValue(0.2));
	nav2_util::declare_parameter_if_not_declared(node,plugin_name_ + ".pos_y_yaw_gain",rclcpp::ParameterValue(0.2));
	nav2_util::declare_parameter_if_not_declared(node,plugin_name_ + ".yaw_gain",rclcpp::ParameterValue(0.2));
	nav2_util::declare_parameter_if_not_declared(node,plugin_name_ + ".static_yaw_gain",rclcpp::ParameterValue(0.2));
	nav2_util::declare_parameter_if_not_declared(node,plugin_name_ + ".cost_x_gain",rclcpp::ParameterValue(0.2));
	nav2_util::declare_parameter_if_not_declared(node,plugin_name_ + ".cost_y_gain",rclcpp::ParameterValue(0.2));
	nav2_util::declare_parameter_if_not_declared(node,plugin_name_ + ".cost_y_yaw_gain",rclcpp::ParameterValue(0.2));
	nav2_util::declare_parameter_if_not_declared(node,plugin_name_ + ".cost_y_lookahead_dist", rclcpp::ParameterValue(0.2));
	nav2_util::declare_parameter_if_not_declared(node,plugin_name_ + ".cost_y_lookahead_time",rclcpp::ParameterValue(0.2));
	nav2_util::declare_parameter_if_not_declared(node,plugin_name_ + ".cost_yaw_gain",rclcpp::ParameterValue(0.2));
	nav2_util::declare_parameter_if_not_declared(node,plugin_name_ + ".low_pass_gain",rclcpp::ParameterValue(0.2));

	nav2_util::declare_parameter_if_not_declared(node,plugin_name_ + ".max_cost",rclcpp::ParameterValue(0.2));
	nav2_util::declare_parameter_if_not_declared(node,plugin_name_ + ".max_curve_vel",rclcpp::ParameterValue(0.2));
	nav2_util::declare_parameter_if_not_declared(node,plugin_name_ + ".max_goal_dist",rclcpp::ParameterValue(0.2));
	nav2_util::declare_parameter_if_not_declared(node,plugin_name_ + ".max_backup_dist", rclcpp::ParameterValue(0.2));
	nav2_util::declare_parameter_if_not_declared(node,plugin_name_ + ".min_stop_dist",rclcpp::ParameterValue(0.2));
	nav2_util::declare_parameter_if_not_declared(node,plugin_name_ + ".emergency_acc_lim_x",rclcpp::ParameterValue(0.2));
	nav2_util::declare_parameter_if_not_declared(node,plugin_name_ + ".differential_drive", rclcpp::ParameterValue(true));
	nav2_util::declare_parameter_if_not_declared(node,plugin_name_ + ".constrain_final", rclcpp::ParameterValue(false));
	nav2_util::declare_parameter_if_not_declared(node,plugin_name_ + ".allow_reversing", rclcpp::ParameterValue(false));
	nav2_util::declare_parameter_if_not_declared(node,plugin_name_ + ".odom_topic", rclcpp::ParameterValue("/odom"));
	nav2_util::declare_parameter_if_not_declared(node,plugin_name_ + ".local_plan_topic", rclcpp::ParameterValue("/local_plan"));
	nav2_util::declare_parameter_if_not_declared(node,plugin_name_ + ".local_frame", rclcpp::ParameterValue("odom"));
	nav2_util::declare_parameter_if_not_declared(node,plugin_name_ + ".base_frame", rclcpp::ParameterValue("base_link"));

	node->get_parameter_or(plugin_name_ + ".acc_lim_x", acc_lim_x, 0.5);
	node->get_parameter_or(plugin_name_ + ".acc_lim_y", acc_lim_y, 0.5);
	node->get_parameter_or(plugin_name_ + ".acc_lim_theta", acc_lim_theta, 0.5);
	node->get_parameter_or(plugin_name_ + ".min_vel_x", min_vel_x, -0.1);
	node->get_parameter_or(plugin_name_ + ".max_vel_x", max_vel_x, 0.5);
	node->get_parameter_or(plugin_name_ + ".min_vel_y", min_vel_y, -0.5);
	node->get_parameter_or(plugin_name_ + ".max_vel_y", max_vel_y, 0.5);
	node->get_parameter_or(plugin_name_ + ".min_rot_vel", min_vel_theta, 0.1);
	node->get_parameter_or(plugin_name_ + ".max_rot_vel", max_vel_theta, 0.5);
	node->get_parameter_or(plugin_name_ + ".min_trans_vel", min_vel_trans, 0.1);
	node->get_parameter_or(plugin_name_ + ".max_trans_vel", max_vel_trans, 0.5);
	node->get_parameter_or(plugin_name_ + ".rot_stopped_vel", theta_stopped_vel, 0.05);
	node->get_parameter_or(plugin_name_ + ".trans_stopped_vel", trans_stopped_vel, 0.05);
	node->get_parameter_or(plugin_name_ + ".yaw_goal_tolerance", yaw_goal_tolerance, 0.02);
	node->get_parameter_or(plugin_name_ + ".xy_goal_tolerance", xy_goal_tolerance, 0.1);

	node->get_parameter_or(plugin_name_ + ".goal_tune_time", goal_tune_time, 0.5);
	node->get_parameter_or(plugin_name_ + ".lookahead_time", lookahead_time, 0.5);
	node->get_parameter_or(plugin_name_ + ".lookahead_dist", m_lookahead_dist, 0.5);
	node->get_parameter_or(plugin_name_ + ".start_yaw_error", start_yaw_error, 0.2);
	node->get_parameter_or(plugin_name_ + ".pos_x_gain", pos_x_gain, 1.0);
	node->get_parameter_or(plugin_name_ + ".pos_y_gain", pos_y_gain, 1.0);
	node->get_parameter_or(plugin_name_ + ".pos_y_yaw_gain", pos_y_yaw_gain, 1.0);
	node->get_parameter_or(plugin_name_ + ".yaw_gain", yaw_gain, 1.0);
	node->get_parameter_or(plugin_name_ + ".static_yaw_gain", static_yaw_gain, 3.0);
	node->get_parameter_or(plugin_name_ + ".cost_x_gain", cost_x_gain, 0.1);
	node->get_parameter_or(plugin_name_ + ".cost_y_gain", cost_y_gain, 0.1);
	node->get_parameter_or(plugin_name_ + ".cost_y_yaw_gain", cost_y_yaw_gain, 0.1);
	node->get_parameter_or(plugin_name_ + ".cost_y_lookahead_dist", m_cost_y_lookahead_dist, 0.0);
	node->get_parameter_or(plugin_name_ + ".cost_y_lookahead_time", cost_y_lookahead_time, 1.0);
	node->get_parameter_or(plugin_name_ + ".cost_yaw_gain", cost_yaw_gain, 1.0);
	node->get_parameter_or(plugin_name_ + ".low_pass_gain", low_pass_gain, 0.5);

	node->get_parameter_or(plugin_name_ + ".max_cost", max_cost, 0.9);
	node->get_parameter_or(plugin_name_ + ".max_curve_vel", max_curve_vel, 0.2);
	node->get_parameter_or(plugin_name_ + ".max_goal_dist", max_goal_dist, 0.5);
	node->get_parameter_or(plugin_name_ + ".max_backup_dist", max_backup_dist, 0.5);
	node->get_parameter_or(plugin_name_ + ".min_stop_dist", min_stop_dist, 0.5);
	node->get_parameter_or(plugin_name_ + ".emergency_acc_lim_x", emergency_acc_lim_x, 0.5);
	node->get_parameter_or(plugin_name_ + ".differential_drive", differential_drive, true);
	node->get_parameter_or(plugin_name_ + ".allow_reversing", m_allow_reversing, false);
	node->get_parameter_or(plugin_name_ + ".constrain_final", constrain_final, false);

	node->get_parameter(plugin_name_ + ".odom_topic", odom_topic);
	node->get_parameter(plugin_name_ + ".local_plan_topic", local_plan_topic);
	node->get_parameter(plugin_name_ + ".local_frame", m_local_frame);
	node->get_parameter(plugin_name_ + ".base_frame", m_base_frame);

	// Variable manipulation
	max_vel_trans = max_vel_x;
	trans_stopped_vel = 0.5 * min_vel_trans;

	// Setting up the costmap variables
	costmap_ros_ = costmap_ros;
	costmap_ = costmap_ros_->getCostmap();
	tf_ = tf;
	plugin_name_ = name;
	logger_ = node->get_logger();

	std::string robot_namespace(node->get_namespace());

	// removing the unnecessary "/" from the namespace
	robot_namespace.erase(std::remove(robot_namespace.begin(), robot_namespace.end(), '/'), 
	robot_namespace.end());

	m_local_frame = robot_namespace + m_local_frame;
	m_base_frame = robot_namespace + m_base_frame;

	// Creating odometery subscriber and local plan publisher
	m_odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(odom_topic,  rclcpp::SystemDefaultsQoS(), std::bind(&NeoLocalPlanner::odomCallback,this,std::placeholders::_1));
	m_local_plan_pub = node->create_publisher<nav_msgs::msg::Path>(local_plan_topic, 1);

}

void NeoLocalPlanner::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
	std::lock_guard<std::mutex> lock_reinit(m_mutex);
	m_odometry = msg;
}

}

// neo_local_planner

PLUGINLIB_EXPORT_CLASS(neo_local_planner::NeoLocalPlanner, nav2_core::Controller)
