/*
 * This is part of the Bayesian Object Tracking (bot),
 * (https://github.com/bayesian-object-tracking)
 *
 * Copyright (c) 2015 Max Planck Society,
 * 				 Autonomous Motion Department,
 * 			     Institute for Intelligent Systems
 *
 * This Source Code Form is subject to the terms of the GNU General Public
 * License License (GNU GPL). A copy of the license can be found in the LICENSE
 * file distributed with this source code.
 */

/**
 * \file tracker_node.hpp
 * \date Januray 2016
 * \author Jan Issac (jan.issac@gmail.com)
 */

#pragma once

#include <fl/util/profiling.hpp>

#include <dbot_ros/object_tracker_ros.h>

#include <dbot_ros/util/ros_interface.hpp>

namespace dbot
{
template <typename Tracker>
ObjectTrackerRos<Tracker>::ObjectTrackerRos(
    const std::shared_ptr<Tracker>& tracker,
    const std::shared_ptr<dbot::CameraData>& camera_data,
    int object_count)
    : tracker_(tracker),
      camera_data_(camera_data),
      object_count_(object_count),
      obsrv_updated_(false),
      running_(false)
{
}

template <typename Tracker>
void ObjectTrackerRos<Tracker>::track(const sensor_msgs::Image& ros_image)
{
    auto image = ri::to_eigen_vector<typename Obsrv::Scalar>(
        ros_image, camera_data_->downsampling_factor());

    current_poses_.clear();
    current_state_ = tracker_->track(image);
    geometry_msgs::PoseStamped current_pose;
    int dim = current_state_.size() / object_count_;
    for (int i = 0; i < object_count_; ++i)
    {
        current_pose.pose =
            ri::to_ros_pose(current_state_.middleRows(i * dim, dim));
        current_pose.header.stamp    = ros_image.header.stamp;
        current_pose.header.frame_id = ros_image.header.frame_id;

        current_poses_.push_back(current_pose);
    }
}


template <typename Tracker>
void ObjectTrackerRos<Tracker>::update_obsrv(
    const sensor_msgs::Image& ros_image)
{
    std::lock_guard<std::mutex> lock_obsrv(obsrv_mutex_);
    current_ros_image_ = ros_image;
    obsrv_updated_     = true;
}


template <typename Tracker>
void ObjectTrackerRos<Tracker>::shutdown()
{
    running_ = false;
}

template <typename Tracker>
void ObjectTrackerRos<Tracker>::run()
{
    running_ = true;

    while (ros::ok() && running_)
    {
        if (!obsrv_updated_)
        {
            usleep(100);
            continue;
        }

        run_once();
    }
}

template <typename Tracker>
bool ObjectTrackerRos<Tracker>::run_once()
{
    if (!obsrv_updated_) return false;

    Obsrv obsrv;
    sensor_msgs::Image ros_image;
    {
        std::lock_guard<std::mutex> lock_obsrv(obsrv_mutex_);
        ros_image      = current_ros_image_;
        obsrv_updated_ = false;
    }
    track(ros_image);

    return true;
}

template <typename Tracker>
auto ObjectTrackerRos<Tracker>::current_state() const -> State
{
    return current_state_;
}

template <typename Tracker>
auto ObjectTrackerRos<Tracker>::current_pose() const
    -> geometry_msgs::PoseStamped
{
    return current_poses_[0];
}

template <typename Tracker>
auto ObjectTrackerRos<Tracker>::current_poses() const
    -> std::vector<geometry_msgs::PoseStamped>
{
    return current_poses_;
}
}
