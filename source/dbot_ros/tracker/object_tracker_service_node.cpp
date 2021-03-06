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
 * \file rbc_particle_filter_object_ros_object_tracker.cpp
 * \date November 2015
 * \author Jan Issac (jan.issac@gmail.com)
 */

#include <Eigen/Dense>

#include <fstream>
#include <ctime>
#include <memory>
#include <thread>

#include <ros/ros.h>
#include <ros/package.h>

#include <fl/util/profiling.hpp>

#include <opi/interactive_marker_initializer.hpp>
#include <osr/free_floating_rigid_bodies_state.hpp>

#include <dbot/camera_data.hpp>
#include <dbot/simple_wavefront_object_loader.hpp>
#include <dbot/tracker/rbc_particle_filter_object_tracker.hpp>
#include <dbot/builder/rbc_particle_filter_tracker_builder.hpp>

#include <dbot_ros/object_tracker_ros.h>
#include <dbot_ros/object_tracker_publisher.h>
#include <dbot_ros/util/ros_interface.hpp>
#include <dbot_ros/util/ros_camera_data_provider.hpp>

#include <dbot_ros_msgs/RunObjectTracker.h>

static bool running = false;
static std::thread tracker_thread;

void run(dbot::ObjectResourceIdentifier ori, osr::PoseVelocityVector pose)
{
    ros::NodeHandle nh("~");
    /* ------------------------------ */
    /* - Create the object model    - */
    /* ------------------------------ */

    // Use the ORI to load the object model usign the
    // SimpleWavefrontObjectLoader
    auto object_model_loader = std::shared_ptr<dbot::ObjectModelLoader>(
        new dbot::SimpleWavefrontObjectModelLoader(ori));

    // Load the model usign the simple wavefront load and center the
    // frames
    // of all object part meshes
    auto object_model =
        std::make_shared<dbot::ObjectModel>(object_model_loader, true);

    /* ------------------------------ */
    /* - Setup camera data          - */
    /* ------------------------------ */
    int downsampling_factor;
    std::string camera_info_topic;
    std::string depth_image_topic;
    dbot::CameraData::Resolution resolution;
    nh.getParam("camera_info_topic", camera_info_topic);
    nh.getParam("depth_image_topic", depth_image_topic);
    nh.getParam("downsampling_factor", downsampling_factor);
    nh.getParam("resolution/width", resolution.width);
    nh.getParam("resolution/height", resolution.height);

    auto camera_data_provider = std::shared_ptr<dbot::CameraDataProvider>(
        new dbot::RosCameraDataProvider(nh,
                                        camera_info_topic,
                                        depth_image_topic,
                                        resolution,
                                        downsampling_factor,
                                        60.0));
    // Create camera data from the RosCameraDataProvider which takes the
    // data
    // from a ros camera topic
    auto camera_data = std::make_shared<dbot::CameraData>(camera_data_provider);

    /* ------------------------------ */
    /* - Few types we will be using - */
    /* ------------------------------ */
    typedef osr::FreeFloatingRigidBodiesState<> State;
    typedef dbot::RbcParticleFilterObjectTracker Tracker;
    typedef dbot::RbcParticleFilterTrackerBuilder<Tracker> TrackerBuilder;
    typedef TrackerBuilder::StateTransitionBuilder StateTransitionBuilder;
    typedef TrackerBuilder::ObservationModelBuilder ObservationModelBuilder;

    // parameter shorthand prefix
    std::string pre = "particle_filter/";

    /* ------------------------------ */
    /* - State transition function  - */
    /* ------------------------------ */
    // We will use a linear observation model built by the object transition
    // model builder. The linear model will generate a random walk.
    dbot::ObjectTransitionModelBuilder<State>::Parameters params_state;
    nh.getParam(pre + "object_transition/linear_sigma",
                params_state.linear_sigma);
    nh.getParam(pre + "object_transition/angular_sigma",
                params_state.angular_sigma);
    nh.getParam(pre + "object_transition/velocity_factor",
                params_state.velocity_factor);
    params_state.part_count = 1;

    auto state_trans_builder = std::shared_ptr<StateTransitionBuilder>(
        new dbot::ObjectTransitionModelBuilder<State>(params_state));

    /* ------------------------------ */
    /* - Observation model          - */
    /* ------------------------------ */
    dbot::RbObservationModelBuilder<State>::Parameters params_obsrv;
    nh.getParam(pre + "use_gpu", params_obsrv.use_gpu);

    if (params_obsrv.use_gpu)
    {
        nh.getParam(pre + "gpu/sample_count", params_obsrv.sample_count);
    }
    else
    {
        nh.getParam(pre + "cpu/sample_count", params_obsrv.sample_count);
    }

    nh.getParam(pre + "observation/occlusion/p_occluded_visible",
                params_obsrv.occlusion.p_occluded_visible);
    nh.getParam(pre + "observation/occlusion/p_occluded_occluded",
                params_obsrv.occlusion.p_occluded_occluded);
    nh.getParam(pre + "observation/occlusion/initial_occlusion_prob",
                params_obsrv.occlusion.initial_occlusion_prob);

    nh.getParam(pre + "observation/kinect/tail_weight",
                params_obsrv.kinect.tail_weight);
    nh.getParam(pre + "observation/kinect/model_sigma",
                params_obsrv.kinect.model_sigma);
    nh.getParam(pre + "observation/kinect/sigma_factor",
                params_obsrv.kinect.sigma_factor);
    params_obsrv.delta_time = 1. / 30.;

    // gpu only parameters
    nh.getParam(pre + "gpu/use_custom_shaders",
                params_obsrv.use_custom_shaders);
    nh.getParam(pre + "gpu/vertex_shader_file",
                params_obsrv.vertex_shader_file);
    nh.getParam(pre + "gpu/fragment_shader_file",
                params_obsrv.fragment_shader_file);
    nh.getParam(pre + "gpu/geometry_shader_file",
                params_obsrv.geometry_shader_file);

    auto obsrv_model_builder = std::shared_ptr<ObservationModelBuilder>(
        new dbot::RbObservationModelBuilder<State>(
            object_model, camera_data, params_obsrv));

    /* ------------------------------ */
    /* - Create Filter & Tracker    - */
    /* ------------------------------ */
    TrackerBuilder::Parameters params_tracker;
    params_tracker.evaluation_count = params_obsrv.sample_count;
    nh.getParam(pre + "moving_average_update_rate",
                params_tracker.moving_average_update_rate);
    nh.getParam(pre + "max_kl_divergence", params_tracker.max_kl_divergence);

    auto tracker_builder = dbot::RbcParticleFilterTrackerBuilder<Tracker>(
        state_trans_builder, obsrv_model_builder, object_model, params_tracker);
    auto tracker = tracker_builder.build();

    dbot::ObjectTrackerRos<Tracker> ros_object_tracker(
        tracker, camera_data, ori.count_meshes());

    ros_object_tracker.tracker()->initialize({pose});

    /* ------------------------------ */
    /* - Tracker publisher          - */
    /* ------------------------------ */
    int object_color[3];
    nh.getParam(pre + "object_color/R", object_color[0]);
    nh.getParam(pre + "object_color/G", object_color[1]);
    nh.getParam(pre + "object_color/B", object_color[2]);
    auto tracker_publisher = dbot::ObjectStatePublisher(
        ori, object_color[0], object_color[1], object_color[2]);

    /* ------------------------------ */
    /* - Run the tracker            - */
    /* ------------------------------ */
    ros::Subscriber subscriber =
        nh.subscribe(depth_image_topic,
                     1,
                     &dbot::ObjectTrackerRos<Tracker>::update_obsrv,
                     &ros_object_tracker);
    (void)subscriber;

    ROS_INFO_STREAM("Tracking object " << ori.mesh_without_extension(0));
    auto pause_duration = ros::Duration(0.001);
    while (ros::ok() && running)
    {
        pause_duration.sleep();
        if (ros_object_tracker.run_once())
        {
            tracker_publisher.publish(ros_object_tracker.current_pose());
        }
    }
    ROS_INFO("Tracking terminated.");
}

bool run_object_tracker_srv(dbot_ros_msgs::RunObjectTrackerRequest& req,
                            dbot_ros_msgs::RunObjectTrackerResponse& res)
{
    if (running)
    {
        ROS_INFO("Preempting current tracker ...");
        running = false;
        if (tracker_thread.joinable()) tracker_thread.join();
    }

    ROS_INFO("Setup new object to track");
    running = true;

    tracker_thread = std::thread(
        [          =]()
        {
            try
            {
                run(dbot::ObjectResourceIdentifier(
                        ros::package::getPath(req.object_state.ori.package),
                        req.object_state.ori.directory,
                        {req.object_state.ori.name}),
                    ri::to_pose_velocity_vector(req.object_state.pose.pose));
            }
            catch (std::exception& e)
            {
                ROS_ERROR("%s", e.what());
            }
        });

    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_tracker_service");
    ros::NodeHandle nh;
    ros::NodeHandle nh_prv("~");

    std::string service_name;
    nh_prv.getParam("object_tracker_service_name", service_name);

    auto srv = nh.advertiseService(service_name, run_object_tracker_srv);

    ROS_INFO("Object tracker service up and running.");
    ROS_INFO("Waiting for tracking requests...");

    ros::spin();

    return 0;
}
