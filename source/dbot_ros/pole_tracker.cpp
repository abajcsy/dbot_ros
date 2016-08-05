/*
 * This is part of the Kinect Pole Balancing project.
 *
 * Copyright (c) 2016 Max Planck Society,
 * 				 Autonomous Motion Department,
 * 			   Institute for Intelligent Systems
 *
 * This Source Code Form is subject to the terms of the GNU General Public
 * License License (GNU GPL). A copy of the license can be found in the LICENSE
 * file distributed with this source code.
 */

/**
 * \file pole_tracker.cpp
 * \date July 2016
 * \author Andrea Bajcsy (andrea.victoria.b@gmail.com)
 */

#include <vector>
#include <cmath>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <time.h>
#include <array>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/Marker.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/btMatrix3x3.h>

#include <ros/package.h>
#include <tf/transform_broadcaster.h>

#include <dbot_ros_msgs/ObjectState.h>

#define PARTICLE_OBJ_TOPIC "/rbc_particle_filter_object_tracker/object_state"
#define GAUSS_OBJ_TOPIC "/rms_gaussian_filter_object_tracker/object_state"

#define KINECT_FRAME "/XTION_RGB"

#define VICON_ANGLE_TOPIC "pole_angle"
#define POLE_ANGLE_VEL_TOPIC "pole_angle_vel_kinect"

#define SIGN               1.0
#define PI                 3.14159265359

#define ROLL_BASELINE      1.5708
#define PITCH_BASELINE     0.0
#define YAW_BASELINE       0.0
#define CALIB_EPSILON      0.03

// values for adjusting the kinect angle with respect to VICON (gathered empirically)
#define SLOPE              0.703053333333333
#define OFFSET             -0.008968266666667

class PoleTracker{

private:
   ros::NodeHandle nh;
   ros::Subscriber object_sub;
   ros::Publisher pole_angle_pub;
   ros::Publisher pole_angle_vel_pub;

   // for collecting data on the transform of Apollo's kinect
   ros::Subscriber VICON_head_sub;
   double ro, pi, ya;
   double x, y, z;

   // for switching between tracker types and pole_angle topics
   std::string tracker_topic;
   std::string pole_angle_topic;
   std::string pole_angle_vel_topic;

   // for adjusting for angle offset after calibration
   double pole_angle_offset;

   ros::Publisher VICON_angle_pub;
   ros::Publisher Kinect_angle_pub;

   // for keeping basic metrics about angle measurements
   double num_measurements;
   double angle_sum;
   std::vector<btScalar> angle_data;

   ros::Time start;
   ros::Time end;

public:

   // set up publishers and subscribers
   PoleTracker(ros::NodeHandle &node_handle, std::string tracker, std::string pole_topic, std::string angle_offset){
      // save which object tracker we are using
      tracker_topic = PARTICLE_OBJ_TOPIC;
      if(tracker.compare("gaussian") == 0){
         tracker_topic = GAUSS_OBJ_TOPIC;
      }
      // save topic to which we are publishing the pole angle
      pole_angle_topic = pole_topic;
      pole_angle_vel_topic = POLE_ANGLE_VEL_TOPIC;

      // set the pole angle offset
      pole_angle_offset = atof(angle_offset.c_str());

      // for basic statistics
      num_measurements = 0.0;
      angle_sum = 0.0;

      // start ROS and initialize publisher/subscriber
      start = ros::Time::now();
      nh = node_handle;

      object_sub = nh.subscribe<dbot_ros_msgs::ObjectState>(tracker_topic, 1000, &PoleTracker::get_angle_and_vel_callback, this);
      VICON_head_sub = nh.subscribe<geometry_msgs::TransformStamped>("/vicon/apollo_head/apollo_head", 1000, &PoleTracker::get_head_callback, this);

      pole_angle_pub = nh.advertise<std_msgs::Float64>(pole_angle_topic, 1000);
      pole_angle_vel_pub = nh.advertise<std_msgs::Float64>(pole_angle_vel_topic, 1000);
   }

   void get_head_callback(const geometry_msgs::TransformStamped::ConstPtr &msg){
      // sanity check
      if(!msg){
         return;
      }

      geometry_msgs::Transform transform = msg->transform;
      geometry_msgs::Vector3 pos = transform.translation;
      geometry_msgs::Quaternion rot = transform.rotation;

      tf::Quaternion quat;
      tf::quaternionMsgToTF(rot, quat);

      // the tf::Quaternion has a method to acess roll pitch and yaw
      double roll, pitch, yaw;
      tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

      ro = roll; pi = pitch; ya = yaw;
      x = pos.x; y = pos.y; z = pos.z;
   }

   // adjusts the kinect angle to closer match VICON based on a linear regression
   // computed from VICON and Kinect angle measurements of various pole angles
   double adjust_kinect_angle(double kinect_angle){
      return (kinect_angle*SLOPE + OFFSET);
   }

   // gets the angle and angular velocity of the pole
   void get_angle_and_vel_callback(const dbot_ros_msgs::ObjectState::ConstPtr& msg){
      // sanity check in case message corrupt
      if(!msg){
         return;
      }

      geometry_msgs::PoseStamped obj_pose = msg->pose;
      geometry_msgs::PoseStamped obj_vel = msg->velocity;

      // get position and orientation of tracked object
      geometry_msgs::Point pos = obj_pose.pose.position;
      geometry_msgs::Quaternion orient = obj_pose.pose.orientation;

      // get angular velocity of tracked object
      geometry_msgs::Quaternion vel_orient = obj_vel.pose.orientation;

      // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
      tf::Quaternion quat, vel_quat;
      tf::quaternionMsgToTF(orient, quat);
      tf::quaternionMsgToTF(vel_orient, vel_quat);

      // TODO THIS IS KIND OF HACKEY
      // need to match the kinect frame orientation with the pole frame
      // so rotate tf quaternion around y-axis by 180 degrees to match camera
      tf::Quaternion rotate_180_y = tf::createQuaternionFromRPY(0,-PI/6,0);
      quat = rotate_180_y * quat;
      vel_quat = rotate_180_y * vel_quat;

      // the tf::Quaternion has a method to acess roll pitch and yaw
      double roll, pitch, yaw;
      tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

      // get angular velocity for roll, pitch, and yaw angles
      // NOTE: using only yaw angle for published angular velocity
      double vel_roll, vel_pitch, vel_yaw;
      tf::Matrix3x3(vel_quat).getRPY(vel_roll, vel_pitch, vel_yaw);
      double final_angle_vel = vel_yaw;

      geometry_msgs::Vector3 rpy;
      rpy.x = roll;
      rpy.y = pitch;
      rpy.z = yaw;

      // gets yaw angle and axis (for sanity checks)
      btScalar angle = quat.getAngle();
      tf::Vector3 axis = quat.getAxis();

      bool good_roll = (fabs(rpy.x) >= ROLL_BASELINE-CALIB_EPSILON && fabs(rpy.x) <= ROLL_BASELINE+CALIB_EPSILON);
      bool good_pitch = (fabs(rpy.y) >= PITCH_BASELINE-CALIB_EPSILON && fabs(rpy.y) <= PITCH_BASELINE+CALIB_EPSILON);
      bool good_yaw = (fabs(rpy.z) >= YAW_BASELINE-CALIB_EPSILON && fabs(rpy.z) <= YAW_BASELINE+CALIB_EPSILON);

      ROS_INFO("pole_angle_offset: %f", pole_angle_offset);
      ROS_INFO("vel_r: %f, vel_p: %f, vel_y: %f", vel_roll, vel_pitch, vel_yaw);
      ROS_INFO("r: %d, p: %d, y: %d", good_roll, good_pitch, good_yaw);

      // a good calibration means that the plane of camera and plane of pole are paralell
      if(good_roll && good_pitch && good_yaw){
            ROS_INFO("----------------------------------------");
            ROS_INFO("----------- GOOD CALIBRATION -----------");
            ROS_INFO("----------------------------------------");
      }

      // note, we multiply by SIGN in order to get:
      //    - rad = pole is tilted to left (from camera POV)
      //    0 rad = pole is perfectly vertical
      //    + rad = pole is tilted to right

      double final_angle = (SIGN*rpy.z)-pole_angle_offset;
      ROS_INFO("      Raw Kinect angle: %f", final_angle);
      final_angle = adjust_kinect_angle(final_angle);
      ROS_INFO(" Adjusted Kinect angle: %f", final_angle);

      //ROS_INFO("Quaternion Angle (rad): %f", angle);
      ROS_INFO("              Roll (rad): %f", rpy.x);
      ROS_INFO("             Pitch (rad): %f", rpy.y);
      ROS_INFO("        [Sign] Yaw (rad): %f", SIGN*rpy.z);
      ROS_INFO("            (rad-offset): %f", final_angle);
      ROS_INFO("       (deg with offset): %f", final_angle*180/PI);
      //ROS_INFO("       Quaternion Axis: [%f, %f, %f]", axis.getX(), axis.getY(), axis.getZ());
      std::cout << std::endl;


      // collect data on each angle measurement for basic statistics
      angle_data.push_back(final_angle);
      angle_sum += final_angle;
      num_measurements += 1;

      // publish transform of pole coordinate frame for RVIZ visualization
      static tf::TransformBroadcaster br;
      tf::Transform transform;
      tf::Quaternion q;

      transform.setOrigin( tf::Vector3(pos.x, pos.y, pos.z) );
      q.setRPY(rpy.x, rpy.y, rpy.z);
      transform.setRotation(q);
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), KINECT_FRAME, tracker_topic));

      // publish pole angle and angular velocity
      pole_angle_pub.publish(final_angle);
      pole_angle_vel_pub.publish(final_angle_vel);
   }

   void run(){

      while(nh.ok()){
            ros::spin();
      }

      // print basic stats about angle measurements
      end = ros::Time::now();

      // TODO remove this eventually - just a basic sanity check
      if(num_measurements != angle_data.size()){
         num_measurements = angle_data.size();
         std::cout << std::endl;
         std::cout << "INCORRECT num_measurements! Corrected: " << num_measurements << std::endl;
      }

      double angle_mean = angle_sum/num_measurements;
      double sum_dev = 0.0, std_dev = 0.0;

      int i;
      for(i = 0; i < num_measurements; i++){
         sum_dev = (angle_data[i]-angle_mean)*(angle_data[i]-angle_mean);
      }
      std_dev = sqrt(sum_dev/num_measurements);

      std::string tt_str = "Particle";
      if(tracker_topic.compare(GAUSS_OBJ_TOPIC) == 0){
         tt_str = "Gaussian";
      }

      std::cout << std::endl;
      std::cout << "------------ " << tt_str << " EXPERIMENTAL RESULTS -----------" << std::endl;
      std::cout << "             Duration (secs): " << (end-start) << std::endl;
      std::cout << "           Number of samples: " << num_measurements << std::endl;
      std::cout << "        Yaw Angle mean (rad): " << angle_mean << std::endl;
      std::cout << "                       (deg): " << (angle_mean*180/PI) << std::endl;
      std::cout << "     Yaw Angle std_dev (rad): " << std_dev << std::endl;
      std::cout << "                       (deg): " << (std_dev*180/PI) << std::endl;
      std::cout << "------------------------------------------------------" << std::endl;
      std::cout << std::endl;

      std::cout << "-------------- Measured Transform for apollo_head: ---------------" << std::endl;
      std::cout << "x: " << x << ", y: " << y << ", z: " << z << std::endl;
      std::cout << "ro: " << ro << ", pi: " << pi << ", ya: " << ya << std::endl;
      std::cout << "-----------------------------------------------------------------" << std::endl;

      angle_data.clear();
   }
};

int main(int argc, char** argv) {

   ros::init(argc, argv, "pole_tracker");
   ros::NodeHandle node_handle;

   if(argc != 4){
      ROS_ERROR("Not enough arguments!");
      ROS_ERROR("Usage:                      pole_tracker <tracker_type> <pole_angle_topic> <pole_angle_offset>");
      ROS_ERROR("Supported tracker types:    particle or gaussian");
      ros::shutdown();
   }

   std::string tracker = argv[1];
   std::string pole_topic = argv[2];
   std::string angle_offset_raw = argv[3];
   std::cout << "angle_offset_raw: " << angle_offset_raw << std::endl;

   PoleTracker driver(node_handle, tracker, pole_topic, angle_offset_raw);
   driver.run();
}
