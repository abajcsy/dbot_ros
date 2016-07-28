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

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/Marker.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/btMatrix3x3.h>

#include <ros/package.h>
#include <tf/transform_broadcaster.h>

#define PARTICLE_OBJ_TOPIC "/rbc_particle_filter_object_tracker/object_model"
#define GAUSS_OBJ_TOPIC "/rms_gaussian_filter_object_tracker/object_model"

#define KINECT_FRAME "/XTION_RGB"

#define SIGN 1.0
#define PI 3.14159265359

#define ROLL_BASELINE 1.5708
#define PITCH_BASELINE 0.0
#define YAW_BASELINE 0.0
#define CALIB_EPSILON 0.03

class PoleTracker{

private:
   ros::NodeHandle nh;
   ros::Subscriber object_sub;
   ros::Publisher pole_angle_pub;

   // for switching between tracker types and pole_angle topics
   std::string tracker_topic;
   std::string pole_angle_topic;

   // for adjusting for angle offset after calibration
   double pole_angle_offset;

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

      // set the pole angle offset
      pole_angle_offset = atof(angle_offset.c_str());

      // for basic statistics
      num_measurements = 0.0;
      angle_sum = 0.0;

      // start ROS and initialize publisher/subscriber
      start = ros::Time::now();
      nh = node_handle;
      object_sub = nh.subscribe<visualization_msgs::Marker>(tracker_topic, 1000, &PoleTracker::get_angle_callback, this);
      pole_angle_pub = nh.advertise<std_msgs::Float64>(pole_angle_topic, 1000);
   }

   // gets the angle of the pole
   void get_angle_callback(const visualization_msgs::Marker::ConstPtr& msg){
      // sanity check in case message corrupt
      if(!msg){
         return;
      }

      // get position and orientation of tracked object
      geometry_msgs::Point pos = (msg->pose).position;
      geometry_msgs::Quaternion orient = (msg->pose).orientation;

      static tf::TransformBroadcaster br;
      tf::Transform transform;
      transform.setOrigin( tf::Vector3(pos.x, pos.y, pos.z) );

      // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
      tf::Quaternion quat;
      tf::quaternionMsgToTF(orient, quat);

      // TODO THIS IS KIND OF HACKEY
      // need to match the kinect frame orientation with the pole frame
      // so rotate tf quaternion around y and z-axis by 180 degrees to match camera
      tf::Quaternion rotate_180_yz = tf::createQuaternionFromRPY(0,-PI/6,0);
      quat = rotate_180_yz * quat;

      // the tf::Quaternion has a method to acess roll pitch and yaw
      double roll, pitch, yaw;
      tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

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

      // publish transform of pole for RVIZ and reference
      tf::Quaternion q;
      q.setRPY(rpy.x, rpy.y, rpy.z);
      transform.setRotation(q);
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), KINECT_FRAME, tracker_topic));

      pole_angle_pub.publish(final_angle);
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

      // TODO CHECK THIS -- I THINK SOMETHING IS WRONG BECAUSE THE MEASURES ARE TOO LARGE
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

   PoleTracker driver(node_handle, tracker, pole_topic, angle_offset_raw);
   driver.run();
}
