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
#include <algorithm>
#include <fstream>
#include <string>
#include <stdio.h>
#include <time.h>
#include <array>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/Marker.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <message_filters/subscriber.h>

#include <boost/foreach.hpp>
#include <dirent.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/btMatrix3x3.h>

#include <ros/package.h>

#define foreach BOOST_FOREACH

class PoleDataParser{

private:
   ros::NodeHandle nh;

   // for testing offset between VICON and Kinect angle estimation
   //ros::Subscriber VICON_angle_sub;
   //ros::Subscriber Kinect_angle_sub;

   //std::array<std::vector<double>,2> vicon_angle_data;
   //std::array<std::vector<double>,2> kinect_angle_data;

   // for switching between tracker types and pole_angle topics
   std::string vicon_topic;
   std::string kinect_topic;

   std::string rosbag_dir;
   std::string vicon_out_filename;
   std::string kinect_out_filename;

   // files for writing data to
   std::fstream v_file, k_file;
public:

   // set up publishers and subscribers
   PoleDataParser(ros::NodeHandle &node_handle, std::string rosbag_dir, std::string vicon_out_filename, std::string kinect_out_filename){
      vicon_topic = "pole_angle";
      kinect_topic = "pole_angle_kinect";

      this->rosbag_dir = rosbag_dir;
      this->vicon_out_filename = vicon_out_filename;
      this->kinect_out_filename = kinect_out_filename;

      // start ROS and initialize publisher/subscriber
      //nh = node_handle;

      // open data files for writing
      v_file.open(vicon_out_filename, std::fstream::in | std::fstream::out | std::fstream::trunc);
      k_file.open(kinect_out_filename, std::fstream::in | std::fstream::out | std::fstream::trunc);
      if(!v_file || !k_file){
         std::string error = "Could not open file: " + vicon_out_filename + " or " + kinect_out_filename + " for writing\n";
         std::cerr << error;
         std::cerr << "Error: " << strerror(errno) << std::endl;
         return;
      }else{
         v_file << "theta time VICON_theta\n";
         k_file << "theta time Kinect_theta\n";
      }
      //VICON_angle_sub = nh.subscribe<std_msgs::Float64>(vicon_topic, 1000, &PoleDataParser::get_vicon_angle_callback, this);
      //Kinect_angle_sub = nh.subscribe<std_msgs::Float64>(kinect_topic, 1000, &PoleDataParser::get_kinect_angle_callback, this);

   }

   /**
    * Inherits from message_filters::SimpleFilter<M>
    * to use protected signalMessage function
    */
   template <class M>
   class BagSubscriber : public message_filters::SimpleFilter<M>{
      public:
         void newMessage(const boost::shared_ptr<M const> &msg){
             signalMessage(msg);
            }
   };

   void run(){
      DIR *dpdf;
      struct dirent *epdf;
      std::vector<std::string> files;
      dpdf = opendir(rosbag_dir.c_str());
      if (dpdf != NULL){
         while (epdf = readdir(dpdf)){
            std::string fname = epdf->d_name;
            std::cout << fname << std::endl;
            if(fname.compare(".") != 0 && fname.compare("..") != 0){
               if(fname.at(0) == 'm'){
                  fname.replace(0,1,"-");
               }
               files.push_back(fname);
            }
         }
         // sort list of filenames alphabetically
         std::sort(files.begin(), files.end());
         // so dumb but need for sorting
         for(int i = 0; i < files.size(); i+=1){
            if(files.at(i).at(0) == '-'){
               files.at(i).replace(0,1,"m");
            }
         }
      }
      for(int i = 0; i < files.size(); i+=1){
         // get the amount of degrees this file is measuring
         std::stringstream curr_file(files.at(i));
         std::string segment;
         std::vector<std::string> seglist;

         // get just the angle we are measuring
         while(std::getline(curr_file, segment, 'd')){
            seglist.push_back(segment);
         }
         std::string theta = seglist.at(0);
         // if negative angle, then convert 'm' to '-'
         if(theta.at(0) == 'm'){
            theta.replace(0,1,"-");
         }
         std::cout << "reading rosbag " << files.at(i) << " measuring angle " << theta << std::endl;

         rosbag::Bag bag;
         bag.open(rosbag_dir+files.at(i), rosbag::bagmode::Read);

         std::vector<std::string> topics;
         topics.push_back(std::string(kinect_topic));
         topics.push_back(std::string(vicon_topic));

         rosbag::View view(bag, rosbag::TopicQuery(topics));

         // Set up fake subscribers to capture images
         BagSubscriber<std_msgs::Float64> vicon_sub, kinect_sub;

         bool firsttime = true;
         double offset;
         foreach(rosbag::MessageInstance const m, view){

            if (firsttime){
               offset = m.getTime().toSec();
               firsttime = false;
            }

            // get kinect data and write to file
            if (m.getTopic() == kinect_topic || ("/" + m.getTopic() == kinect_topic)){
               std_msgs::Float64::ConstPtr kinect_angle = m.instantiate<std_msgs::Float64>();
               if (kinect_angle != NULL){
                 kinect_sub.newMessage(kinect_angle);
                 ros::Time t_k = m.getTime();
                 k_file << theta << " " << t_k.toSec()-offset << " " << kinect_angle->data << "\n";
              }
             }
             // get VICON data and write to tfile
             if (m.getTopic() == vicon_topic || ("/" + m.getTopic() == vicon_topic)){
               std_msgs::Float64::ConstPtr vicon_angle = m.instantiate<std_msgs::Float64>();
               if (vicon_angle != NULL){
                 vicon_sub.newMessage(vicon_angle);
                 ros::Time t_v = m.getTime();
                 v_file << theta << " " << t_v.toSec()-offset << " " << vicon_angle->data << "\n";
              }
             }
         }

         bag.close();
      }
      closedir(dpdf);
      k_file.close();
      v_file.close();
   }
};

int main(int argc, char** argv) {

   ros::init(argc, argv, "pole_data_parser");
   ros::NodeHandle node_handle;

   if(argc != 4){
      ROS_ERROR("Not enough arguments!");
      ROS_ERROR("Usage:                      pole_data_parser <rosbag_dir> <v_out_filename> <k_out_filename>");
      ros::shutdown();
   }

   std::string rosbag_dir = argv[1];
   std::string v_out_filename = argv[2];
   std::string k_out_filename = argv[3];

   PoleDataParser driver(node_handle, rosbag_dir, v_out_filename, k_out_filename);
   driver.run();
}
