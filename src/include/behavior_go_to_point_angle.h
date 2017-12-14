/*!*******************************************************************************************
 *  \file       behavior_go_to_point_angle.h
 *  \brief      behavior go to point angle definition file.
 *  \details    This file contains the behaviorGoToPoint declaration. To obtain more information about
 *              it's definition consult the behavior_go_to_point_angle.cpp file.
 *  \authors    Jacek Cieślak
 *  \copyright  Copyright 2017 Politechnika Poznańska (PUT)
 *
 *     This program is free software: you can redistribute it and/or modify
 *     it under the terms of the GNU General Public License as published by
 *     the Free Software Foundation, either version 3 of the License, or
 *     (at your option) any later version.
 *
 *     This program is distributed in the hope that it will be useful,
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *     GNU General Public License for more details.
 *
 *     You should have received a copy of the GNU General Public License
 *     along with this program. If not, see http://www.gnu.org/licenses/.
 ********************************************************************************/
#ifndef GO_TO_POINT_ANGLE_H
#define GO_TO_POINT_ANGLE_H

#include <string>

//ROS
#include <ros/ros.h>
#include "std_srvs/Empty.h"
#include <droneMsgsROS/droneSpeeds.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <yaml-cpp/yaml.h>
#include <tuple>
#include <droneMsgsROS/dronePositionRefCommandStamped.h>
#include <droneMsgsROS/droneYawRefCommand.h>
#include <droneMsgsROS/droneTrajectoryControllerControlMode.h>
#include <droneMsgsROS/setControlMode.h>

//Aerosstack msgs
#include <droneMsgsROS/BehaviorEvent.h>
#include <droneMsgsROS/droneDAltitudeCmd.h>
#include <droneMsgsROS/dronePose.h>
#include <droneMsgsROS/droneCommand.h>
#include <droneMsgsROS/ConsultBelief.h>

//Aerostack libraries
#include <behavior_process.h>

class behaviorGoToPointAngle: public BehaviorProcess{
public:
  behaviorGoToPointAngle();
  ~behaviorGoToPointAngle();
private:
  ros::NodeHandle node_handle;

  //Config variables
  std::string drone_id;
  std::string drone_id_namespace;
  std::string my_stack_directory;
  std::string behavior_name_str;
  std::string estimated_pose_str;
  std::string rotation_angles_str;
  std::string controllers_str;
  std::string estimated_speed_str;
  std::string yaw_controller_str;
  std::string service_topic_str;
  std::string drone_position_str;
  std::string speed_topic;
  std::string drone_control_mode_str;
  std::string d_altitude_str;
  std::string execute_query_srv;

  //Subscriber
  ros::Subscriber estimated_pose_sub;
  ros::Subscriber estimated_speed_sub;
  ros::Subscriber rotation_angles_sub;
  //Publisher
  ros::Publisher controllers_pub;
  ros::Publisher yaw_controller_pub;
  ros::Publisher drone_position_pub;
  ros::Publisher d_altitude_pub;
  //Service Clients
  ros::ServiceClient mode_service;
  ros::ServiceClient query_client;

  //Message
  droneMsgsROS::dronePose estimated_pose_msg;
  droneMsgsROS::droneSpeeds target_position;
  droneMsgsROS::droneSpeeds estimated_speed_msg;
  geometry_msgs::Vector3Stamped rotation_angles_msg;

  bool is_finished;
  void ownSetUp();
  void ownStart();
  void ownRun();
  void ownStop();

  std::tuple<bool, std::string> ownCheckSituation();

  //CallBacks
  void estimatedPoseCallBack(const droneMsgsROS::dronePose&);
  void estimatedSpeedCallBack(const droneMsgsROS::droneSpeeds&);
  void rotationAnglesCallBack(const geometry_msgs::Vector3Stamped&);
};

#endif
