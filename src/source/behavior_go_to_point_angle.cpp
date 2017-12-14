/*!*******************************************************************************************
 *  \file       behavior_go_to_point_angle.cpp
 *  \brief      Behavior Go To Point Angle implementation file.
 *  \details    This file implements the behaviorGoToPointAngle class.
 *  \authors    Jacek Cieślak
 *  \copyright  Copyright 2017 Politechnika Poznańska (PUT) *
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
#include "../include/behavior_go_to_point_angle.h"

behaviorGoToPointAngle::behaviorGoToPointAngle(){

}

behaviorGoToPointAngle::~behaviorGoToPointAngle(){

}

void behaviorGoToPointAngle::ownSetUp(){
  std::cout << "ownSetUp" << std::endl;

  node_handle.param<std::string>("drone_id", drone_id, "1");
  node_handle.param<std::string>("drone_id_namespace", drone_id_namespace, "drone"+drone_id);
  node_handle.param<std::string>("my_stack_directory", my_stack_directory,
                                 "~/workspace/ros/quadrotor_stack_catkin/src/quadrotor_stack");

  node_handle.param<std::string>("estimated_pose_topic", estimated_pose_str, "estimated_pose");
  node_handle.param<std::string>("controllers_topic", controllers_str, "command/high_level");
  node_handle.param<std::string>("rotation_angles_topic", rotation_angles_str, "rotation_angles");
  node_handle.param<std::string>("estimated_speed_topic",estimated_speed_str,"estimated_speed");
  node_handle.param<std::string>("yaw_controller_str",yaw_controller_str , "droneControllerYawRefCommand");
  node_handle.param<std::string>("service_topic_str",service_topic_str , "droneTrajectoryController/setControlMode");
  node_handle.param<std::string>("drone_position_str",drone_position_str , "dronePositionRefs");
  node_handle.param<std::string>("speed_topic",speed_topic , "droneSpeedsRefs");
  node_handle.param<std::string>("drone_control_mode",drone_control_mode_str,"droneTrajectoryController/controlMode");
  node_handle.param<std::string>("d_altitude",d_altitude_str,"command/dAltitude");
  node_handle.param<std::string>("consult_belief",execute_query_srv,"consult_belief");
}

void behaviorGoToPointAngle::ownStart(){
  std::cout << "ownStart" << std::endl;
  //Initialize topics
  estimated_pose_sub = node_handle.subscribe(estimated_pose_str, 1000, &behaviorKeepMoving::estimatedPoseCallBack, this);
  rotation_angles_sub = node_handle.subscribe(rotation_angles_str, 1000, &behaviorKeepMoving::rotationAnglesCallback, this);
  estimated_speed_sub = node_handle.subscribe(estimated_speed_str, 1000, &behaviorKeepMoving::estimatedSpeedCallback, this);
  controllers_pub = node_handle.advertise<droneMsgsROS::droneCommand>(controllers_str, 1, true);
  yaw_controller_pub=node_handle.advertise<droneMsgsROS::droneYawRefCommand>(yaw_controller_str,1000);
  mode_service=node_handle.serviceClient<droneMsgsROS::setControlMode>(service_topic_str);
  drone_position_pub=node_handle.advertise< droneMsgsROS::dronePositionRefCommandStamped>(drone_position_str,1000);
  speed_topic_pub=node_handle.advertise<droneMsgsROS::droneSpeeds>(speed_topic,1000);
  d_altitude_pub = node_handle.advertise<droneMsgsROS::droneDAltitudeCmd>(d_altitude_str,1);
  query_client = node_handle.serviceClient <droneMsgsROS::ConsultBelief> (execute_query_srv);

  //get arguments
  /*
   * ZROBIĆ!!!!
   */

  //behavior implementation
  /*
   * obliczenie wektora prędkości. Jeśli się da, to w X, Y, Z. Jeśli nie, to tylko w X, Y
   * W takim przypadku zrezygnujemy z 3D na rzecz 2D (odchodzi kontrola wysokości)
   * oblicznie zadanego Yaw
   * ?? Rozpoczęcie obrotu? ??
   */

}

void behaviorGoToPointAngle::ownRun(){
  /*
   * kontrola obrotu
   * Po zakończeniu obrotu ruch w osiach 3D (jeśli możliwe na raz).
   * Jeśli na raz nie, to najpierw zmiana wysokości, potem ruch 2D
   * Jeśli się nie da, to tylko ruch 2D
   * kontrola kiedy dotrze do punktu
   */
}

void behaviorGoToPointAngle::ownStop(){
  is_finished = false;
  estimated_pose_sub.shutdown();
  estimated_speed_sub.shutdown();
  rotation_angles_sub.shutdown();
}

//CallBacks
void behaviorGoToPointAngle::estimatedPoseCallBack(const droneMsgsROS::dronePose &){
  estimated_speed_msg = msg;
}

void behaviorGoToPointAngle::estimatedSpeedCallBack(const droneMsgsROS::droneSpeeds &){
  estimated_speed_msg = msg;
}

void behaviorGoToPointAngle::rotationAnglesCallBack(const geometry_msgs::Vector3Stamped &){
  rotation_angles_msg = msg;
}
