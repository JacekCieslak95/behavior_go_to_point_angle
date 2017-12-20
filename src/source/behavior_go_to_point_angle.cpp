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

BehaviorGoToPointAngle::BehaviorGoToPointAngle(){

}

BehaviorGoToPointAngle::~BehaviorGoToPointAngle(){

}

void BehaviorGoToPointAngle::ownSetUp(){
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
  node_handle.param<std::string>("behavior_rotate_start",rotation_start_srv,"behavior_rotate/start");
  node_handle.param<std::string>("behavior_rotate_stop",rotation_stop_srv,"behavior_rotate/stop");
}

void BehaviorGoToPointAngle::ownStart(){
  std::cout << "ownStart" << std::endl;
  //Initialize topics
  estimated_pose_sub = node_handle.subscribe(estimated_pose_str, 1000, &BehaviorGoToPointAngle::estimatedPoseCallBack, this);
  rotation_angles_sub = node_handle.subscribe(rotation_angles_str, 1000, &BehaviorGoToPointAngle::rotationAnglesCallback, this);
estimated_speed_sub = node_handle.subscribe(estimated_speed_str, 1000, &BehaviorGoToPointAngle::estimatedSpeedCallback, this);
  controllers_pub = node_handle.advertise<droneMsgsROS::droneCommand>(controllers_str, 1, true);
  yaw_controller_pub=node_handle.advertise<droneMsgsROS::droneYawRefCommand>(yaw_controller_str,1000);
  mode_service=node_handle.serviceClient<droneMsgsROS::setControlMode>(service_topic_str);
  drone_position_pub=node_handle.advertise< droneMsgsROS::dronePositionRefCommandStamped>(drone_position_str,1000);
  //speed_topic_pub=node_handle.advertise<droneMsgsROS::droneSpeeds>(speed_topic,1000);
  d_altitude_pub = node_handle.advertise<droneMsgsROS::droneDAltitudeCmd>(d_altitude_str,1);
  query_client = node_handle.serviceClient <droneMsgsROS::ConsultBelief> (execute_query_srv);
  rotation_start_client = node_handle.serviceClient<droneMsgsROS::StartBehavior>(rotation_start_srv);
  rotation_stop_client = node_handle.serviceClient<droneMsgsROS::StartBehavior>(rotation_stop_srv);

  //get arguments
  std::string arguments=getArguments();
  YAML::Node config_file = YAML::Load(arguments);
    //get target position
  if(config_file["coordinates"].IsDefined()){
    std::vector<double> points = config_file["coordinates"].as<std::vector<double>>();
    target_position.x = points[0];
    target_position.y = points[1];
    target_position.z = points[2];
  }
  else if(config_file["relative_coordinates"].IsDefined()){
    std::vector<double> points = config_file["relative_coordinates"].as<std::vector<double>>();
    target_position.x = points[0] + estimated_pose_msg.x + 0.0001;
    target_position.y = points[1] + estimated_pose_msg.y;
    target_position.z = points[2] + estimated_pose_msg.z;
  }
  else{
    setStarted(false);
    return;
  }
    //get speed
  if(config_file["speed"].IsDefined()){
    speed=config_file["speed"].as<float>();
  }
  else{
    speed = 5;
    std::cout<<"Could not read speed. Default speed="<<speed<<std::endl;
  }
    //get angle
  if(config_file["angle"].IsDefined()){
    angle=config_file["angle"].as<float>();
  }
  else{
    angle=0;
    std::cout<<"Could not read angle. Default angle="<<angle<<std::endl;
  }

  //calculate speeds and angle
  double distance = sqrt(pow(target_position.x-estimated_pose_msg.x,2)
                         + pow(target_position.y-estimated_pose_msg.y,2)
                         + pow(target_position.z-estimated_pose_msg.z,2));
  setpoint_speed_msg.dx = speed * (target_position.x - estimated_pose_msg.x) / distance;
  setpoint_speed_msg.dy = speed * (target_position.y - estimated_pose_msg.y) / distance;
  setpoint_speed_msg.dz = speed * (target_position.z - estimated_pose_msg.z) / distance;

  target_position.yaw = angle + atan2(setpoint_speed_msg.dy, setpoint_speed_msg.dx);
  std::cout << "calculated speed in axes:" << std::endl
            << "\tx: " << setpoint_speed_msg.dx
            << "\ty: " << setpoint_speed_msg.dy
            << "\tz: " << setpoint_speed_msg.dz << std::endl;

  //behavior implementation
  droneMsgsROS::StartBehavior startRotationMessage;
  std::ostringstream capturador;
  capturador<<"ANGLE="<<target_position.yaw;
  std::string startRotationArguments(capturador.str());

  rotation_start_client.call(startRotationMessage);
  /*
   * obliczenie wektora prędkości. Jeśli się da, to w X, Y, Z. Jeśli nie, to tylko w X, Y
   * W takim przypadku zrezygnujemy z 3D na rzecz 2D (odchodzi kontrola wysokości)
   * oblicznie zadanego Yaw
   * ?? Rozpoczęcie obrotu? ??
   */

}

void BehaviorGoToPointAngle::ownRun(){
  /*
   * kontrola obrotu
   * Po zakończeniu obrotu ruch w osiach 3D (jeśli możliwe na raz).
   * Jeśli na raz nie, to najpierw zmiana wysokości, potem ruch 2D
   * Jeśli się nie da, to tylko ruch 2D
   * kontrola kiedy dotrze do punktu
   */
}

void BehaviorGoToPointAngle::ownStop(){
  is_finished = false;
  estimated_pose_sub.shutdown();
  estimated_speed_sub.shutdown();
  rotation_angles_sub.shutdown();
}

std::tuple<bool,std::string> BehaviorGoToPointAngle::ownCheckSituation()
{
  droneMsgsROS::ConsultBelief query_service;
  std::ostringstream capturador;
  capturador << "battery_level(self,LOW)";
  std::string query(capturador.str());
  query_service.request.query = query;
  query_client.call(query_service);
  if(query_service.response.success)
  {
    return std::make_tuple(false,"Error: Battery low, unable to perform action");
    //return false;
  }
  std::ostringstream capturador2;
  capturador2<<"flight_state(self,LANDED)";
  std::string query2(capturador2.str());
  query_service.request.query = query2;
  query_client.call(query_service);
  if(query_service.response.success)
  {
    return std::make_tuple(false,"Error: Drone landed");
    //return false;
  }

  return std::make_tuple(true,"");
}


//CallBacks
void BehaviorGoToPointAngle::estimatedSpeedCallback(const droneMsgsROS::droneSpeeds& msg){
  estimated_speed_msg=msg;
}
void BehaviorGoToPointAngle::estimatedPoseCallBack(const droneMsgsROS::dronePose& msg){
  estimated_pose_msg=msg;
}
void BehaviorGoToPointAngle::rotationAnglesCallback(const geometry_msgs::Vector3Stamped& msg){
  rotation_angles_msg=msg;
}
