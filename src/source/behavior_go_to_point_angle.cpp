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
  safetyR0=0.75;
  safetyR1=1.5;
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
  is_finished = false;
  state = 0;
  std::cout << "ownStart" << std::endl;
  //Initialize topics
  estimated_pose_sub = node_handle.subscribe(estimated_pose_str, 1000, &BehaviorGoToPointAngle::estimatedPoseCallBack, this);
  rotation_angles_sub = node_handle.subscribe(rotation_angles_str, 1000, &BehaviorGoToPointAngle::rotationAnglesCallback, this);
  estimated_speed_sub = node_handle.subscribe(estimated_speed_str, 1000, &BehaviorGoToPointAngle::estimatedSpeedCallback, this);
  controllers_pub = node_handle.advertise<droneMsgsROS::droneCommand>(controllers_str, 1, true);
  yaw_controller_pub=node_handle.advertise<droneMsgsROS::droneYawRefCommand>(yaw_controller_str,1000);
  mode_service=node_handle.serviceClient<droneMsgsROS::setControlMode>(service_topic_str);
  drone_position_pub=node_handle.advertise< droneMsgsROS::dronePositionRefCommandStamped>(drone_position_str,1000);
  speed_topic_pub=node_handle.advertise<droneMsgsROS::droneSpeeds>(speed_topic,1000);
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
  if(target_position.z < 0.7 ) target_position.z = 0.7;
    //get speed
  if(config_file["speed"].IsDefined()){
    speed=config_file["speed"].as<float>();
  }
  else{
    speed = 5;
    std::cout<<"Could not read speed. Default speed="<<speed<<std::endl;
  }
  if(config_file["avoid_drone_id"].IsDefined()){
    intruderDistanceXY = 1000;
    leaderID=config_file["avoid_drone_id"].as<int>();
    if (leaderID == atoi(drone_id.c_str())){ //check if is not following itself
      std::cout<<"Can not avoid itself!" << std::endl;
      setStarted(false);
      return;
    }
    std::cout<<"Avoiding drone " << leaderID << std::endl;
    estimated_intruder_pose_str = std::string("/drone") + std::to_string(leaderID) + std::string("/estimated_pose");
    estimated_intruder_speed_str = std::string("/drone") + std::to_string(leaderID) + std::string("/estimated_speed");
    estimated_intruder_pose_sub = node_handle.subscribe(estimated_intruder_pose_str, 1000, &BehaviorGoToPointAngle::estimatedIntruderPoseCallBack, this);
    estimated_intruder_speed_sub = node_handle.subscribe(estimated_intruder_speed_str, 1000, &BehaviorGoToPointAngle::estimatedIntruderSpeedCallback, this);

  }
  else{
    leaderID = -1;
    std::cout<<"Could not read avoid_drone_ID. Collision avoidance disabled" << std::endl;
    intruderDistanceXY = 1000;
  }
    //get angle
  if(config_file["angle"].IsDefined()){
    angle=config_file["angle"].as<float>() * M_PI/180;
  }
  else{
    angle=0;
    std::cout<<"Could not read angle. Default angle="<<angle<<std::endl;
  }


  estimated_pose_msg = *ros::topic::waitForMessage<droneMsgsROS::dronePose>(estimated_pose_str, node_handle, ros::Duration(2));

  //calculate speeds and angle
  double distance = sqrt(pow(target_position.x-estimated_pose_msg.x,2)
                         + pow(target_position.y-estimated_pose_msg.y,2)
                         + pow(target_position.z-estimated_pose_msg.z,2));
  setpoint_speed_msg.dx = speed * (target_position.x - estimated_pose_msg.x) / distance;
  setpoint_speed_msg.dy = speed * (target_position.y - estimated_pose_msg.y) / distance;
  setpoint_speed_msg.dz = speed * (target_position.z - estimated_pose_msg.z) / distance;

  target_position.yaw=atan2(target_position.y-estimated_pose_msg.y,target_position.x-estimated_pose_msg.x)+angle;
  target_position.yaw = fmod(target_position.yaw + 2*M_PI, 2*M_PI);
  std::cout << "Setpoint_angle (rad): " << target_position.yaw << "  (deg):" << target_position.yaw * 180/M_PI <<std::endl;

  /*//behavior implementation
  droneMsgsROS::StartBehavior startRotationMessage;
  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "angle";
  out << YAML::Value << target_position.yaw * 180/M_PI;
  out << YAML::EndMap;
  std::string startRotationArguments(out.c_str());
  startRotationMessage.request.arguments = startRotationArguments;
  startRotationMessage.request.timeout = 20;*/
  //rotation_start_client.call(startRotationMessage);

  estimated_speed_msg = *ros::topic::waitForMessage<droneMsgsROS::droneSpeeds>(estimated_speed_str, node_handle, ros::Duration(2));
  estimated_pose_msg = *ros::topic::waitForMessage<droneMsgsROS::dronePose>(estimated_pose_str, node_handle, ros::Duration(2));
  droneMsgsROS::setControlMode mode;
  mode.request.controlMode.command=mode.request.controlMode.SPEED_CONTROL;
  mode_service.call(mode);

  droneMsgsROS::droneSpeeds point;
  point.dx=estimated_speed_msg.dx;
  point.dy=estimated_speed_msg.dy;
  point.dz=estimated_speed_msg.dz;
  speed_topic_pub.publish(point);

  ros::topic::waitForMessage<droneMsgsROS::droneTrajectoryControllerControlMode>(
    drone_control_mode_str, node_handle
  );
  droneMsgsROS::droneCommand msg;
  msg.command = droneMsgsROS::droneCommand::MOVE;
  controllers_pub.publish(msg);

}

void BehaviorGoToPointAngle::ownRun(){
  if(!is_finished){
    switch(state){
    case 0:{  //check if rotation is finished
        float angle_variation_maximum=.1 * M_PI/180;
        if(std::abs(target_position.yaw- fmod(estimated_pose_msg.yaw + 2*M_PI, 2*M_PI)) < angle_variation_maximum){
          state = 1;
          estimated_speed_msg = *ros::topic::waitForMessage<droneMsgsROS::droneSpeeds>(estimated_speed_str, node_handle, ros::Duration(2));
          estimated_pose_msg = *ros::topic::waitForMessage<droneMsgsROS::dronePose>(estimated_pose_str, node_handle, ros::Duration(2));
          droneMsgsROS::setControlMode mode;
          mode.request.controlMode.command=mode.request.controlMode.SPEED_CONTROL;
          mode_service.call(mode);

          droneMsgsROS::droneSpeeds point;
          point.dx=estimated_speed_msg.dx;
          point.dy=estimated_speed_msg.dy;
          point.dz=estimated_speed_msg.dz;
          speed_topic_pub.publish(point);

          ros::topic::waitForMessage<droneMsgsROS::droneTrajectoryControllerControlMode>(
            drone_control_mode_str, node_handle
          );
          droneMsgsROS::droneCommand msg;
          msg.command = droneMsgsROS::droneCommand::MOVE;
          controllers_pub.publish(msg);
          break;
        }
        else{
          float dYaw;
          float setpoint_yaw=target_position.yaw;
          float current_yaw = fmod(estimated_pose_msg.yaw + 2*M_PI, 2*M_PI);
          float yaw_diff = fmod((setpoint_yaw - current_yaw)+2*M_PI,2*M_PI);

          if(std::abs(yaw_diff) > angle_variation_maximum && std::abs(yaw_diff) < (2*M_PI - angle_variation_maximum)){
            dYaw = (-2.0) * (fmod((yaw_diff/M_PI + 1),2)-1);
            //calculate dYaw speed
          }
          droneMsgsROS::droneSpeeds droneSpeed;
          droneSpeed.dx=0;
          droneSpeed.dy=0;
          droneSpeed.dz=0;
          droneSpeed.dyaw=dYaw;
          speed_topic_pub.publish(droneSpeed);
        }
        break;
    }
    case 1:{  //movement in xyz
      float distance_variation_maximum = 0.2;
      double distanceXY = sqrt(pow(target_position.x-estimated_pose_msg.x,2)
                             + pow(target_position.y-estimated_pose_msg.y,2));
      if(intruderDistanceXY < safetyR0 && leaderID != -1){
        state = 3;
        break;
      }
      else if (intruderDistanceXY < safetyR1  && leaderID != -1){
        state = 4;
        break;
      }

      setpoint_speed_msg.dx = speed * (target_position.x - estimated_pose_msg.x) / distanceXY;
      setpoint_speed_msg.dy = speed * (target_position.y - estimated_pose_msg.y) / distanceXY;
      setpoint_speed_msg.dz = 1.0 * (target_position.z - estimated_pose_msg.z);

      if (setpoint_speed_msg.dz > 2.0 ) setpoint_speed_msg.dz = 2.0;
      else if (setpoint_speed_msg.dz < (-2.0)) setpoint_speed_msg.dz = -2.0;

      if((std::abs(target_position.x - estimated_pose_msg.x)) < distance_variation_maximum){
        setpoint_speed_msg.dx=0.0;
      }
      if((std::abs(target_position.y - estimated_pose_msg.y)) < distance_variation_maximum){
        setpoint_speed_msg.dy=0.0;
      }
      if((std::abs(target_position.z - estimated_pose_msg.z)) < distance_variation_maximum){
        setpoint_speed_msg.dz=0.0;
      }
      if(sqrt(pow(target_position.x-estimated_pose_msg.x,2)
              + pow(target_position.y-estimated_pose_msg.y,2) + pow(target_position.z-estimated_pose_msg.z,2)) < sqrt(3) * distance_variation_maximum){
        state = 2;
      }

      droneMsgsROS::droneSpeeds droneSpeed;
      droneSpeed.dx=setpoint_speed_msg.dx;
      droneSpeed.dy=setpoint_speed_msg.dy;
      droneSpeed.dz=setpoint_speed_msg.dz;
      droneSpeed.dyaw=0.0;
      speed_topic_pub.publish(droneSpeed);

      break;
    }
    case 2:{ //stop movement, start hovering
      estimated_speed_msg = *ros::topic::waitForMessage<droneMsgsROS::droneSpeeds>(estimated_speed_str, node_handle, ros::Duration(2));

      droneMsgsROS::droneSpeeds point;
      point.dx=estimated_speed_msg.dx;
      point.dy=estimated_speed_msg.dy;
      point.dz=estimated_speed_msg.dz;
      speed_topic_pub.publish(point);

      ros::topic::waitForMessage<droneMsgsROS::droneTrajectoryControllerControlMode>(
        drone_control_mode_str, node_handle
      );

      droneMsgsROS::droneSpeeds droneSpeed;
      droneSpeed.dx=0;
      droneSpeed.dy=0;
      droneSpeed.dz=0;
      speed_topic_pub.publish(droneSpeed);

      droneMsgsROS::droneCommand msg;
      msg.command = droneMsgsROS::droneCommand::HOVER;
      controllers_pub.publish(msg);

      estimated_speed_msg = *ros::topic::waitForMessage<droneMsgsROS::droneSpeeds>(estimated_speed_str, node_handle, ros::Duration(2));

      BehaviorProcess::setFinishEvent(droneMsgsROS::BehaviorEvent::GOAL_ACHIEVED);
      BehaviorProcess::setFinishConditionSatisfied(true);
      is_finished=true;
      break;
    }
    case 3:{//safety zone 0 - escpe from intruder
      if(intruderDistanceXY > safetyR1){ // start movement in normal way
        state = 1;
        break;
      }
      else if(intruderDistanceXY > safetyR0){ // stop movement
        state = 4;
        break;
      }

      float yIntruderEstimate = estimated_intruder_pose_msg.y + (estimated_intruder_speed_msg.dy/estimated_intruder_speed_msg.dx) *
          (estimated_pose_msg.x - estimated_intruder_pose_msg.x);
      float droneDirection;
      float temp_dx, temp_dy;
      droneMsgsROS::droneSpeeds droneSpeed;

      if (estimated_intruder_speed_msg.dy > 0.1 && estimated_intruder_speed_msg.dx > 0.1){
        droneDirection = (-1) * (estimated_intruder_speed_msg.dx/estimated_intruder_speed_msg.dy);
        if (yIntruderEstimate > estimated_pose_msg.y){
          temp_dy = (-1) * std::abs(droneDirection);
        }
        else{
          temp_dy = std::abs(droneDirection);
        }
        temp_dx = temp_dy / droneDirection;
        float temp_length = sqrt(pow(temp_dx,2) + pow(temp_dy,2));
        droneSpeed.dx = speed * temp_dx/temp_length;
        droneSpeed.dy = speed * temp_dy/temp_length;
      }
      else if (estimated_intruder_speed_msg.dy < 0.1 && estimated_intruder_speed_msg.dx < 0.1){
        float targetYaw = atan2(target_position.y-estimated_pose_msg.y,target_position.x-estimated_pose_msg.x);
        float intruderYaw = atan2(estimated_intruder_pose_msg.y-estimated_pose_msg.y,estimated_intruder_pose_msg.x-estimated_pose_msg.x);
        float movementYaw;
        if (std::abs(targetYaw - intruderYaw) < M_PI/2){
          if (targetYaw - intruderYaw > 0 ) movementYaw = targetYaw + (M_PI/2 - std::abs(targetYaw - intruderYaw));
          else movementYaw = targetYaw - (M_PI/2 - std::abs(targetYaw - intruderYaw));
        }
        else movementYaw = targetYaw;
        droneSpeed.dx = speed * cos(movementYaw);
        droneSpeed.dy = speed * sin(movementYaw);
        }
      else if (estimated_intruder_speed_msg.dy < 0.1){
        if (estimated_intruder_pose_msg.x < estimated_pose_msg.x){
          droneSpeed.dx = speed;
        }
        else{
          droneSpeed.dx = (-1.0) * speed;
        }
        droneSpeed.dy = 0.0;
      }
      else{
        if (estimated_intruder_pose_msg.y < estimated_pose_msg.y){
          droneSpeed.dy = speed;
        }
        else{
          droneSpeed.dy = (-1.0) * speed;
        }
        droneSpeed.dx = 0.0;
      }

      droneSpeed.dz = 0.0;
      droneSpeed.dyaw = 0.0;
      speed_topic_pub.publish(droneSpeed);

      break;
    }
    case 4:{//safety zone 1 - stop movement
      if(intruderDistanceXY < safetyR0){ // start escape
        state = 3;
        break;
      }
      else if(intruderDistanceXY > safetyR1){ // start movement in normal way
        state = 1;
        break;
      }
      float yIntruderEstimate = estimated_intruder_pose_msg.y + (estimated_intruder_speed_msg.dy/estimated_intruder_speed_msg.dx) *
          (estimated_pose_msg.x - estimated_intruder_pose_msg.x);
      float droneDirection;
      float escapeSpeed = speed * (1 - 0.5 * (intruderDistanceXY - safetyR0)/(safetyR1 - safetyR0));
      float temp_dx, temp_dy;
      droneMsgsROS::droneSpeeds droneSpeed;

      if (estimated_intruder_speed_msg.dy > 0.1 && estimated_intruder_speed_msg.dx > 0.1){
        droneDirection = (-1) * (estimated_intruder_speed_msg.dx/estimated_intruder_speed_msg.dy);
        if (yIntruderEstimate > estimated_pose_msg.y){
          temp_dy = (-1) * std::abs(droneDirection);
        }
        else{
          temp_dy = std::abs(droneDirection);
        }
        temp_dx = temp_dy / droneDirection;
        float temp_length = sqrt(pow(temp_dx,2) + pow(temp_dy,2));
        droneSpeed.dx = escapeSpeed * temp_dx/temp_length;
        droneSpeed.dy = escapeSpeed * temp_dy/temp_length;
      }
      else if (estimated_intruder_speed_msg.dy < 0.1 && estimated_intruder_speed_msg.dx < 0.1){
        float targetYaw = atan2(target_position.y-estimated_pose_msg.y,target_position.x-estimated_pose_msg.x);
        float intruderYaw = atan2(estimated_intruder_pose_msg.y-estimated_pose_msg.y,estimated_intruder_pose_msg.x-estimated_pose_msg.x);
        float movementYaw;
        if (std::abs(targetYaw - intruderYaw) < M_PI/2){
          if (targetYaw - intruderYaw > 0 ) movementYaw = targetYaw + (M_PI/2 - std::abs(targetYaw - intruderYaw));
          else movementYaw = targetYaw - (M_PI/2 - std::abs(targetYaw - intruderYaw));
        }
        else movementYaw = targetYaw;
        droneSpeed.dx = escapeSpeed * cos(movementYaw);
        droneSpeed.dy = escapeSpeed * sin(movementYaw);
        }
      else if (estimated_intruder_speed_msg.dy < 0.1){
        if (estimated_intruder_pose_msg.x < estimated_pose_msg.x){
          droneSpeed.dx = escapeSpeed;
        }
        else{
          droneSpeed.dx = (-1.0) * escapeSpeed;
        }
        droneSpeed.dy = 0.0;
      }
      else{
        if (estimated_intruder_pose_msg.y < estimated_pose_msg.y){
          droneSpeed.dy = speed;
        }
        else{
          droneSpeed.dy = (-1.0) * escapeSpeed;
        }
        droneSpeed.dx = 0.0;
      }

      droneSpeed.dz = 0.0;
      droneSpeed.dyaw = 0.0;
      speed_topic_pub.publish(droneSpeed);

      break;
    }
    default:{
        std::cout<<"State error"<<std::endl;
        break;
    }
    }
  }
}

void BehaviorGoToPointAngle::ownStop(){
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
void BehaviorGoToPointAngle::estimatedIntruderSpeedCallback(const droneMsgsROS::droneSpeeds& msg){
  estimated_intruder_speed_msg=msg;
}
void BehaviorGoToPointAngle::estimatedPoseCallBack(const droneMsgsROS::dronePose& msg){
  estimated_pose_msg=msg;
}
void BehaviorGoToPointAngle::estimatedIntruderPoseCallBack(const droneMsgsROS::dronePose& msg){
  estimated_intruder_pose_msg=msg;
  intruderDistanceXY = sqrt(pow((estimated_pose_msg.x - estimated_intruder_pose_msg.x),2)
                            + pow((estimated_pose_msg.y - estimated_intruder_pose_msg.y),2));
}
void BehaviorGoToPointAngle::rotationAnglesCallback(const geometry_msgs::Vector3Stamped& msg){
  rotation_angles_msg=msg;
}
