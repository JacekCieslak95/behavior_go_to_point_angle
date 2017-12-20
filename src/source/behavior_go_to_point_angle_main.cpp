/*!*******************************************************************************************
 *  \file       behavior_go_to_point_angle_main.cpp
 *  \brief      behavior go to point angle file.
 *  \details    This file contains the behaviorGoToPointAngle main file.
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
#include "../include/behavior_go_to_point_angle.h"
#include <boost/thread/thread.hpp>

int main(int argc, char** argv){
  ros::init(argc, argv, ros::this_node::getName());

  std::cout << ros::this_node::getName() << std::endl;

  BehaviorGoToPointAngle behavior;
  behavior.setUp();
  ros::Rate rate(10);

  while(ros::ok()){
    ros::spinOnce();
    behavior.run();
    rate.sleep();
  }
  return 0;
}
