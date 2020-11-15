/************************************************************************
BSD 3-Clause License

Copyright (c) 2020, Raj Shinde
Copyright (c) 2020, Prasheel Renkuntla
Copyright (c) 2020, Shubham Sonawane
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *************************************************************************/

/**
 *  @copyright BSD 3-Clause License
 *  @copyright Copyright © 2020 Raj Shinde, Prasheel Renkuntla, Shubham Sonawane
 *  @file    navigateRobot.cpp
 *  @author  Raj Shinde
 *  @author  Prasheel Renkuntla
 *  @author  Shubham Sonawane
 *  @date    15/11/2020
 *  @version 1.0
 *  @brief   Autonomous Wheelchair
 *  @section Implementation file for navigation of the robot
 */

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>

#include "navigateRobot.hpp"

NavigateRobot::NavigateRobot() {
}

NavigateRobot::~NavigateRobot() {
}

void NavigateRobot::twistRobot(const geometry_msgs::TwistConstPtr &msg) {
  //  Set geometry messages to the robot
  double transVelocity = msg->linear.x;
  double rotVelocity = msg->angular.z;
  double velDiff = 0.312 * rotVelocity;
  double leftPower = (transVelocity - velDiff) / 0.039;
  double rightPower = (transVelocity + velDiff) / 0.039;
  //  check for individual node
  ROS_INFO_STREAM("\n Left wheel: " << leftPower
                  << ",  Right wheel: "<< rightPower << "\n");
}

int NavigateRobot::start(bool flag) {
  //  initialise node handle
  ros::NodeHandle nh;
  if (flag) {
    //  start a subcriber to the given topic
    ros::Subscriber sub = nh.subscribe("/cmd_vel", 1000, 
                                      &NavigateRobot::twistRobot, this);
    ros::Rate loop_rate(10);
    while (ros::ok()) {
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
  return 0;
}
