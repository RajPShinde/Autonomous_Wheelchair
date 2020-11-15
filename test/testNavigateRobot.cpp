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
 *  @copyright Copyright © 2019 Raj Shinde, Prasheel Renkuntla, Shubham Sonawane
 *  @file    testNavigateRobot.cpp
 *  @author  Raj Shinde
 *  @author  Prasheel Renkuntla
 *  @author  Shubham Sonawane
 *  @date    15/11/2020
 *  @version 1.0 
 *  @brief   Autonomous Wheelchair
 *  @section To test Navigation of wheelchair
 */

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <gtest/gtest.h>
#include "navigateRobot.hpp"

/**
 * @def TEST(TestNavigateRobot, testTwistRobot)
 * @brief To check if the robot is able to twist on given topic
 */
TEST(TestNavigateRobot, testTwistRobot) {
geometry_msgs::Twist msg;
msg.linear.x = 0.0;
msg.linear.y = 0.0;
msg.linear.z = 0.0;
msg.angular.x = 0.0;
msg.angular.y = 0.0;
msg.angular.z = 0.0;
//  initialise the test object
NavigateRobot testN = NavigateRobot();

ros::NodeHandle nh;
//  Create a publisher on the topic
ros::Publisher pub =
nh.advertise<geometry_msgs::Twist>
("/navigation_velocity_smoother/raw_cmd_vel", 5);

//  run publisher and check publisher and subscriber
ros::WallDuration(5.0).sleep();
ros::spinOnce();
EXPECT_EQ(0, pub.getNumSubscribers());

//  run subscriber and check publisher and subscriber
ros::Subscriber sub =
nh.subscribe("/navigation_velocity_smoother/raw_cmd_vel", 5,
             &NavigateRobot::twistRobot, &testN);

for (int i = 0; i < 10; i++) {
  pub.publish(msg);
  ros::WallDuration(0.1).sleep();
  ros::spinOnce();
}

//  Should return 1 as there is one publisher
EXPECT_EQ(1, sub.getNumPublishers());
//  Should return 1 as there is one subscriber
EXPECT_EQ(1, pub.getNumSubscribers());
}

TEST(TestNavigateRobot, testStart) {
    geometry_msgs::Twist msg;
    msg.linear.x = 0.0;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;
    NavigateRobot testN = NavigateRobot();

    ros::NodeHandle nh;
    ros::Publisher pub =
    nh.advertise<geometry_msgs::Twist>
    ("/navigation_velocity_smoother/raw_cmd_vel", 5, &testN);

    //  run fake publisher and subscriber
    ros::WallDuration(5.0).sleep();
    ros::spinOnce();
    EXPECT_EQ(0, pub.getNumSubscribers());

    int res = 0;
    testN.flag = true;
    // EXPECT_EQ(1, .getNumPublishers());
    EXPECT_EQ(0, testN.start(false));
}

/**
 *  @brief  Main Function for running tests for Navigate Robot Class
 *  @param  int argc, char argv
 *  @return int
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "testNavigator");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
