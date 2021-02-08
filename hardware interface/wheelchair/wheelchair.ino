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
 *  @file    wheelchair_navigation.ino
 *  @author  Raj Shinde
 *  @date    22/05/2020
 *  @version 0.1
 *  @brief   Autonomous Wheelchair
 *  @section File to control motor speed
 */
 
#include<Servo.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <SoftwareSerial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

#define PI 3.141

const int encA1 = 2;
const int encB1 = 3; 
const int encA2 = 19;
const int encB2 = 18; 
const double ppr = 7;
int total1 = 0;
int total2 = 0;
double kp = 2;
double ki = 1;
double kd = 2;
double errorSum1 = 0;
double errorSum2 = 0;
double prevError1 = 0;
double prevError2 = 0;
double targetRpm1 = 0;
double targetRpm2 = 0;
double tick1 = 0;
double tick2 = 0;
int dir1 = 0;
int dir2 = 0;
double rpm1 = 0;
double rpm2 = 0;
Servo sb1;
Servo sb2;

//robot dimensions
double bodyRadius = 0.312;
double wheelRadius = 0.0779;

void commandCallBack(const geometry_msgs::Twist &msg) {

}

ros::NodeHandle node;
std_msgs::String str_msg;
std_msgs::String speed_msg;
geometry_msgs::Twist rpm;
ros::Subscriber<geometry_msgs::Twist> sub("/wheelchair/mobile_base_controller/cmd_vel", &commandCallBack);
ros::Publisher pub("Ack", &str_msg);
ros::Publisher pub_msg("speed", &rpm);

void setup() {
  node.initNode();
  node.subscribe(sub);  
  node.advertise(pub);
  node.advertise(pub_msg);
  sb1.attach(5);
  sb2.attach(6);
  sb1.writeMicroseconds(1500);
  sb2.writeMicroseconds(1500);
  pinMode(encA1,INPUT_PULLUP);
  pinMode(encB1,INPUT_PULLUP);
  pinMode(encA2,INPUT_PULLUP);
  pinMode(encB2,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), encoderCallback1, RISING);
  attachInterrupt(digitalPinToInterrupt(19), encoderCallback2, RISING);
  Serial.begin(57600);
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 59286;   // preload timer 65536-16MHz/256/2Hz (34286 for 0.5sec) (59286 for 0.1sec)
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  interrupts();             // enable all interrupts
}

void loop() {
  
}

void encoderCallback1() {

}

void encoderCallback2() {

}

int computePid1(const double &currentRpm, const double &setpoint) {

}

int computePid2(const double &currentRpm, const double &setpoint) {

}

ISR(TIMER1_OVF_vect)        // interrupt service routine - tick every 0.1sec
{

}
