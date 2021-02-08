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
  float v1, v2, vb, linear, angular;
  linear=msg.linear.x;
  angular=msg.angular.z;
  vb= angular * bodyRadius;
  v1=linear-vb;
  v2=linear+vb;
  targetRpm1 = (v1*60)/(2*PI*wheelRadius);
  if (targetRpm1 > 70) {
    targetRpm1 = 70;
   } else if (targetRpm1 < -70) {
    targetRpm1 = -70;
   }
  targetRpm2 = (v2*60)/(2*PI*wheelRadius);
  if (targetRpm2 > 70) {
    targetRpm2 = 70;
   } else if (targetRpm2 < -70) {
    targetRpm2 = -70;
   }
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
if (node.connected()) {
str_msg.data = "OK";
      
digitalWrite(13, LOW);   
pub.publish(&str_msg);

}
node.spinOnce();
delay(100);

  //  targetRpm1 = 70;
  //    targetRpm1 = 70;
  //  Serial.println("set 70");
  //  delay(5000);
  //  targetRpm2 = 50;
  //  Serial.println("set 50");
  //  delay(5000);
  //  targetRpm2 = 30;
  //  Serial.println("set 30");
  //  delay(5000);
  //  targetRpm2 = -30;
  //  Serial.println("set -30");
  //  delay(5000);
  //  targetRpm2 = -50;
  //  Serial.println("set -50");
  //  delay(5000);
  //  targetRpm2 = -70;
  //  Serial.println("set -70");
  //  delay(5000);
  
}

void encoderCallback1() {
  tick1 += 1;   // Increment encoder count on interrupt
  total1 += 1;
  dir1 =  digitalRead(encB1);
  if (dir1 == 0) 
    dir1 = -1;
}

void encoderCallback2() {
  tick2 += 1;   // Increment encoder count on interrupt
  total2 += 1;
  dir2 =  digitalRead(encB2);
  if (dir2 == 0) 
    dir2 = 1;
  else 
    dir2=-1;
}

int computePid1(const double &currentRpm, const double &setpoint) {
  double error = setpoint - currentRpm;
  int processValue = kp*error + ki*errorSum1 + kd*(error-prevError1);
  errorSum1 += error;
  prevError1 = error;
  if (errorSum1 > 2000) errorSum1 = 2000;
  if (errorSum1 < 1000) errorSum1 = 1000;
  return processValue;
}

int computePid2(const double &currentRpm, const double &setpoint) {
  double error = setpoint - currentRpm;
  int processValue = kp*error + ki*errorSum2 + kd*(error-prevError2);
  errorSum2 += error;
  prevError2 = error;
  if (errorSum2 >2000) errorSum2 = 2000;
  if (errorSum2 <1000) errorSum2 = 1000;
  return processValue;
}

ISR(TIMER1_OVF_vect)        // interrupt service routine - tick every 0.1sec
{
   TCNT1 = 59286;

  double currentRpm1 = (600*tick1*dir1)/(71*ppr);
  double currentRpm2 = (600*tick2*dir2)/(71*ppr);
  
  
  //  String mstr = String(currentRpm1);
  //  char result[8];
  //  mstr.toCharArray(result, 4);
  //  node.logwarn("Current RPM 1");
  //  node.logwarn(result);
  
  //  String mstr1 = String(targetRpm1);
  //  char result1[8];
  //  mstr1.toCharArray(result1, 4);
  //  node.logwarn("Target RPM 1");
  //  node.logwarn(result1);
  
  //  String mstr2 = String(currentRpm2);
  //  char result2[8];
  //  mstr2.toCharArray(result2, 4);
  //  node.logwarn("Current RPM 2");
  //  node.logwarn(result2);
    
  //  String mstr3 = String(targetRpm2);
  //  char result3[8];
  //  mstr3.toCharArray(result3, 4);
  //  node.logwarn("Target RPM 2");
  //  node.logwarn(result3);
  
  //     String mstr6 = String(tick1);
  //  char result6[8];
  //  mstr6.toCharArray(result6, 8);
  //  node.logwarn("Encoder 1");
  //  node.logwarn(result6);
  //  node.logwarn("\n");
  
  //  String mstr7 = String(tick2);
  //  char result7[8];
  //  mstr7.toCharArray(result7, 8);
  //  node.logwarn("Encoder 2");
  //  node.logwarn(result7);
  //  node.logwarn("\n");

  tick1 = 0;                // reset encoder counts
  tick2 = 0;                // reset encoder counts
  int pwm1=computePid1((currentRpm1), targetRpm1);
  int pwm2=computePid2((currentRpm2), targetRpm2);
  rpm.linear.x=currentRpm1;
  rpm.linear.y=currentRpm2;
  rpm.linear.z=targetRpm1;
  rpm.angular.x=targetRpm2;
  rpm.angular.y=pwm1;
  rpm.angular.z=pwm2;

  pub_msg.publish(&rpm);

  //   String mstr4 = String(pwm1);
  //  char result4[8];
  //  mstr4.toCharArray(result4, 8);
  //  node.logwarn("PWM 1");
  //  node.logwarn(result4);
  //  node.logwarn("\n");
  //  String mstr5 = String(pwm2);
  //  char result5[8];
  //  mstr5.toCharArray(result5, 8);
  //  node.logwarn("PWM 2");
  //  node.logwarn(result5);
  //  node.logwarn("\n");


  if (targetRpm1 == 0) {
    sb1.writeMicroseconds(1500);
  }
  else if (pwm1 < 2000 && pwm1 >1000) {
    sb1.writeMicroseconds(pwm1);
  }
  else{
    if (pwm1>2000) {
      sb1.writeMicroseconds(2000);
    }
    else if (pwm1<1000) {
      sb1.writeMicroseconds(1000);
    }
  }
  
  if (targetRpm2 == 0) {
    sb2.writeMicroseconds(1500);
  }
  else if (pwm2 < 2000 && pwm2 >1000) {
    sb2.writeMicroseconds(pwm2);
  }
  else{
    if (pwm2>2000) {
      sb2.writeMicroseconds(2000);
    }
    else if (pwm2<1000) {
      sb2.writeMicroseconds(1000);
    }
  }
  node.spinOnce();
}
