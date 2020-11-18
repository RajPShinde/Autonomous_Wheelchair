<img src="logo.png"/>

[![Build Status](https://travis-ci.org/RajPShinde/Autonomous_Wheelchair.svg?branch=master)](https://travis-ci.org/RajPShinde/Autonomous_Wheelchair)
[![Coverage Status](https://coveralls.io/repos/github/RajPShinde/Autonomous_Wheelchair/badge.svg?branch=master&service=github)](https://coveralls.io/github/RajPShinde/Autonomous_Wheelchair?branch=master&service=github)
[![License BSD 3-Clause](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://github.com/RajPShinde/Autonomous_Wheelchair/blob/master/LICENSE)
[![Documentation](https://img.shields.io/badge/docs-generated-brightgreen.svg)](https://github.com/RajPShinde/Autonomous_Wheelchair/doxygen)
---

## Authors
* **Raj Prakash Shinde** [Porfolio](https://rajpshinde.github.io/) 
* **Shubham Sonawane** [Porfolio](https://shubhamsonawane.wixsite.com/portfolio)
* **Prasheel Renkuntla** [Porfolio](https://prasheel24.github.io/)

## Description
A ROS Package to make a Differential-Drive Wheelchair Navigate Autonomously in an indoor Environment using onboard LiDAR and Sonar Sensors. The Wheelchair provides manual control to user through a joystick, in case the wheelchair deviates from its path. 
<p align="center">
<img src="data/simulation.gif"/>
</p>
<p align="center">
<img src="data/wheelchair.gif"/>
</p>

## Dependencies
1. Ubuntu 16.04 or >
2. ROS Kinetic or >
3. Navigation Stack

## Sprint Planning and Discussion
Sprint Discussion's - [Link](https://docs.google.com/document/d/1YxuiONLKsmspN5a6GJSREl9pPx3vZLULn2Mh-LnGHLA/edit?usp=sharing)

## Agile Iterative Process Log
Access Product Backlog, Worklog & Iteration Log here- [Link](https://docs.google.com/spreadsheets/d/16jTj_WTD0Le5l6ijkAiI3PhKVDuv9GQzufeO_Imgy9o/edit?usp=sharing)

## Build
Steps to build
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone https://github.com/RajPShinde/Autonomous_Wheelchair
cd ~/catkin_ws/
catkin_make
```
To build tests
```
catkin_make tests
```
## Run
Mapping
```
source devel/setup.bash
roslaunch autonomous_wheelchair simulationMapping.launch
```
Navigation
```
source devel/setup.bash
roslaunch autonomous_wheelchair simulationNavigation.launch
```
Tests
```
source devel/setup.bash
catkin_make run_tests
```
## TODO
REplace with TEB Motion Planner

## Bugs
None

## Disclaimer
```
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
```
