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
 *  @file    pathPlanner.cpp
 *  @author  Raj Shinde
 *  @author  Prasheel Renkuntla
 *  @author  Shubham Sonawane
 *  @date    12/11/2020
 *  @version 1.0
 *  @brief   Autonomous wheelchair
 *  @section Implementation file for path planning algorithm
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <ros/console.h>
#include <pluginlib/class_loader.h>
#include <pluginlib/class_list_macros.h>

#include <set>
#include <numeric>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <iterator>
#include <algorithm>
#include <string>
#include <vector>
#include <map>
#include <limits>

#include "pathPlanner.hpp"
// #include "gridSquare.hpp"

PLUGINLIB_EXPORT_CLASS(astar_plugin::AStarPlanner, nav_core::BaseGlobalPlanner)

namespace astar_plugin {

  AStarPlanner::AStarPlanner() {
  }

  AStarPlanner::AStarPlanner(ros::NodeHandle &nh) {
    
  }

  AStarPlanner::AStarPlanner(std::string name,
                             costmap_2d::Costmap2DROS *cost_ros) {
    
  }

  void AStarPlanner::initialize(std::string name,
                                costmap_2d::Costmap2DROS *cost_ros) {
  
}

bool AStarPlanner::makePlan(const geometry_msgs::PoseStamped &start,
                            const geometry_msgs::PoseStamped &goal,
                            std::vector<geometry_msgs::PoseStamped> &plan) {
  
}

void AStarPlanner::convertToMapCoordinates(float &x, float &y) {
  
}

std::vector<float> AStarPlanner::getMapCoordinates(float x, float y) {
  
}

int AStarPlanner::getCellIndex(float x, float y) {
  
}

void AStarPlanner::getCellCoordinates(int index, float &x, float &y) {
  
}

bool AStarPlanner::isCoordinateInBounds(float x, float y) {
  
}

std::vector<int> AStarPlanner::runAStar(int startCell, int goalCell) {
  
}

std::vector<int> AStarPlanner::findPath(int startCell,
                                        int goalCell,
                                        std::vector<float> g_score) {
  
}


std::vector<int> AStarPlanner::constructPath(int startCell,
                                             int goalCell,
                                             std::vector<float> g_score) {
  
}

void AStarPlanner::addNeighborCellToOpenList(
  std::set<GridSquare> &openSquaresList,
    int neighborCell,
    int goalCell,
    std::vector<float> g_score) {
  
}

std::vector<int> AStarPlanner::findFreeNeighborCell(int cell) {
  
}

bool AStarPlanner::isStartAndGoalValid(int startCell, int goalCell) {
  
}

float AStarPlanner::getMoveToCellCost(int i1, int j1, int i2, int j2) {
  
}

float AStarPlanner::getMoveToCellCost(int cellIndex1, int cellIndex2) {
  
}

float AStarPlanner::calculateHCellScore(int cellIndex, int goalCellSquare) {
  
}

int AStarPlanner::calculateCellIndex(int i, int j) {

}

int AStarPlanner::getCellRowIndex(int index) {

}

int AStarPlanner::getCellColIndex(int index) {
 
}

bool AStarPlanner::isCellFree(int i, int j) {
  
}

bool AStarPlanner::isCellFree(int cellIndex) {
  
}

};  // namespace astar_plugin

bool operator<(GridSquare const &c1,
               GridSquare const &c2) {
  
}
