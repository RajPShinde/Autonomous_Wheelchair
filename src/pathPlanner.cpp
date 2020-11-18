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

PLUGINLIB_EXPORT_CLASS(astar_plugin::AStarPlanner, nav_core::BaseGlobalPlanner)

namespace astar_plugin {

  AStarPlanner::AStarPlanner() {
  }

  AStarPlanner::AStarPlanner(ros::NodeHandle &nh) {
    //  initialise node
    ROSNodeHandle = nh;
  }

  AStarPlanner::AStarPlanner(std::string name,
                             costmap_2d::Costmap2DROS *cost_ros) {
    //  overridden function call for the node initialisation
    initialize(name, cost_ros);
  }

  void AStarPlanner::initialize(std::string name,
                                costmap_2d::Costmap2DROS *cost_ros) {
  if (!initialized) {
    costmap_ros = cost_ros;
    costmap = costmap_ros->getCostmap();
    ros::NodeHandle private_nh("~/" + name);
    originX = costmap->getOriginX();
    originY = costmap->getOriginY();

    width = costmap->getSizeInCellsX();
    height = costmap->getSizeInCellsY();
    resolution = costmap->getResolution();
    mapSize = width * height;
    tBreak = 1 + 1 / (mapSize);
    //  value = 0;
    occupancyGridMap = new bool[mapSize];
    // Below implementation to fill occupancy grip map do not work.!
    // //std::map<int, int> mymap = costmap;
    // auto iy = 0;
    // auto ix = 0;
    // auto y_size = static_cast<int>(costmap->getSizeInCellsY());
    // auto x_size = static_cast<int>(costmap->getSizeInCellsX());
    // std::vector<int> yVec(y_size, 10);
    // std::vector<int> xVec(x_size, 10);

    // for (auto iyv : yVec) {
    //   for (auto ixv : xVec) {
    //     unsigned int cost = static_cast<int>(costmap->getCost(ix, iy));
    //     if (cost == 0) {
    //       occupancyGridMap[iy * width + ix] = true;
    //     } else {
    //       occupancyGridMap[iy * width + ix] = false;
    //     }
    //     ix++;
    //   }
    //   iy++;
    // }
    // while (iy < y_size) {
    //   while (ix < x_size) {
    //     unsigned int cost = static_cast<int>(costmap->getCost(ix, iy));
    //     if (cost == 0) {
    //       occupancyGridMap[iy * width + ix] = true;
    //     } else {
    //       occupancyGridMap[iy * width + ix] = false;
    //     }
    //     ix++;
    //   }
    //   iy++;
    // }

    //  Only available option is to chose for loop to ACCESS INDEX of element
    for (unsigned int iy = 0; iy < costmap->getSizeInCellsY(); iy++) {
      for (unsigned int ix = 0; ix < costmap->getSizeInCellsX(); ix++) {
        unsigned int cost = static_cast<int>(costmap->getCost(ix, iy));
        if (cost == 0) {
          occupancyGridMap[iy * width + ix] = true;
        } else {
          occupancyGridMap[iy * width + ix] = false;
        }
      }
    }
    ROS_INFO_STREAM("AStar planner initialized.");
    initialized = true;
  } else {
    ROS_WARN_STREAM("The planner has already been initialized..doing nothing");
  }
}

bool AStarPlanner::makePlan(const geometry_msgs::PoseStamped &start,
                            const geometry_msgs::PoseStamped &goal,
                            std::vector<geometry_msgs::PoseStamped> &plan) {
  if (!initialized) {
    ROS_ERROR_STREAM("The planner has not been initialized, please call"
                      << "initialize() to use the planner");
    return false;
  }
  ROS_INFO_STREAM("Got a start " << start.pose.position.x << ", "
                   << start.pose.position.y <<  " and a goal:"
                   << goal.pose.position.x << ", "
                   << goal.pose.position.y);
  plan.clear();
  if (goal.header.frame_id != costmap_ros->getGlobalFrameID()) {
    ROS_ERROR_STREAM("This planner as configured will only accept goals "
                     << " in the" << costmap_ros->getGlobalFrameID().c_str()
                     << " frame, but a goal was sent in the"
                     << goal.header.frame_id.c_str() << " frame.");
    return false;
  }
  //  take the start and goal to make plan
  float startX = start.pose.position.x;
  float startY = start.pose.position.y;

  float goalX = goal.pose.position.x;
  float goalY = goal.pose.position.y;
  convertToMapCoordinates(startX, startY);  //  sx - origin, sy -origin
  convertToMapCoordinates(goalX, goalY);  //  gx - origin, gy - origin

  int startCell = 0;
  int goalCell = 0;
  //  check if coordinates are within boundary
  if (isCoordinateInBounds(startX, startY) &&
      isCoordinateInBounds(goalX, goalY)) {
    startCell = getCellIndex(startX, startY);  //  (x/res) * width + (y/res)
    goalCell = getCellIndex(goalX, goalY);
  } else {
    ROS_WARN_STREAM("the start or goal is out of the map");
    return false;
  }

  if (isStartAndGoalValid(startCell, goalCell)) {
    std::vector<int> bestPath;
    bestPath.clear();
    //  main function call to run A star on the map
    bestPath = runAStar(startCell, goalCell);
    //  run through map to make path
    if (bestPath.size() > 0) {
      for (auto& i : bestPath) {
        float x = 0.0;
        float y = 0.0;
        float previous_x = 0.0;
        float previous_y = 0.0;

        int index = i;
        int previous_index;
        getCellCoordinates(index, x, y);
        if (i != 0) {
          previous_index = (i-1);
        } else {
          previous_index = index;
        }
        getCellCoordinates(previous_index, previous_x, previous_y);
        tf::Vector3 vectorToTarget;
        vectorToTarget.setValue(x - previous_x, y - previous_y, 0.0);

        float angle = atan2(static_cast<double>(vectorToTarget.y()),
                            static_cast<double>(vectorToTarget.x()));
        geometry_msgs::PoseStamped pose = goal;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation = tf::createQuaternionMsgFromYaw(angle);

        plan.push_back(pose);
      }
      return true;
    } else {
      ROS_WARN_STREAM("The planner failed to find a path" <<
                      " choose other goal position");
      return false;
    }
  } else {
    ROS_WARN_STREAM("Not valid start or goal");
    return false;
  }
}

void AStarPlanner::convertToMapCoordinates(float &x, float &y) {
  //  input from world, output w.r.t map
  x = x - originX;
  y = y - originY;
  getMapCoordinates(x, y);
}

std::vector<float> AStarPlanner::getMapCoordinates(float x, float y) {
  //  method to check for test cases
  std::vector<float> coordinates {x, y};
  return coordinates;
}

int AStarPlanner::getCellIndex(float x, float y) {
  int cell;
  float newX = x / (resolution);
  float newY = y / (resolution);
  //  returns cell index here
  cell = calculateCellIndex(newY, newX);
  return cell;
}

void AStarPlanner::getCellCoordinates(int index, float &x, float &y) {
  x = getCellColIndex(index) * resolution;
  y = getCellRowIndex(index) * resolution;

  x = x + originX;
  y = y + originY;

  getMapCoordinates(x, y);
}

bool AStarPlanner::isCoordinateInBounds(float x, float y) {
  //  check if the coordinates are in bounds
  bool valid = true;
  if (x > (width * resolution) || y > (height * resolution)) {
    valid = false;
  }
  return valid;
}

std::vector<int> AStarPlanner::runAStar(int startCell, int goalCell) {
  std::vector<int> bestPath;
  std::vector<float> g_score;
  //  take a vector with infinity values till map size for g function
  g_score.assign(mapSize, infinity);
  bestPath = findPath(startCell, goalCell, g_score);
  return bestPath;
}

std::vector<int> AStarPlanner::findPath(int startCell,
                                        int goalCell,
                                        std::vector<float> g_score) {
  value++;
  std::vector<int> bestPath;
  std::vector<int> emptyPath;
  GridSquare cell;
  std::set<GridSquare> openSquaresList;
  int currentCell;

  g_score[startCell] = 0;
  cell.currentGridSquare = startCell;
  cell.fCost = g_score[startCell] + calculateHCellScore(startCell, goalCell);
  //  open list to have free cells
  openSquaresList.insert(cell);
  currentCell = startCell;

  while (!openSquaresList.empty() && g_score[goalCell] == infinity) {
    currentCell = openSquaresList.begin()->currentGridSquare;
    openSquaresList.erase(openSquaresList.begin());
    std::vector<int> neighborCells;
    neighborCells = findFreeNeighborCell(currentCell);
    for (auto& i : neighborCells) {
      if (g_score[i] == infinity) {
          g_score[i] = g_score[currentCell] +
                                          getMoveToCellCost(currentCell,
                                          i);
        //  add neighbor cells to list
        addNeighborCellToOpenList(openSquaresList,
                                        i,
                                        goalCell, g_score);
      }
    }
  }
  if (g_score[goalCell] != infinity) {
    //  planner constructs path from here
    bestPath = constructPath(startCell, goalCell, g_score);
    return bestPath;
  } else {
    ROS_INFO_STREAM("Failure to find a path !");
    return emptyPath;
  }
}


std::vector<int> AStarPlanner::constructPath(int startCell,
                                             int goalCell,
                                             std::vector<float> g_score) {
  std::vector<int> bestPath;
  std::vector<int> path;

  path.insert(path.begin() + bestPath.size(), goalCell);
  int currentCell = goalCell;
  //  construct a path that has minimum g and h function value
  while (currentCell != startCell) {
    std::vector<int> neighborCells;
    neighborCells = findFreeNeighborCell(currentCell);
    std::vector<float> gScoresNeighbors;
    for (auto& i : neighborCells) {
      gScoresNeighbors.push_back(g_score[i]);
    }
    std::vector<float>::iterator result =
                  std::min_element(gScoresNeighbors.begin(),
                                   gScoresNeighbors.end());

    int posMinGScore = std::distance(gScoresNeighbors.begin(), result);
    currentCell = neighborCells[posMinGScore];

    path.insert(path.begin() + path.size(), currentCell);
  }
  int p_size = path.size();
  std::vector<int> tempPath = path;
  auto it = 0;
  //  loop through to make path
  for (auto& i : path) {
    bestPath.insert(bestPath.begin() + bestPath.size(),
                    tempPath[(p_size - (it + 1))]);
    ++it;
  }
  return bestPath;
}

void AStarPlanner::addNeighborCellToOpenList(
  std::set<GridSquare> &openSquaresList,
    int neighborCell,
    int goalCell,
    std::vector<float> g_score) {
  GridSquare cell;
  cell.currentGridSquare = neighborCell;
  cell.fCost = g_score[neighborCell] +
               calculateHCellScore(neighborCell, goalCell);
  openSquaresList.insert(cell);
}

std::vector<int> AStarPlanner::findFreeNeighborCell(int cell) {
  int rowIndex = getCellRowIndex(cell);
  int colIndex = getCellColIndex(cell);
  int neighborIndex;
  std::vector<int> freeNeighborCells = {0};
  std::vector<int> row {-1, 0, 1};
  std::vector<int> col {-1, 0, 1};
  //  check through the neighbor cells
  for (auto& i : row) {
    for (auto& j : col) {
      if ((rowIndex + i >= 0) && (rowIndex + i < height)
        && (colIndex + j >= 0) && (colIndex + j < width)
        && (!(i == 0 && j == 0))) {
        neighborIndex = ((rowIndex + i) * width) + (colIndex + j);
        if (isCellFree(neighborIndex)) {
          freeNeighborCells.push_back(neighborIndex);
        }
      }
    }
  }
  return freeNeighborCells;
}

bool AStarPlanner::isStartAndGoalValid(int startCell, int goalCell) {
  bool isvalid = true;
  bool isFreeStartCell = isCellFree(startCell);
  bool isFreeGoalCell = isCellFree(goalCell);
  if (startCell == goalCell) {  //  same coordinates
    isvalid = false;
  } else {
    if (!isFreeStartCell && !isFreeGoalCell) {  //  if the cells are empty
      isvalid = false;
    } else {
      if (!isFreeStartCell) {  //  if start cell is empty
        isvalid = false;
      } else {
        if (!isFreeGoalCell) {  //  if goal cell is empty
          isvalid = false;
        } else {
          if (findFreeNeighborCell(goalCell).size() == 0) {
            isvalid = false;
          } else {
            if (findFreeNeighborCell(startCell).size() == 0) {
              isvalid = false;
            }
          }
        }
      }
    }
  }
  return isvalid;
}

float AStarPlanner::getMoveToCellCost(int i1, int j1, int i2, int j2) {
  float moveCost = infinity;
  //  if moving in diagonal, then h movecost is root 2
  if ((j2 == j1 + 1 && i2 == i1 + 1) || (i2 == i1 - 1 && j2 == j1 + 1)
      || (i2 == i1 - 1 && j2 == j1 - 1) || (j2 == j1 - 1 && i2 == i1 + 1)) {
    moveCost = 1.4;
  } else {
    //  if moving in straight line, then h movecost is 1
    if ((j2 == j1 && i2 == i1 - 1) || (i2 == i1 && j2 == j1 - 1)
       || (i2 == i1 + 1 && j2 == j1) || (i2 == i1 && j2 == j1 + 1)) {
      moveCost = 1;
    }
  }
  return moveCost;
}

float AStarPlanner::getMoveToCellCost(int cellIndex1, int cellIndex2) {
  int i1 = 0, i2 = 0, j1 = 0, j2 = 0;
  i1 = getCellRowIndex(cellIndex1);
  j1 = getCellColIndex(cellIndex1);
  i2 = getCellRowIndex(cellIndex2);
  j2 = getCellColIndex(cellIndex2);
  return getMoveToCellCost(i1, j1, i2, j2);
}

float AStarPlanner::calculateHCellScore(int cellIndex, int goalCellSquare) {
  int x1 = getCellRowIndex(goalCellSquare);
  int y1 = getCellColIndex(goalCellSquare);
  int x2 = getCellRowIndex(cellIndex);
  int y2 = getCellColIndex(cellIndex);
  float eucNorm = std::sqrt(((x1 - x2)*(x1 - x2)) + ((y1 - y2)*(y1 - y2)));
  return eucNorm;
}

int AStarPlanner::calculateCellIndex(int i, int j) {
  return (i * width) + j;
}

int AStarPlanner::getCellRowIndex(int index) {
  return index / width;
}

int AStarPlanner::getCellColIndex(int index) {
  return index % width;
}

bool AStarPlanner::isCellFree(int i, int j) {
  int cellIndex = (i * width) + j;
  return occupancyGridMap[cellIndex];
}

bool AStarPlanner::isCellFree(int cellIndex) {
  return occupancyGridMap[cellIndex];
}

};  // namespace astar_plugin

bool operator<(GridSquare const &c1,
               GridSquare const &c2) {
  return c1.fCost < c2.fCost;
}
