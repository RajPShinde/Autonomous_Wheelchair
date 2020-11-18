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
 *  @file    pathPlanner.h
 *  @author  Raj Shinde
 *  @author  Prasheel Renkuntla
 *  @author  Shubham Sonawane
 *  @date    12/11/2020
 *  @version 1.0
 *  @brief   Autonomous wheelchair
 *  @section Header file for path planning using A star plugin.
 */

#ifndef INCLUDE_PATHPLANNER_HPP_
#define INCLUDE_PATHPLANNER_HPP_

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>
#include <nav_core/base_global_planner.h>

#include <set>
#include <string>
#include <vector>
#include <utility>
#include <limits>
#include <random>
#include "gridSquare.hpp"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"

namespace astar_plugin {
class AStarPlanner : public nav_core::BaseGlobalPlanner {
 public:
    int value;
    int mapSize;  //  size of the occupancy grid map
    bool *occupancyGridMap;  //  pointer to check if map exists
    float infinity = std::numeric_limits<float>::infinity();  //  inf to start
    float tBreak;
    ros::NodeHandle ROSNodeHandle;  //  Nodehandle object
    float originX;
    float originY;
    float resolution;
    costmap_2d::Costmap2DROS *costmap_ros;
    costmap_2d::Costmap2D *costmap;
    bool initialized;  //  variable to check if path planner is initialised
    int width;
    int height;

    /**
     *   @brief Constructor of class AStar planner
     *   @param none
     *   @return none
     */
    AStarPlanner();

    /**
     *   @brief Overloaded constructor to call ros node handle.
     *   @param ros::NodeHandle
     *   @return none
     */
    explicit AStarPlanner(ros::NodeHandle &);

    /**
     *   @brief Overloaded constructor to initialise 2D cost map 
     *   @param string, name
     *   @param costmap_2d::Costmap2DROS, ROS 2D cost map
     *   @return none
     */
    AStarPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

    /**
     *   @brief Function inherited from base class to initialise map
     *   @param string, name
     *   @param costmap_2d::Costmap2DROS, ROS 2D cost map
     *   @return none
     */
    void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

    /**
     *   @brief Function to make a plan to reach the goal 
     *   @param const geometry_msgs::PoseStamped, start pose
     *   @param const geometry_msgs::PoseStamped, goal pose
     *   @param std::vector<geometry_msgs::PoseStamped>, vector plan to reach
     *   @return bool, returns true if plan exists
     */
    bool makePlan(const geometry_msgs::PoseStamped &start,
                  const geometry_msgs::PoseStamped &goal,
                  std::vector<geometry_msgs::PoseStamped> &plan);
    /**
     *   @brief Function to run astar algorithm
     *   @param int, start cell
     *   @param int, goal cell
     *   @return std::vector<int>, returns best path coordinates
     */
    std::vector<int> runAStar(int startCell, int goalCell);

    /**
     *   @brief Function to find path to reach the goal 
     *   @param int, start cell
     *   @param int, goal cell
     *   @param std::vector<float>, g function value for each cell
     *   @return std::vector<int>, if correct, then best path else empty path
     */
    std::vector<int> findPath(int startCell, int goalCell,
                              std::vector<float> g_score);

    /**
     *   @brief Function to construct a path to reach the goal 
     *   @param int, start cell
     *   @param int, goal cell
     *   @param std::vector<float>, g function value for each cell
     *   @return std::vector<int>, if correct, then best path else empty path
     */
    std::vector<int> constructPath(int startCell, int goalCell,
                                   std::vector<float> g_score);

    /**
     *   @brief Function to get map coordinates
     *   @param float, x coordinate
     *   @param float, y coordinate
     *   @return std::vector<float> returns map coordinates
     */
    std::vector<float> getMapCoordinates(float x, float y);

    /**
     *   @brief Function to calculate H cell score
     *   @param int, cell index value 
     *   @param int, cell limits
     *   @return float, H value
     */
    float calculateHCellScore(int cellIndex, int cellSquare);

    /**
     *   @brief Function to calculate cell index
     *   @param int, cell y value 
     *   @param int, cell x value
     *   @return int, cell index
     */
    int calculateCellIndex(int i, int j);

    /**
     *   @brief Function to get Cell Row index
     *   @param int, cell index value 
     *   @return int, cell row index
     */
    int getCellRowIndex(int index);

    /**
     *   @brief Function to get Cell Column index
     *   @param int, cell index value 
     *   @return int, cell column index
     */
    int getCellColIndex(int index);

    /**
     *   @brief Function to check if the cell is free
     *   @param int, cell index value 
     *   @return bool, returns true if free
     */
    bool isCellFree(int cellIndex);

    /**
     *   @brief Overloaded function to check if cell is free
     *   @param int, cell x value
     *   @param int, cell y value
     *   @return bool, returns true if free
     */
    bool isCellFree(int i, int j);

    /**
     *   @brief Function to get moving cost to a cell
     *   @param int, first cell index
     *   @param int, second cell index
     *   @return float, cell cost
     */
    float getMoveToCellCost(int cellIndex1, int cellIndex2);

   /**
     *   @brief Overloaded function to get moving cost to a cell
     *   @param int, first cell x index
     *   @param int, first cell y index
     *   @param int, second cell x index
     *   @param int, second cell y index
     *   @return float, cell cost
     */
    float getMoveToCellCost(int i1, int j1, int i2, int j2);

    /**
     *   @brief Function to find free neighbouring cell to traverse
     *   @param int, previous cell index
     *   @return std::vector<int>, new cell indices
     */
    std::vector<int> findFreeNeighborCell(int cellIndex);

    /**
     *   @brief Function to convert coordinates into a static map
     *   @param int, x coordinate
     *   @param int, y coordinate
     *   @return none
     */
    void convertToMapCoordinates(float &x, float &y);

    /**
     *   @brief Function to get cell index
     *   @param int, x coordinate
     *   @param int, y coordinate
     *   @return int, index of cell
     */
    int getCellIndex(float x, float y);

    /**
     *   @brief Function to get cell coordinates
     *   @param int, index
     *   @param float, x coordinate
     *   @param float, y coordinate
     *   @return none
     */
    void getCellCoordinates(int index, float &x, float &y);

    /**
     *   @brief Function to check whether given coordinates are under boundary
     *   @param int, x coordinate
     *   @param int, y coordinate
     *   @return bool, returns true if inside boundary
     */
    bool isCoordinateInBounds(float x, float y);

    /**
     *   @brief Function to add nearest neighbor to open list of coordinates
     *   @param std::set<GridSquare>, set of coordinates
     *   @param int, neighbour cell
     *   @param int, goal cell
     *   @param std::vector<float>, current g function value for cell
     *   @return none
     */
    void addNeighborCellToOpenList(std::set<GridSquare> &OPL,
                                         int neighborCell,
                                         int goalCell,
                                         std::vector<float> g_score);

    /**
     *   @brief Function to check if the start and end goal are valid
     *   @param int, start cell
     *   @param int, goal cell
     *   @return bool, returns true if valid.
     */    
    bool isStartAndGoalValid(int startCell, int goalCell);
};
};  //  namespace astar_plugin

#endif  //  INCLUDE_PATHPLANNER_HPP