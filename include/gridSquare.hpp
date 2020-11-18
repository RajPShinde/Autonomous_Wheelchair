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
 *  @file    goal.cpp
 *  @author  Raj Shinde
 *  @author  Prasheel Renkuntla
 *  @author  Shubham Sonawane
 *  @date    11/17/2020
 *  @version 1.0
 *  @brief   Autonomous Wheelchair
 *  @section Header for gridsquare
 */

#ifndef INCLUDE_GRIDSQUARE_HPP_
#define INCLUDE_GRIDSQUARE_HPP_

#include <set>
#include <string>
#include <vector>
#include <utility>
#include <random>
#include <numeric>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <iterator>
#include <algorithm>

class GridSquare {
 public:
    int currentGridSquare;  //  current grid square
    float fCost;  //  function cost

    /**
     *   @brief Constructor of class GridSquare
     *   @param none
     *   @return none
     */    
    GridSquare();

    /**
     *   @brief Destructor of class GridSquare
     *   @param none
     *   @return none
     */    
    ~GridSquare();

    /**
     *   @brief Function fo get current grid square
     *   @param none
     *   @return int, current grid square
     */    
    int getCurrentGridSquare();

    /**
     *   @brief Function to get current function Cost
     *   @param none
     *   @return float, current F cost
     */    
    float getFCost();
};

#endif  //  INCLUDE_GRIDSQUARE_HPP_
