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
 *  @file    testGridSquare.cpp
 *  @author  Raj Shinde
 *  @author  Prasheel Renkuntla
 *  @author  Shubham Sonawane
 *  @date    11/17/2020
 *  @version 1.0 
 *  @brief   Autonomous Wheelchair
 *  @section To test grid squares of the map. for planner
 */

#include <gtest/gtest.h>
#include "gridSquare.hpp"

/**
 * @def TEST(TestGridSquare, testGetCurrentSquare)
 * @brief To check if the current gridsquare is 0
 */
TEST(TestGridSquare, testGetCurrentSquare) {
int res = 0;
GridSquare testGS;
//  Should return true when initial gridsquare is 0
ASSERT_EQ(res, testGS.getCurrentGridSquare());
}

/**
 * @def TEST(TestGridSquare, testFCost)
 * @brief To check if the current fCost is 0
 */
TEST(TestGridSquare, testFCost) {
float resF = 0.0;
GridSquare testGS;
//  Should return true when initial fcost is 0
EXPECT_EQ(resF, testGS.getFCost());
}

/**
 *  @brief  Main Function for running tests of GridSquare Class
 *  @param  int argc, char argv
 *  @return int
 */
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
