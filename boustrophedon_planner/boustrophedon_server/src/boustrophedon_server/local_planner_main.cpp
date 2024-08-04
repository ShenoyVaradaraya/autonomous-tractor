/**
 * @file local_planner_main.cpp
 * @author Varadaraya Ganesh Shenoy (varadaraya.shenoy@fluxauto.xyz)
 * @brief 
 * @date 2023-03-25
 * 
 *  Copyright © Flux Auto India Private Limited 2023
 * 
 */

#include "boustrophedon_server/local_planner.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "local_planner");
  local_planner lp;
  lp.driver();
  return 0;
}