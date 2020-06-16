#ifndef COST_H
#define COST_H

#include <vector>

double inefficiency_cost(int target_speed, int intended_lane, int final_lane, 
                         const std::vector<int> &lane_speeds);

#endif  // COST_H

/*------------------------------START OF CPP------------------------------*/


#include <cmath>

double inefficiency_cost(int target_speed, int intended_lane, int final_lane, 
                         const std::vector<int> &lane_speeds) {
  // Cost becomes higher for trajectories with intended lane and final lane 
  //   that have traffic slower than target_speed.

  /**
   * TODO: Replace cost = 0 with an appropriate cost function.
   */
   
   int dv = 2*target_speed - lane_speeds[final_lane] - lane_speeds[intended_lane];
   int current_speed = lane_speeds[final_lane];
   double cost = 1 - exp(-(1.0 * std::abs(dv) / current_speed));
   
   
   /*//Reference ANS
   double speed_intended = lane_speeds[intended_lane];
   double speed_final = lane_speeds[final_lane];
   double cost = (2.0*target_speed - speed_intended - speed_final)/target_speed;
   */
    
  return cost;
}