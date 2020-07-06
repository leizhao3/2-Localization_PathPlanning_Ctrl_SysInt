#ifndef HYBRID_BREADTH_FIRST_H_
#define HYBRID_BREADTH_FIRST_H_

#include <vector>

using std::vector;

class HBF {
 public:
  // Constructor
  HBF();

  // Destructor
  virtual ~HBF();

  // HBF structs
  struct maze_s {
    int g;  // iteration
    int f;
    double x;
    double y;
    double theta; //theta is between (0, 2*M_PI)
  };

  struct maze_path {
    vector<vector<vector<int>>> closed; 
      /** @param closed store the stack/ind(x)/ind(y) of the closed coordinates.
        *               closed[stack][idx(x)][idx(y)] = 1;
        */
    vector<vector<vector<maze_s>>> came_from;
      /** @param came_from store the maze_s.theta/x/y where the current state come from. 
        *                  Access by HBF::maze_s = came_from[stack][idx(x)][idx(y)]
        */
    maze_s final;
  };
  
  // HBF functions
  int theta_to_stack_number(double theta);

  int idx(double float_num);

  vector<maze_s> expand(maze_s &state);

  vector<maze_s> reconstruct_path(vector<vector<vector<maze_s>>> &came_from, 
                                  vector<double> &start, HBF::maze_s &final);

  maze_path search(vector<vector<int>> &grid, vector<double> &start, 
                   vector<int> &goal);
  
  vector<vector<vector<int>>> generate_heuristic(vector< vector<int> > &grid, 
                                                 vector<int> &goal);
  
  double heuristic(double x, double y, vector<int> &goal);

  static bool compare_maze_s(const HBF::maze_s &lhs, const HBF::maze_s &rhs);

 private:
  const int NUM_THETA_CELLS = 90;
  const double SPEED = 1.45;
  const double LENGTH = 0.5;
};



/*------------------------------START OF CPP------------------------------*/


#include <math.h>
#include <iostream>
#include <iomanip>
#include <vector>
#include <algorithm>
//#include "hybrid_breadth_first.h"

using std::vector;
using std::cout;
using std::endl;
using std::setw;

// Initializes HBF
HBF::HBF() {}

HBF::~HBF() {}

int HBF::theta_to_stack_number(double theta){
  /** @param theta Takes an angle (in RADIANS) and returns which "stack" in the 3D 
   * configuration space this angle corresponds to. Angles near 0 go in the 
   * lower stacks while angles near 2 * pi go in the higher stacks.
   */
  double new_theta = fmod((theta + 2 * M_PI),(2 * M_PI)); 
    /** @param new_theta is between [0,2 * M_PI]
     */
  int stack_number = (int)(round(new_theta * NUM_THETA_CELLS / (2*M_PI))) 
                   % NUM_THETA_CELLS;
      /** @param stack_number is between (0,90); 
       *  @param NUM_THETA_CELLS = 90;
       */

  return stack_number;
}

int HBF::idx(double float_num) {
  // Returns the index into the grid for continuous position. So if x is 3.621, 
  //   then this would return 3 to indicate that 3.621 corresponds to array 
  //   index 3.
  return int(floor(float_num));
}


vector<HBF::maze_s> HBF::expand(HBF::maze_s &state) {
  /** Explore all potential expand for the next step. 
   *  @param state the current state of the vehicle
   */
  int g = state.g;
  double x = state.x;
  double y = state.y;
  double theta = state.theta;
    
  int g2 = g+1; //increase iteration
  vector<HBF::maze_s> next_states; 
  /** @param next_states stores all possible next state based on current stage & control
   * 
   */

  //cycle through all steering angle with 5 deg increment
  for(double delta_i = -35; delta_i < 40; delta_i+=5) {
    double delta = M_PI / 180.0 * delta_i; //convert deg to rad
    double omega = SPEED / LENGTH * tan(delta); //omega = angular acceleration
    double theta2 = theta + omega;
    if(theta2 < 0) {
      theta2 += 2*M_PI;
    }
    double x2 = x + SPEED * cos(theta);
    double y2 = y + SPEED * sin(theta);
    HBF::maze_s state2;
    state2.g = g2;
    state2.x = x2;
    state2.y = y2;
    state2.theta = theta2;
    next_states.push_back(state2);
  }

  return next_states;
}

vector< HBF::maze_s> HBF::reconstruct_path(
  vector<vector<vector<HBF::maze_s>>> &came_from, vector<double> &start, 
  HBF::maze_s &final) {
  /** Find the path {final, final-1, ..., start+1, start}
   *  @param came_from store the theta/x/y where the current state come from. 
   *                   Access by HBF::maze_s = came_from[stack][idx(x)][idx(y)]
   *  @param start the start location
   *  @param final the final location
   */

  vector<maze_s> path = {final};
  
  int stack = theta_to_stack_number(final.theta);

  maze_s current = came_from[stack][idx(final.x)][idx(final.y)];
  
  stack = theta_to_stack_number(current.theta);
  
  double x = current.x;
  double y = current.y;

  while(x != start[0] || y != start[1]) {
    path.push_back(current);
    current = came_from[stack][idx(x)][idx(y)];
    x = current.x;
    y = current.y;
    stack = theta_to_stack_number(current.theta);
  }
  
  return path;
}

HBF::maze_path HBF::search(vector< vector<int> > &grid, vector<double> &start, 
                           vector<int> &goal) {
  // Working Implementation of breadth first search. Does NOT use a heuristic
  //   and as a result this is pretty inefficient. Try modifying this algorithm 
  //   into hybrid A* by adding heuristics appropriately.

  /**
   * TODO: Add heuristics and convert this function into hybrid A*
   */
  vector<vector<vector<int>>> closed(
    NUM_THETA_CELLS, vector<vector<int>>(grid[0].size(), vector<int>(grid.size())));
  vector<vector<vector<maze_s>>> came_from(
    NUM_THETA_CELLS, vector<vector<maze_s>>(grid[0].size(), vector<maze_s>(grid.size())));


  double theta = start[2];
  int stack = theta_to_stack_number(theta);
  int g = 0;

  maze_s state;
  state.g = g;
  state.x = start[0];
  state.y = start[1];
  state.f = g + heuristic(state.x, state.y, goal);
  state.theta = theta;

  closed[stack][idx(state.x)][idx(state.y)] = 1;
  came_from[stack][idx(state.x)][idx(state.y)] = state;
  int total_closed = 1;
  vector<maze_s> opened = {state};
  bool finished = false;
  while(!opened.empty()) {
    std::sort(opened.begin(),opened.end(),compare_maze_s);
    maze_s current = opened[0]; //grab first elment
    opened.erase(opened.begin()); //pop first element

    int x = current.x;
    int y = current.y;

    if(idx(x) == goal[0] && idx(y) == goal[1]) {
      std::cout << "found path to goal in " << total_closed << " expansions" 
                << std::endl;
      maze_path path;
      path.came_from = came_from;
      path.closed = closed;
      path.final = current;

      return path;
    }

    vector<maze_s> next_state = expand(current);

    for(int i = 0; i < next_state.size(); ++i) {
      int g2 = next_state[i].g;
      double x2 = next_state[i].x;
      double y2 = next_state[i].y;
      double theta2 = next_state[i].theta;

      //Check whether or not the next_state is inside the grid
      if((x2 < 0 || x2 >= grid.size()) || (y2 < 0 || y2 >= grid[0].size())) {
        // invalid cell
        continue;
      }

      int stack2 = theta_to_stack_number(theta2);

      //Check whether or not the next cell is closed or has obstacle
      if(closed[stack2][idx(x2)][idx(y2)] == 0 && grid[idx(x2)][idx(y2)] == 0) {
        opened.push_back(next_state[i]); //stored in the opened to keep the while loop running
        closed[stack2][idx(x2)][idx(y2)] = 1;
        came_from[stack2][idx(x2)][idx(y2)] = current;
        ++total_closed;
      }
    }
  }

  std::cout << "no valid path." << std::endl;
  HBF::maze_path path;
  path.came_from = came_from;
  path.closed = closed;
  path.final = state;

  return path;
}


double HBF::heuristic(double x, double y, vector<int> &goal){
  return fabs(y - goal[0]) + fabs(x - goal[1]); //return grid distance to goal
}

bool HBF::compare_maze_s(const HBF::maze_s &lhs, const HBF::maze_s &rhs){
  return lhs.f < rhs.f;
}



vector<vector<vector<int>>> HBF::generate_heuristic(vector< vector<int> > &grid, 
                                                 vector<int> &goal) {
  /** Generate heuristic matrix
   * 
   */
  
  /**
   * @param h1  non-holonomic-without-obstacles heuristic
   *            ignores obstacles but takes into account the non-holonomic nature of 
   *            the car. This heuristic, which can be completely pre-computed for the 
   *            entire 4D space (vehicle location, and orientation, and direction of 
   *            motion), helps in the end-game by approaching the goal with the desired heading
   * @param h2  holonomic-with-obstacles heuristic
   *            It is calculated online by performing dynamic programming. in 2-D 
   *            (ignoring vehicle orientation and motion direction).
   */
  vector<vector<vector<int>>> h1(
    NUM_THETA_CELLS, vector<vector<int>>(grid[0].size(), vector<int>(grid.size()))); 
  vector<vector<vector<int>>> h2(
    NUM_THETA_CELLS, vector<vector<int>>(grid[0].size(), vector<int>(grid.size())));
  for(int stack=0; stack<NUM_THETA_CELLS; stack++) {
    for(int x=0; x<grid.size(); x++) {
      for(int y=0; y<grid[0].size(); y++) {
        h2[stack][x][y] = 999;
      }
    }
  }  

  //Check for h2 initialization
  int width = 5;
  for(int stack=0; stack<NUM_THETA_CELLS; stack++) {
    for(int x=0; x<grid.size(); x++) {
      for(int y=0; y<grid[0].size(); y++) {
        cout << setw(width) << h2[stack][x][y];
      }
      cout << endl;
    }
    break;
  }  



  //Define h2
  int const cost = 1;

  vector<vector<int>> forward = {
    {-1,  0}, //go up
    { 0, -1}, //go left
    { 1,  0}, //go down
    { 0,  1}}; //go right
  
  bool change = true;
  while(change) {
    change = false;


    //for(int stack=0; stack<NUM_THETA_CELLS; stack++) {
    for(int stack=0; stack<1; stack++) {
      for(int x=0; x<grid.size(); x++) {
        for(int y=0; y<grid[0].size(); y++) {

          //Check whether or not reach the goal
          if((x==goal[0]) && (y==goal[1])) {
            change = true;
            h2[stack][x][y] = 0;
          } 

          else if (grid[x][y] == 0){
            //Check the potential grid in 4 orientations
            for(int i=0; i<forward.size(); i++) {
              int x2 = x + forward[i][0];
              int y2 = y + forward[i][1];

              //Check whether or not x2,y2 is in the grid & no encounter obstcles
              if((x2>=0) && (x2<grid.size()) &&
                 (y2>=0) && (y2<grid[0].size()) &&
                 grid[x2][y2] == 0) {
                int v2 = h2[stack][x][y] + cost;

                //Check whether or not this is conject on the last step
                if(v2<h2[stack][x][y]) {
                  change = true;
                  h2[stack][x][y] = v2;
                }
              }
            }
          } 
        }
      }
    }

    //Check for h2 after assigning the value
    cout<< "-------------------------------------" <<endl;
    for(int stack=0; stack<NUM_THETA_CELLS; stack++) {
      for(int x=0; x<grid.size(); x++) {
        for(int y=0; y<grid[0].size(); y++) {
          cout << setw(width) << h2[stack][x][y];
        }
        cout << endl;
      }
      break;
    } 
  }

  
  //Check for h2 after assigning the value
  cout<< "-------------------------------------" <<endl;
  for(int stack=0; stack<NUM_THETA_CELLS; stack++) {
    for(int x=0; x<grid.size(); x++) {
      for(int y=0; y<grid[0].size(); y++) {
        cout << setw(width) << h2[stack][x][y];
      }
      cout << endl;
    }
    break;
  }  

  

  return h2;                            

}



#endif  // HYBRID_BREADTH_FIRST_H_