#include <cmath>
#include <iostream>
#include <vector>

#include "Eigen/Dense"
#include "L10_27_QuinticPolynomialSolver_grader.h"

using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * TODO: complete this function
 */
vector<double> JMT(vector<double> &start, vector<double> &end, double T) {
  /**
   * Calculate the Jerk Minimizing Trajectory that connects the initial state
   * to the final state in time T.
   *
   * @param start - the vehicles start location given as a length three array
   *   corresponding to initial values of [s, s_dot, s_double_dot]
   * @param end - the desired end state for vehicle. Like "start" this is a
   *   length three array.
   * @param T - The duration, in seconds, over which this maneuver should occur.
   *
   * @output an array of length 6, each value corresponding to a coefficent in 
   *   the polynomial:
   *   s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
   *
   * EXAMPLE
   *   > JMT([0, 10, 0], [10, 10, 0], 1)
   *     [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
   */
  vector<double> alpha = {};
  
  alpha.push_back(start[0]);
  alpha.push_back(start[1]);
  alpha.push_back(start[2]/2.0);

  double C0 = start[0] + start[1]*T + 1/2.0*start[2]*pow(T,2.0);
  double C1 = start[1] + start[2]*T;
  double C2 = start[2];

  MatrixXd timeMat(3,3);
  timeMat <<  pow(T,3),     pow(T,4),     pow(T,5),
            3*pow(T,2),   4*pow(T,3),   5*pow(T,4),  
            6*pow(T,1),  12*pow(T,2),  20*pow(T,3);  
  
  MatrixXd sfMat(3,1);
  sfMat <<  end[0]-C0,
            end[1]-C1,
            end[2]-C2;

  MatrixXd alphaMat(3,1);
  alphaMat << timeMat.inverse() * sfMat;

  for(int i=0; i<alphaMat.size(); i++) {
      alpha.push_back(alphaMat.data()[i]);
  }



  return alpha;
}

int main() {

  // create test cases
  vector< test_case > tc = create_tests();

  bool total_correct = true;

  for(int i = 0; i < tc.size(); ++i) {
    vector<double> jmt = JMT(tc[i].start, tc[i].end, tc[i].T);
    bool correct = close_enough(jmt, answers[i]);
    total_correct &= correct;
  }

  if(!total_correct) {
    std::cout << "Try again!" << std::endl;
  } else {
    std::cout << "Nice work!" << std::endl;
  }

  return 0;
}