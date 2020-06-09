#include <cmath>
#include <iostream>

int main() {
  // define coordinates and theta
  double x_part, y_part, x_obs, y_obs, theta_part;
  x_part = 6.48387;
  y_part = 1.93387;
  theta_part = -M_PI/2; // theta is the yaw angle of the vehicle. in VEHICLE coord.
  x_obs = 2.6615;
  y_obs = 5.7091;


  // transform to map x coordinate
  double x_map;
  x_map = x_part + (cos(theta_part) * x_obs) - (sin(theta_part) * y_obs);

  // transform to map y coordinate
  double y_map;
  y_map = y_part + (sin(theta_part) * x_obs) + (cos(theta_part) * y_obs);

  // (6,3)
  std::cout << x_map << "\t\t " << y_map << std::endl;

  return 0;
}