#include <iostream>
#include <cmath>
#include <iomanip>

void Update(double x, 
              double y,
              double theta,
              double v,
              double theta_dot,
              double dt);

int main(){
    double x = 102; //unit: meter
    double y = 65; //unit:meter
    double theta = 5.0*M_PI/8.0; //unit: radian

    double v = 110; //unit:meter/sec
    double theta_dot = M_PI/8.0; //unit: radian
    double dt = 0.1;

    Update(x, y, theta, v, theta_dot, dt);

    return 0;
}


void Update(double x, 
              double y,
              double theta,
              double v,
              double theta_dot,
              double dt){
          
          double theta_f = theta + theta_dot*dt;
          x = x + v/theta_dot*(sin(theta_f)-sin(theta));
          y = y + v/theta_dot*(cos(theta)-cos(theta_f));
          theta = theta_f;
        
          std::cout<<std::setw(15)<<"x"<<std::setw(15)<<"y"<<std::setw(15)<<"theta"<<std::endl;
          std::cout<<std::setw(15)<< x <<std::setw(15)<< y <<std::setw(15)<< theta <<std::endl;

        }
