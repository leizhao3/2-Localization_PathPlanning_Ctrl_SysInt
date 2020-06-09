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
    double x = 5.990652e+00; //unit: meter
    double y = 1.890657e+00; //unit:meter
    double theta = -1.244077e-02; //unit: radian

    double v = 3.961100e+00; //unit:meter/sec
    double theta_dot = 3.093700e+00; //unit: radian
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
