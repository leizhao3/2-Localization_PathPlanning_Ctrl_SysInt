//TOBS1 = (6,3)     TOBS2 = (2,2)   TOBS3 = (0,5).
//mu1=L1= (5,3)     mu2=L2= (2,1)   mu3=L5= (4,7).

#include <iostream>
#include <cmath>
#include <vector>

using std::vector;
using std::cout;
using std::endl;

int main(){
    vector< vector<double> > obs = {{6.0,3.0},
                                    {2.0,2.0},
                                    {0.0,5.0}};
    
    vector< vector<double> > mu = {{5.0,3.0},
                                   {2.0,1.0},
                                   {4.0,7.0}};
    
    const double sigma_x = 0.3;
    const double sigma_y = 0.3;

    vector<double> posterior;
    double p_Final = 1;

    for(int i=0; i<obs.size(); i++){
        double x = obs[i][0];
        double y = obs[i][1];

        double mu_x = mu[i][0];
        double mu_y = mu[i][1];

        double exponent = exp(-(pow((x-mu_x),2)/(2*pow(sigma_x,2))+
                            pow((y-mu_y),2)/(2*pow(sigma_x,2))));

        posterior.push_back(1/(2*M_PI*sigma_x*sigma_y) * exponent);
        cout<<"P is " <<std::scientific <<posterior[i] <<endl;

        p_Final *= posterior[i];
        
    }

    cout<<"P_Final is " <<std::scientific <<p_Final <<endl;

    return 0;
}
