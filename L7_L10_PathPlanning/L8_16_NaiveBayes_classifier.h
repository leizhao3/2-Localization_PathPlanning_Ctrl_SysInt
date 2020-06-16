#ifndef CLASSIFIER_H
#define CLASSIFIER_H

#include <string>
#include <vector>
#include "Eigen/Dense"

using Eigen::ArrayXd;
using std::string;
using std::vector;

vector<double> data_Analysis(const vector<double> &data);

struct State {

  vector<string> labels; ///{"left","keep","right"};
  vector<vector<double>> s; //{{mu, sigma}_left,{mu, sigma}_keep,{mu, sigma}_right}
  vector<vector<double>> d; //{{mu, sigma}_left,{mu, sigma}_keep,{mu, sigma}_right}
  vector<vector<double>> sdot; //{{mu, sigma}_left,{mu, sigma}_keep,{mu, sigma}_right}
  vector<vector<double>> ddot; //{{mu, sigma}_left,{mu, sigma}_keep,{mu, sigma}_right}
  
};

class GNB {
 public:
  /**
   * Constructor
   */
  GNB();

  /**
   * Destructor
   */
  virtual ~GNB();

  /**
   * Train classifier
   */
  void train(const vector<vector<double>> &data, 
             const vector<string> &labels);

  /**
   * Predict with trained classifier
   */
  string predict(const vector<double> &sample);

  vector<string> possible_labels = {"left","keep","right"};

  vector<double> p_Ck;

  const double lane_width = 4.0;

  State state;
};

#endif  // CLASSIFIER_H

/*----------------------------START OF .cpp----------------------------*/

#include <math.h>
#include <string>
#include <vector>
#include <iomanip>
#include <iostream>

using Eigen::ArrayXd;
using std::string;
using std::vector;
using std::cout;
using std::endl;
using std::setw;

// Initializes GNB
GNB::GNB() {
  /**
   * TODO: Initialize GNB, if necessary. May depend on your implementation.
   */

  state.labels = {"left","keep","right"};
  state.s = {}; //{{mu, sigma}_left,{mu, sigma}_keep,{mu, sigma}_right}
  state.d = {}; //{{mu, sigma}_left,{mu, sigma}_keep,{mu, sigma}_right}
  state.sdot = {}; //{{mu, sigma}_left,{mu, sigma}_keep,{mu, sigma}_right}
  state.ddot = {}; //{{mu, sigma}_left,{mu, sigma}_keep,{mu, sigma}_right}
  
}

GNB::~GNB() {}

void GNB::train(const vector<vector<double>> &data, 
                const vector<string> &labels) {
  /**
   * Trains the classifier with N data points and labels.
   * @param data - array of N observations
   *   - Each observation is a tuple with 4 values: s, d, s_dot and d_dot.
   *   - Example : [[3.5, 0.1, 5.9, -0.02],
   *                [8.0, -0.3, 3.0, 2.2],
   *                 ...
   *                ]
   * @param labels - array of N labels
   *   - Each label is one of "left", "keep", or "right".
   *
   * TODO: Implement the training function for your classifier.
   */

  vector<double> left_s_={}, left_d_={}, left_sdot_={}, left_ddot_={};
  vector<double> keep_s_={}, keep_d_={}, keep_sdot_={}, keep_ddot_={};
  vector<double> right_s_={}, right_d_={}, right_sdot_={}, right_ddot_={};



  for(int i=0; i<data.size(); i++) {
    if(labels[i].compare(state.labels[0]) == 0) {
      left_s_.push_back(data[i][0]);
      left_d_.push_back(data[i][1]);
      //left_d_.push_back(fmod(data[i][1],lane_width));
      left_sdot_.push_back(data[i][2]);
      left_ddot_.push_back(data[i][3]);
    }

    if(labels[i].compare(state.labels[1]) == 0) {
      keep_s_.push_back(data[i][0]);
      keep_d_.push_back(data[i][1]);
      //keep_d_.push_back(fmod(data[i][1],lane_width));
      keep_sdot_.push_back(data[i][2]);
      keep_ddot_.push_back(data[i][3]);
    }

    if(labels[i].compare(state.labels[2]) == 0) {
      right_s_.push_back(data[i][0]);
      right_d_.push_back(data[i][1]);
      //right_d_.push_back(fmod(data[i][1],lane_width));
      right_sdot_.push_back(data[i][2]);
      right_ddot_.push_back(data[i][3]);
    }
  }

  

  //Analyze x for label = left
  vector<double> left_s_mu_sigma_ = data_Analysis(left_s_);
  vector<double> left_d_mu_sigma_ = data_Analysis(left_d_);
  vector<double> left_sdot_mu_sigma_ = data_Analysis(left_sdot_);
  vector<double> left_ddot_mu_sigma_ = data_Analysis(left_ddot_);
  /*
  cout << "left = s\t d\t sdot\t ddot\t" << endl;
  cout << "  mu = " << left_s_mu_sigma_[0]<<"  "
                    << left_d_mu_sigma_[0]<<"  " 
                    << left_sdot_mu_sigma_[0]<<"  " 
                    << left_ddot_mu_sigma_[0]<<"  " 
                    << endl;
  cout << "  sigma = " << left_s_mu_sigma_[1]<<"  "
                       << left_d_mu_sigma_[1]<<"  " 
                       << left_sdot_mu_sigma_[1]<<"  " 
                       << left_ddot_mu_sigma_[1]<<"  " 
                       << endl;*/

  state.s.push_back(left_s_mu_sigma_);
  state.d.push_back(left_d_mu_sigma_);
  state.sdot.push_back(left_sdot_mu_sigma_);
  state.ddot.push_back(left_ddot_mu_sigma_);
  p_Ck.push_back(1.0*left_s_.size()/data.size());

  //Analyze x for label = keep
  vector<double> keep_s_mu_sigma_ = data_Analysis(keep_s_);
  vector<double> keep_d_mu_sigma_ = data_Analysis(keep_d_);
  vector<double> keep_sdot_mu_sigma_ = data_Analysis(keep_sdot_);
  vector<double> keep_ddot_mu_sigma_ = data_Analysis(keep_ddot_);
  /*
  cout << "keep = s\t d\t sdot\t ddot\t" << endl;
  cout << "  mu = " << keep_s_mu_sigma_[0]<<"  "
                    << keep_d_mu_sigma_[0]<<"  " 
                    << keep_sdot_mu_sigma_[0]<<"  " 
                    << keep_ddot_mu_sigma_[0]<<"  " 
                    << endl;
  cout << "  sigma = " << keep_s_mu_sigma_[1]<<"  "
                       << keep_d_mu_sigma_[1]<<"  " 
                       << keep_sdot_mu_sigma_[1]<<"  " 
                       << keep_ddot_mu_sigma_[1]<<"  " 
                       << endl;*/

  state.s.push_back(keep_s_mu_sigma_);
  state.d.push_back(keep_d_mu_sigma_);
  state.sdot.push_back(keep_sdot_mu_sigma_);
  state.ddot.push_back(keep_ddot_mu_sigma_);
  p_Ck.push_back(1.0*keep_s_.size()/data.size());

  //Analyze x for label = right
  vector<double> right_s_mu_sigma_ = data_Analysis(right_s_);
  vector<double> right_d_mu_sigma_ = data_Analysis(right_d_);
  vector<double> right_sdot_mu_sigma_ = data_Analysis(right_sdot_);
  vector<double> right_ddot_mu_sigma_ = data_Analysis(right_ddot_);
  /*
  cout << "right = s\t d\t sdot\t ddot\t" << endl;
  cout << "  mu = " << right_s_mu_sigma_[0]<<"  "
                    << right_d_mu_sigma_[0]<<"  " 
                    << right_sdot_mu_sigma_[0]<<"  " 
                    << right_ddot_mu_sigma_[0]<<"  " 
                    << endl;
  cout << "  sigma = " << right_s_mu_sigma_[1]<<"  "
                       << right_d_mu_sigma_[1]<<"  " 
                       << right_sdot_mu_sigma_[1]<<"  " 
                       << right_ddot_mu_sigma_[1]<<"  " 
                       << endl;*/

  state.s.push_back(right_s_mu_sigma_);
  state.d.push_back(right_d_mu_sigma_);
  state.sdot.push_back(right_sdot_mu_sigma_);
  state.ddot.push_back(right_ddot_mu_sigma_);
  p_Ck.push_back(1.0*right_s_.size()/data.size());

  /*
  int width = 10;
  cout<< setw(width) << "label" << setw(width) << "p_Ck" << endl;
  for(int i=0; i<p_Ck.size(); i++) {
    cout<< setw(width) << state.labels[i] << setw(width) << p_Ck[i] << endl;
  }*/


  
  
}

string GNB::predict(const vector<double> &sample) {
  /**
   * Once trained, this method is called and expected to return 
   *   a predicted behavior for the given observation.
   * @param observation - a 4 tuple with s, d, s_dot, d_dot.
   *   - Example: [3.5, 0.1, 8.5, -0.2]
   * @output A label representing the best guess of the classifier. Can
   *   be one of "left", "keep" or "right".
   *
   * TODO: Complete this function to return your classifier's prediction
   */

  int width = 10;
  double mu;
  double sigma;
  double sigma2;
  double v;
  vector<double> y_output = {}; //store possibility of each label

  /*
  std::cout <<setw(width)<<"LKR_p "
              <<setw(width)<<"LKR_sds[i]"
              <<setw(width)<<"sample[i]"
              <<setw(width)<<"LKR_means[i]"
              <<endl;*/

  for(int j=0; j<possible_labels.size(); j++) {
    double p_v_C = 1.0; 
    //std::cout << "---------------------" << endl;
    //std::cout << "j = " << j << endl;

    //Calculate PI_p_v_C
    for(int i=0; i<sample.size(); i++) {
      if(i==0) { //for s
        mu = state.s[j][0];
        sigma = state.s[j][1];
        sigma2 = sigma*sigma;
        v = sample[i];
      }
      if(i==1) { //for d
        mu = state.d[j][0];
        sigma = state.d[j][1];
        sigma2 = sigma*sigma;
        v = sample[i];
        //v = fmod(sample[i], lane_width);
      }
      if(i==2) { //for sdot
        mu = state.sdot[j][0];
        sigma = state.sdot[j][1];
        sigma2 = sigma*sigma;
        v = sample[i];
      }
      if(i==3) { //for ddot
        mu = state.ddot[j][0];
        sigma = state.ddot[j][1];
        sigma2 = sigma*sigma;
        v = sample[i];
      }


      //double v = sample[i];
      p_v_C *= 1.0/sqrt(2.0 * M_PI * sigma2) * exp(-(v-mu)*(v-mu)/(2.0*sigma2));

      /*
      std::cout <<setw(width)<<p_v_C 
              <<setw(width)<<sigma
              <<setw(width)<<v
              <<setw(width)<<mu
              <<endl;*/

    }

    //Calculate y for each label
    double y = p_Ck[j]*p_v_C;

    y_output.push_back(y);

  }

  /*
  cout << "y_output = " << endl;
  for(int i=0; i<y_output.size(); i++) {
    cout << y_output[i] << " ";
  }*/

  //Find the maximum of y
  double y_max = 0.0;
  int y_max_idx;
  for(int i=0; i<y_output.size(); i++) { 
    if(y_output[i] > y_max) {
      y_max = y_output[i];
      y_max_idx = i;
    }
  }

  
  return this -> possible_labels[y_max_idx];
}


vector<double> data_Analysis(const vector<double> &data) {
  /**
   * Return the mu & sigma of the data
   */

  vector<double> mu_sigma = {};
  double mu;
  double sigma;
  int N = data.size(); //number of data in the data set

  //Calculate the mu
  double sum = 0.0;
  for(int i=0; i<N; i++) {
    sum += data[i];
  }
  mu = sum/N;
  mu_sigma.push_back(mu);

  //Calculate sigma
  double sum2 = 0.0;
  for(int i=0; i<N; i++) {
    double x_mu = data[i]-mu;
    sum2 += x_mu*x_mu;
  }
  sigma = sqrt(sum2/N);
  mu_sigma.push_back(sigma);

  return mu_sigma;


}