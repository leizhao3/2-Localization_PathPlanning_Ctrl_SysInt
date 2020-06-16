#ifndef CLASSIFIER_H
#define CLASSIFIER_H

#include <string>
#include <vector>
#include "Eigen/Dense"

using Eigen::ArrayXd;
using std::string;
using std::vector;

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
  
  ArrayXd left_means;
  ArrayXd left_sds;
  double left_prior;
  
  ArrayXd keep_means;
  ArrayXd keep_sds;
  double keep_prior;
  
  ArrayXd right_means;
  ArrayXd right_sds;
  double right_prior;

};

#endif  // CLASSIFIER_H

/*----------------------------START OF .cpp----------------------------*/

#include <math.h>
#include <string>
#include <vector>
#include <iostream>
#include <iomanip>

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
  left_means = ArrayXd(4);
  left_means << 0,0,0,0;
  
  left_sds = ArrayXd(4);
  left_sds << 0,0,0,0;
    
  left_prior = 0;
    
  keep_means = ArrayXd(4);
  keep_means << 0,0,0,0;
  
  keep_sds = ArrayXd(4);
  keep_sds << 0,0,0,0;
  
  keep_prior = 0;
  
  right_means = ArrayXd(4);
  right_means << 0,0,0,0;
  
  right_sds = ArrayXd(4);
  right_sds << 0,0,0,0;
  
  right_prior = 0;
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
  
  // For each label, compute ArrayXd of means, one for each data class 
  //   (s, d, s_dot, d_dot).
  // These will be used later to provide distributions for conditional 
  //   probabilites.
  // Means are stored in an ArrayXd of size 4.
  
  float left_size = 0;
  float keep_size = 0;
  float right_size = 0;
  
  // For each label, compute the numerators of the means for each class
  //   and the total number of data points given with that label.
  for (int i=0; i<labels.size(); ++i) {
    if (labels[i] == "left") {
      // conversion of data[i] to ArrayXd
      left_means += ArrayXd::Map(data[i].data(), data[i].size());
      /**
       * @param ArrayXd::Map(dataPtr, size)
       *    data[i].data(): .data() return a pointer to the first element in the array used internally by the vector.
       */
      left_size += 1;
    } else if (labels[i] == "keep") {
      keep_means += ArrayXd::Map(data[i].data(), data[i].size());
      keep_size += 1;
    } else if (labels[i] == "right") {
      right_means += ArrayXd::Map(data[i].data(), data[i].size());
      right_size += 1;
    }
  }

  // Compute the means. Each result is a ArrayXd of means 
  //   (4 means, one for each class)
  left_means = left_means/left_size;
  keep_means = keep_means/keep_size;
  right_means = right_means/right_size;
  std::cout << "left_means = \n" << left_means << std::endl;
  std::cout << "keep_means = \n" << keep_means << std::endl;
  std::cout << "right_means = \n" << right_means << std::endl;

  
  // Begin computation of standard deviations for each class/label combination.
  ArrayXd data_point;
  
  // Compute numerators of the standard deviations.
  for (int i=0; i<labels.size(); ++i) {
    data_point = ArrayXd::Map(data[i].data(), data[i].size());
    if (labels[i] == "left"){
      left_sds += (data_point - left_means)*(data_point - left_means);
    } else if (labels[i] == "keep") {
      keep_sds += (data_point - keep_means)*(data_point - keep_means);
    } else if (labels[i] == "right") {
      right_sds += (data_point - right_means)*(data_point - right_means);
    }
  }
  
  // compute standard deviations
  left_sds = (left_sds/left_size).sqrt();
  keep_sds = (keep_sds/keep_size).sqrt();
  right_sds = (right_sds/right_size).sqrt();
  std::cout << "left_sds = \n" << left_sds << std::endl;
  std::cout << "keep_sds = \n" << keep_sds << std::endl;
  std::cout << "right_sds = \n" << right_sds << std::endl;
    
  //Compute the probability of each label
  left_prior = left_size/labels.size();
  keep_prior = keep_size/labels.size();
  right_prior = right_size/labels.size();
  std::cout << "left_prior = \n" << left_prior << std::endl;
  std::cout << "keep_prior = \n" << keep_prior << std::endl;
  std::cout << "right_prior = \n" << right_prior << std::endl;
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
  
  // Calculate product of conditional probabilities for each label.
  double left_p = 1.0;
  double keep_p = 1.0;
  double right_p = 1.0; 
  int width = 10;

  std::cout <<setw(width)<<"LKR_p "
              <<setw(width)<<"LKR_sds[i]"
              <<setw(width)<<"sample[i]"
              <<setw(width)<<"LKR_means[i]"
              <<endl;

  for (int i=0; i<4; ++i) {
    std::cout << "---------------------" << endl;
    std::cout << "i = " << i << endl;
    left_p *= (1.0/sqrt(2.0 * M_PI * pow(left_sds[i], 2))) 
            * exp(-0.5*pow(sample[i] - left_means[i], 2)/pow(left_sds[i], 2));
    std::cout <<setw(width)<<left_p 
              <<setw(width)<<left_sds[i]
              <<setw(width)<<sample[i]
              <<setw(width)<<left_means[i]
              <<endl;

    keep_p *= (1.0/sqrt(2.0 * M_PI * pow(keep_sds[i], 2)))
            * exp(-0.5*pow(sample[i] - keep_means[i], 2)/pow(keep_sds[i], 2));
    std::cout <<setw(width)<<keep_p 
              <<setw(width)<<keep_sds[i]
              <<setw(width)<<sample[i]
              <<setw(width)<<keep_means[i]
              <<endl;

    right_p *= (1.0/sqrt(2.0 * M_PI * pow(right_sds[i], 2))) 
            * exp(-0.5*pow(sample[i] - right_means[i], 2)/pow(right_sds[i], 2));
    std::cout <<setw(width)<<right_p 
              <<setw(width)<<right_sds[i]
              <<setw(width)<<sample[i]
              <<setw(width)<<right_means[i]
              <<endl;
  }

  // Multiply each by the prior
  left_p *= left_prior;
  keep_p *= keep_prior;
  right_p *= right_prior;
    
  double probs[3] = {left_p, keep_p, right_p};
  double max = left_p;
  double max_index = 0;

  for (int i=1; i<3; ++i) {
    if (probs[i] > max) {
      max = probs[i];
      max_index = i;
    }
  }

  std::cout << "y_output = " << std::endl;
  for(int i=0; i<3; i++) {
    std::cout << probs[i] << " ";
  }
  
  return this -> possible_labels[max_index];
}