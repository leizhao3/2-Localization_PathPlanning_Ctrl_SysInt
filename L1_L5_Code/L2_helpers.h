#ifndef HELP_FUNCTIONS_H_
#define HELP_FUNCTIONS_H_

#include <math.h>
#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <iomanip>

using namespace std;

float normpdf(float x, float mu, float std);

const float STATIC_ONE_OVER_SQRT_2PI = 0.398942280401432677939946059934381868475858631164934657665;
float normpdf(float x, float mu, float std){
  return (STATIC_ONE_OVER_SQRT_2PI/std)*exp(-0.5*pow((x-mu)/std,2));
}



class Helpers {
public:
//static function to normalize a vector:
	static std::vector<float> normalize_vector(std::vector<float> inputVector){

		//declare sum:
		float sum = 0.0f;

		//declare and resize output vector:
		std::vector<float> outputVector ;
		outputVector.resize(inputVector.size());

		//estimate the sum:
		for (unsigned int i = 0; i < inputVector.size(); ++i) {
			sum += inputVector[i];
		}

		//normalize with sum:
		for (unsigned int i = 0; i < inputVector.size(); ++i) {
			outputVector[i] = inputVector[i]/sum;
		}

		//return normalized vector:
		return outputVector;
	}
	
};

#endif /* HELP_FUNCTIONS_H_ */