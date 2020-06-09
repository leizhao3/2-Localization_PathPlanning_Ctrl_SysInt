#include <iostream>
#include <vector>
#include <random>
#include <map>

using std::vector;

int main()
{
    vector<double> weight_Array = {10,0.002};
    /*
    for(int i; i<10; i++) {
        weight_Array.push_back(0.022);
    }*/
    weight_Array[5] = 0.3;

    std::cout << "weight_Array is " << std::endl;
    for(int i; i<weight_Array.size(); i++) {
        std::cout << weight_Array[i] << std::endl;
    }

    
    //std::discrete_distribution<> d({30, 20, 90, 20, 20});
    std::discrete_distribution<> d(weight_Array.begin(),weight_Array.end());
    std::vector<double> p = d.probabilities();
    int max = d.max();
    int min = d.min();

    std::cout << "d.probabilities()" << std::endl;
    for(auto n : p)
        std::cout << n << ' ';
    std::cout << '\n';

    std::cout << "max\t\tmin" << std::endl;
    std::cout << max <<"\t\t" << min << std::endl;

    //std::cout << d.operator() << std::endl;

    
    std::random_device rd;
    std::mt19937 gen(rd());
    for(int i=0; i< 10; i++) {
        std::cout << "random number generated " << d(gen) << std::endl;
    }

    /*
    std::discrete_distribution<> d({40, 10, 10, 40});
    std::map<int, int> m;
    for(int n=0; n<10000; ++n) {
        ++m[d(gen)];
    }
    for(auto p : m) {
        std::cout << p.first << " generated " << p.second << " times\n";
    }
    */
}