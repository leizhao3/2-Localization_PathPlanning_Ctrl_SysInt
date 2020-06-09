#include <iostream>
#include <iomanip>
#include <string>
#include <vector>


struct Particle {
  int id;
  double x;//unit: meter, in MAP coordinate
  double y;//unit: meter, in MAP coordinate
  double theta;//unit: radian, in MAP coordinate
  double weight;
  std::vector<int> associations;
  std::vector<double> sense_x;
  std::vector<double> sense_y;
};


class ParticleFilter {
    public:
        // Set of current particles
        std::vector<Particle> particles;

        void printParticle();
        void printParticle(Particle particle);
        //void setParticle(std::vector<Particle> particles, double x, double y, double theta);
        void setParticle(double x, double y, double theta);

    private:
        // Number of particles to draw
        int num_particles; 
        
};

void ParticleFilter::printParticle(){
    std::cout << "in printParticle func" << std::endl;
    for(int i=0; i<particles.size(); i++){
        std::cout << i << "\t" 
                  << particles[i].x << "\t" 
                  << particles[i].y << "\t" 
                  << particles[i].theta << "\t" 
                  <<std::endl;
    }
}

void ParticleFilter::printParticle(Particle particle){
    std::cout << particle.x << "\t" 
              << particle.y << "\t" 
              << particle.theta << "\t" 
              <<std::endl;
    
}

//void ParticleFilter::setParticle(std::vector<Particle> particles, double x, double y, double theta){
void ParticleFilter::setParticle(double x, double y, double theta){
    num_particles = 4;
    Particle particle;

    for(int i=0; i<num_particles; i++){
        particle.x = x;
        particle.y = y;
        particle.theta = theta;

        printParticle(particle);

        particles.push_back(particle);
    }
}

int main() {
    ParticleFilter pf;

    pf.setParticle(2, 3, 0.3);

    std::cout << "in main func" << std::endl;
    for(int i=0; i<pf.particles.size(); i++){
        std::cout << i << "\t" 
                  << pf.particles[i].x << "\t" 
                  << pf.particles[i].y << "\t" 
                  << pf.particles[i].theta << "\t" 
                  <<std::endl;
    }

    pf.printParticle();

    return 0;
}


