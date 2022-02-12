#include <cstddef>

class MovingWindowAverage {
public:
  float sum;
  size_t nbSamples;

  void addSample(float newSample){
    sum+=newSample;
    nbSamples++;
  }

  float getAverage(){
    return sum/nbSamples;
  }

  void reset(){
    sum=0.0;
    nbSamples=0;
  }
};

void configureMqttAugmentation() {
}

