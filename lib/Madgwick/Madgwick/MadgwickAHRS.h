#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h

#include <math.h>

class Madgwick {
public:
    Madgwick();
    void begin(float sampleFreq);
    void update(float gx, float gy, float gz, float ax, float ay, float az);
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);
    void setGain(float beta);
    float q0, q1, q2, q3;

private:
    float beta;  // algorithm gain
    float invSampleFreq;
};

#endif