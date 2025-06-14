#include "MadgwickAHRS.h"

Madgwick::Madgwick() {
    beta = 0.1f;
    invSampleFreq = 1.0f / 512.0f;
    q0 = 1.0f;
    q1 = q2 = q3 = 0.0f;
}

void Madgwick::begin(float sampleFreq) {
    invSampleFreq = 1.0f / sampleFreq;
}

void Madgwick::setGain(float b) {
    beta = b;
}

void Madgwick::updateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
    // 簡略化されたダミー実装
}