//
// Created by robit on 21. 11. 3..
//

#ifndef MISSION_CONTROL_LIDAR_2D_PROCESSING_H
#define MISSION_CONTROL_LIDAR_2D_PROCESSING_H

#define PI 3.14159265359

#include <iostream>
#include <math.h>
#include <vector>

using namespace std;

namespace ld2
{
using tfloat = float;
using Ranges = vector<tfloat>;

struct Subranges
{
    int index;
    Ranges ranges;
};

void distBandPassFilter(Ranges &ranges, tfloat min, tfloat max);

void generateGaussianMask(Ranges &mask, double sigma,int size = -1);
//when size==-1 , it automatically decides size to be 4*sigma

void convolute(const Ranges &ranges, const Ranges &mask, Ranges &output);

void diff(const Ranges &input, Ranges& output);

void findZeroCrossing(const Ranges &input, Ranges &output, double threshold = 0.2);

bool splitRanges(const Ranges &ranges, const Ranges &indicator, vector<Subranges> &splitRanges, int padding = 0);

bool assessForObstacle(vector<Subranges> &splitRanges,  double openThreshold, const vector<int> &excludeIndexList, int centerIdx, int &startIndex, int &length);

void hysteresisFilter(const Ranges &input, Ranges &output, double threshold);

void printRanges(const Ranges &ranges);
}

#endif //MISSION_CONTROL_LIDAR_2D_PROCESSING_H
