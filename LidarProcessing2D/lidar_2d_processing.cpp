//
// Created by robit on 21. 11. 3..
//

#include "lidar_2d_processing.h"

void ld2::distBandPassFilter(Ranges &ranges, tfloat min, tfloat max)
{
    Ranges::iterator iter;
    for (iter = ranges.begin(); iter != ranges.end(); iter++)
    {
        if(*iter < min) *iter = min;
        else if(max < *iter) *iter = max;
    }
}

void ld2::generateGaussianMask(Ranges &mask, double sigma, int size)
{
    if(size == -1)
    {
        mask.resize(sigma*4+1);
    }
    else
    {
        mask.resize(size);
    }

    Ranges::iterator iter;

    int i;
    double cumulative = 0;
    for (iter = mask.begin(), i =-mask.size()/2+1; iter != mask.end(); iter++, i++)
    {
        tfloat x = i;
        *iter = exp(-x*x/(2*sigma*sigma));
        cumulative += *iter;
    }
    for (iter = mask.begin(), i =-mask.size()/2+1; iter != mask.end(); iter++, i++)
    {
        *iter = *iter / cumulative;
    }
}

void ld2::convolute(const Ranges &ranges, const Ranges &mask, Ranges &output)
{
    output.resize(ranges.size());
    Ranges::const_iterator iterJ;
    tfloat sum;
    int i, j;
    for (i=0; i <= ranges.size() - mask.size(); i++)
    {
        sum = 0;
        for (iterJ = mask.begin(), j=0; iterJ != mask.end(); iterJ++, j++)
        {
            sum += ranges[i + j] * *iterJ;
        }
        output[i + mask.size() / 2] = sum;
    }
    for (i=0; i <= mask.size() / 2; i++)
    {
        output[i] = ranges[i];
    }
    for (i= ranges.size() - mask.size() / 2; i < ranges.size(); i++)
    {
        output[i] = ranges[i];
    }
}

void ld2::diff(const Ranges &input, Ranges &output)
{
    output.resize(input.size());

    output[0] = 0;

    Ranges::const_iterator iter;
    int i;
    for(iter = output.begin()+1, i=1; iter != output.end(); iter++, i++)
    {
        output[i] = input[i] - input[i-1];
    }
}

void ld2::findZeroCrossing(const Ranges &input, Ranges &output, double threshold)
{
    output.resize(input.size());
    Ranges::const_iterator iter;
    int i;
    for(iter = input.begin()+1, i=1; iter != input.end()-1; iter++, i++)
    {
        output[i] = 0;
        if(*(iter-1) * *(iter) >= 0) continue; //same sign -> skip
        if(abs(*(iter-1)) < threshold || abs(*iter) < threshold) continue; //below threshold -> skip
        output[i] = 1;
    }
}

bool ld2::splitRanges(const Ranges &ranges, const Ranges &indicator, vector<Subranges> &splitRanges, int padding)
{
    if(ranges.size() != indicator.size()) return false;

    Ranges::const_iterator iter;

    splitRanges.push_back(Subranges());
    int splitRangesIdx = 0;
    int i;
    for(iter = ranges.begin(), i=0; iter != ranges.end(); iter++, i++)
    {
        if(indicator[i] == 1)
        {
            splitRanges.push_back(Subranges());
            splitRangesIdx++;
            splitRanges[splitRangesIdx].index = i;
        }
        else
        {
            splitRanges[splitRangesIdx].ranges.push_back(*iter);
        }
    }

    // padding
    vector<Subranges>::iterator rIter;
    for(rIter = splitRanges.begin(); rIter != splitRanges.end(); rIter++)
    {
        for(int i=0; i<padding; i++)
        {
            if(rIter->ranges.size() > 2)
            {
                // pop back
                rIter->ranges.pop_back();

                // pop front
                rIter->ranges.erase(rIter->ranges.begin());

                //increase index
                rIter->index++;
            }
            else
            {
                splitRanges.erase(rIter);
                rIter--;
                break;
            }
        }
    }
    return true;
}


bool ld2::assessForObstacle(vector<Subranges> &splitRanges, double openThreshold, const vector<int> &excludeIndexList, int centerIdx, int &startIndex, int &length)
{
    vector<Subranges>::iterator rIter;
    Ranges::const_iterator iter;

    for(rIter = splitRanges.begin(); rIter != splitRanges.end(); rIter++)
    {
        for(iter = rIter->ranges.begin(); iter != rIter->ranges.end(); iter++)
        {
            if(openThreshold < *iter)
            {
                splitRanges.erase(rIter);
                rIter--;
                break;
            }

            vector<int>::const_iterator exIter;
            bool exclude = false;
            for (exIter = excludeIndexList.begin(); exIter != excludeIndexList.end(); exIter++)
            {
                //cout<<"evaluation:"<<rIter->index<<"~"<<rIter->index + rIter->ranges.size()<<" / "<<*exIter<<endl;
                if (rIter->index <= *exIter && *exIter <= rIter->index + rIter->ranges.size())
                {
                    exclude = true;
                    break;
                }
            }

            if(exclude)
            {
                splitRanges.erase(rIter);
                rIter--;
                break;
            }
        }
    }

    if(splitRanges.size() == 0)
    {
        return false;
    }
    else if(splitRanges.size() == 1)
    {
        startIndex = splitRanges[0].index;
        length = splitRanges[0].ranges.size();
        return true;
    }
    else
    {
        Subranges* bestSubranges = &splitRanges[0];

        for(rIter = splitRanges.begin()+1; rIter != splitRanges.end(); rIter++)
        {
            if(abs(int(bestSubranges->index+bestSubranges->ranges.size()/2 - centerIdx)) > abs(int(rIter->index+rIter->ranges.size()/2 - centerIdx)))
            {
                bestSubranges = &(*rIter);
            }
        }

        startIndex = bestSubranges->index;
        length = bestSubranges->ranges.size();
        return true;
    }
}

void ld2::hysteresisFilter(const Ranges &input, Ranges &output, double threshold)
{
    output.resize(input.size());
    Ranges::const_iterator iter;
    output[0] = input[0];
    int i;
    for(iter = input.begin()+1, i=1; iter != input.end(); iter++, i++)
    {
        // determine continuity
        if(abs(*iter - *(iter-1)) < threshold)
        {
            output[i] = *iter;
        }
        else
        {
            output[i] = tfloat (1/0.0);//inf
        }
    }
}

void ld2::printRanges(const Ranges &ranges)
{
    Ranges::const_iterator iter;

    cout<<"[";
    for(iter = ranges.begin(); iter != ranges.end(); iter++)
    {
        cout<<*iter<<", ";
    }
    cout<<"]"<<endl;
}




