#include "median_filter.h"

//#include <iostream>


MedianFilter::MedianFilter()
{
    for (int i = 0; i < NUM_SAMPLES; i++)
    {
        // initialize steps since addition to zero measurements with increasing indexes for addition time
        // this way we will flush them sequentially out of the arrays as real data comes in
        steps_since_addition[i] = i;
        sorted_measurements[i] = 0;
    }
    
}

void MedianFilter::init(float start_angle){
    for (int i = 0; i < NUM_SAMPLES; i++)
    {
        sorted_measurements[i] = start_angle;
    }
    
}

float MedianFilter::operator() (float x)
{
    // find the element which will be replaced and omit it in a temporary array
    int temp_ssa[NUM_SAMPLES-1];
    float temp_sm[NUM_SAMPLES-1];
    int j = 0; // index for reduced arrays
    for (int i = 0; i < NUM_SAMPLES; i++)
    {
        if (steps_since_addition[i]==NUM_SAMPLES-1)
        {
            //std::cout << "drop entry " << i << std::endl;
            // do nothing for this index i
        } else {
            temp_ssa[j] = steps_since_addition[i];
            temp_sm[j] = sorted_measurements[i];
            j++;
        }
    }

    // find the insertion site in the temporary sorted array of measurements, there are NUM_SAMPLES possible positions
    int index_insert = NUM_SAMPLES-1; // if this value is not modified we would insert at the end of the sorted array
    for (int i = 0; i < NUM_SAMPLES-1; i++)
    {
        if (x<temp_sm[i])
        {
            index_insert = i;
            break;
        }
    }
    //std::cout << "insert at " << index_insert << std::endl;


    // reconstruct the updated arrays from the temporary arrays and the new measurement
    j = 0; //reset auxillary index
    for (int i = 0; i < NUM_SAMPLES; i++)
    {
        if (i==index_insert)
        {
            steps_since_addition[i] = 0;
            sorted_measurements[i] = x;
        } else {
            steps_since_addition[i] = temp_ssa[j] + 1; // increase steps since last addition by one
            sorted_measurements[i] = temp_sm[j];
            j++; // increase auxillary counter for temporary array
        }
        
    }
    
    float median;
    if (NUM_SAMPLES%2==0)
    { //even number of elements
        median = (sorted_measurements[NUM_SAMPLES/2-1] + sorted_measurements[NUM_SAMPLES / 2]) / 2.0;
    } else { //odd number of elements
        median = sorted_measurements[NUM_SAMPLES/2];
    }

    return median;

}