#ifndef FILTER_H
#define FILTER_H

#include "time_utils.h"
#include "foc_utils.h"

/*
*   Generic Filter Class    
*/

class Filter
{
private:
    /* data */
public:
    Filter();
    ~Filter() = default;
    virtual operator() (float x) = 0;
    
protected:
    unsigned long timestamp_prev;  //!< Last execution timestamp
    float y_prev; //!< filtered value in previous execution step 
};


#endif // FILTER_H