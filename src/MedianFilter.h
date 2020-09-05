/*
 * MedianFilter.h
 *
 *  Created on: May 19, 2018
 *      Author: alexandru.bogdan
 */

#ifndef MEDIANFILTER_H_
#define MEDIANFILTER_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct sMedianNode
{
    double value;                   //sample value
    struct sMedianNode *nextAge;    //pointer to next oldest value
    struct sMedianNode *nextValue;  //pointer to next smallest value
    struct sMedianNode *prevValue;  //pointer to previous smallest value
}sMedianNode_t;

typedef struct
{
    unsigned int numNodes;          //median node buffer length
    sMedianNode_t *medianBuffer;    //median node buffer
    sMedianNode_t *ageHead;         //pointer to oldest value
    sMedianNode_t *valueHead;       //pointer to smallest value
    sMedianNode_t *medianHead;      //pointer to median value
}sMedianFilter_t;

int MEDIANFILTER_Init(sMedianFilter_t *medianFilter);
double MEDIANFILTER_Insert(sMedianFilter_t *medianFilter, double sample);

#ifdef __cplusplus
}
#endif
#endif
