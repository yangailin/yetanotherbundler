#ifndef NONLINEAR_H_
#define NONLINEAR_H_

#include "lm.h"
#include "sba.h"
#include "utils.h"

double findDistance(CvMat *x1, CvMat *x2, CvMat *H);
void optimizeHomography(CvMat *H, CvMat *x1[], CvMat *x2[], int nbCorr);

#endif