#ifndef MATCHING_H_
#define MATCHING_H_

#include <time.h>
#include "data_structures.h"
#include "utils.h"
#include "nonlinear.h"
#include "linear.h"
#include "sift.h"

/// constant used in the Harris corner detector (set for accurate det.)
#define CORNERNESS_THRESH 10 
/// constant used in the Harris corner detector
#define HARRIS_WEIGHT 0.04
/// i.e. the maximal reprojection error is 1.5 pixel
#define REPROJ_ERROR_THRESH 1.5

void findCorners(Frame *frame, double t = CORNERNESS_THRESH, double k = HARRIS_WEIGHT,
                 int localMaxWindowRadius = 5);

void findCorrespondences(Frame *frame, CvMat *H = NULL, double SSDMax = 100,
                         double maxDist = REPROJ_ERROR_THRESH,
                         int proximWindowRadius = 50, int neighbWindowRadius = 10);

int RANSAC(Frame *frame, double t = REPROJ_ERROR_THRESH);
int optimHGuidMatchCycle(Frame *frame,int featureExtractor);

#endif