#ifndef LINEAR_H_
#define LINEAR_H_

#include "utils.h"

/// the relationship of homography
#define HOMOGRAPHY 0    
/// the relationship of fundamental matrix
#define FUND_MAT 1      
/// the relationship of camera projection matrix
#define CAM_PROJ_MAT 2

void findHomography(CvMat *x1[], CvMat *x2[], int nbCorr, CvMat *H);

void denormalize(CvMat *x1[], CvMat *x2[], int nbCorr, CvMat *LT,
                 CvMat *ST1, CvMat *ST2, int relationship);

void DLTAlgorithm(CvMat *A, CvMat *X);

void normalize(CvMat *x1[], CvMat *x2[], int nbCorr, CvMat *ST1, CvMat *ST2);

#endif