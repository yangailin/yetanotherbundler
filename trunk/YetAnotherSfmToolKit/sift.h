#ifndef SIFT_H_
#define SIFT_H_

#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "data_structures.h"
#include "nonlinear.h"

/// i.e. the maximal reprojection error is 1.5 pixel
#define REPROJ_ERROR_THRESH 1.5

/*---------------------------- Structures --------------------------------*/


void matchSIFT(Frame *frame, CvMat *H = NULL,float proximWindowRadius=50.0f,double maxDist=REPROJ_ERROR_THRESH);
void findSIFT(Frame *frame, char* keyFileName);
Keypoint ReadKeyFile(char *filename);
Keypoint CheckForMatch(Keypoint key, Keypoint klist);
int DistSquared(Keypoint k1, Keypoint k2);
void FatalError(char *fmt, ...);
Keypoint ReadKeys(FILE *fp);

#endif