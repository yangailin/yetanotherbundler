#ifndef SIFT_H_
#define SIFT_H_

#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "data_structures.h"

/*---------------------------- Structures --------------------------------*/


void matchSIFT(Frame *frame, CvMat *H = NULL);
void findSIFT(Frame *frame, char* keyFileName);
Keypoint ReadKeyFile(char *filename);
Keypoint CheckForMatch(Keypoint key, Keypoint klist);
int DistSquared(Keypoint k1, Keypoint k2);
void FatalError(char *fmt, ...);
Keypoint ReadKeys(FILE *fp);

#endif