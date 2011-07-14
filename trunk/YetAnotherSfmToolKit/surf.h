#ifndef SURF_H_
#define SURF_H_

#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "data_structures.h"
#include "sift.h"

void findSURF(Frame *frame,char* keyFileName);
Keypoint ReadSURFKeyFile(char *filename);
Keypoint ReadSURFKeys(FILE *fp);

#endif