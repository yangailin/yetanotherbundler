#ifndef SFM_H_
#define SFM_H_

#include <fstream>

#include "nonlinear.h"
#include "sift.h"
#include "surf.h"

void SfM(char *seqPath, char *KMatPath,int featureExtractor);

#endif