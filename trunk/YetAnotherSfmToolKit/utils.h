#ifndef UTILS_H_
#define UTILS_H_

#include "data_structures.h"

void norm(CvMat *mat);

/** Returns the integer inhomogeneous coordinates of a homogeneous 2D point
 **************************************************************************************************/
CvPoint get2DPoint(CvMat *hom2DPoint);

/** Returns the integer inhomogeneous coordinates of a homogeneous 2D Harris corner
 **************************************************************************************************/
CvPoint get2DPoint(Corner *corner);

/** Returns the double inhomogeneous coordinates of a homogeneous 2D point
 **************************************************************************************************/
CvPoint2D64f get2DPointf(CvMat *hom2DPoint);

/** Returns the double inhomogeneous coordinates of a homogeneous 2D Harris corner
 **************************************************************************************************/
CvPoint2D64f get2DPointf(Corner *corner);

CvMat *getHomogeneous2DPoint(double x, double y);

void normInhomogeneous(CvMat *mat);

void scaleToInhomogeneous(CvMat *mat);

#endif