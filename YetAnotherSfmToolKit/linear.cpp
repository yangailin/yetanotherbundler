#include "linear.h"

void normalize(CvMat *x1[], CvMat *x2[], int nbCorr, CvMat *ST1, CvMat *ST2)
{
  int dimX1 = x1[0]->rows; // is x1[] a set of 2D points or 3D points?
  int dimX2 = x2[0]->rows; // is x2[] a set of 2D points or 3D points?
  
  //double meanX1[dimX1 - 1]; // the centroid of the first set of points
  //double meanX2[dimX2 - 1]; // the centroid of the second set of points
  double* meanX1 = new double[dimX1];
  double* meanX2 = new double[dimX2];

  for (int i = 0; i < dimX1 - 1; i++)
    meanX1[i] = 0;
  
  for (int i = 0; i < dimX2 - 1; i++)
    meanX2[i] = 0;
  
  for(int i = 0; i < nbCorr; i++) // compute the centroids
  {
    scaleToInhomogeneous(x1[i]);
    scaleToInhomogeneous(x2[i]);
    
    for (int j = 0; j < dimX1 - 1; j++)
      meanX1[j] += cvmGet(x1[i], j, 0);
    
    for (int j = 0; j < dimX2 - 1; j++)
      meanX2[j] += cvmGet(x2[i], j, 0);
  }
  
  for (int i = 0; i < dimX1 - 1; i++)
    meanX1[i] /= (double)nbCorr;
  
  for (int i = 0; i < dimX2 - 1; i++)
    meanX2[i] /= (double)nbCorr;
  
  double meanDist1 = 0; // the average distance of points of the
                        // first set to the centroid of the first set
  double meanDist2 = 0; // the average distance of points of the
                        // second set to the centroid of the second set
  
  for(int i = 0; i < nbCorr; i++) // compute the average distances
  {
    double squaredDist = 0;
    
    for (int j = 0; j < dimX1 - 1; j++)
      squaredDist += pow(cvmGet(x1[i], j, 0) - meanX1[j], 2);
    
    meanDist1 += sqrt(squaredDist);
    
    squaredDist = 0;
    
    for (int j = 0; j < dimX2 - 1; j++)
      squaredDist += pow(cvmGet(x2[i], j, 0) - meanX2[j], 2);
    
    meanDist2 += sqrt(squaredDist);
  }
  
  meanDist1 /= (double)nbCorr;
  meanDist2 /= (double)nbCorr;
  
  // compute the first normalizing similarity transformation
  cvSetIdentity(ST1);
  
  for (int i = 0; i < dimX1 - 1; i++)
    cvmSet(ST1, i , i, sqrt((double)(dimX1 - 1)) / meanDist1);
  
  for (int i = 0; i < dimX1 - 1; i++)
    cvmSet(ST1, i , dimX1 - 1, -sqrt((double)(dimX1 - 1)) * meanX1[i] / meanDist1);
  
  // compute the second normalizing similarity transformation
  cvSetIdentity(ST2);
  
  for (int i = 0; i < dimX2 - 1; i++)
    cvmSet(ST2, i , i, sqrt((double)(dimX2 - 1)) / meanDist2);
  
  for (int i = 0; i < dimX2 - 1; i++)
    cvmSet(ST2, i , dimX2 - 1, -sqrt((double)(dimX2 - 1)) * meanX2[i] / meanDist2);
  
  for(int i = 0; i < nbCorr; i++) // normalize the points of the first and second
                                  // sets with respective computed transformations
  {
    cvMatMul(ST1, x1[i], x1[i]);
    cvMatMul(ST2, x2[i], x2[i]);
  }
}

void findHomography(CvMat *x1[], CvMat *x2[], int nbCorr, CvMat *H)
{  
  CvMat *T1 = cvCreateMat(3, 3, CV_64FC1); // the normalizing similarity transformation for x1[]
  CvMat *T2 = cvCreateMat(3, 3, CV_64FC1); // the normalizing similarity transformation for x2[]
  
  normalize(x1, x2, nbCorr, T1, T2); // compute and apply T1 and T2 to respectively x1[] and x2[];
  
  CvMat *A = cvCreateMat(2 * nbCorr, 9, CV_64FC1); // the single 2*nbCorr x 9 matrix of equations
  
  for (int i = 0; i < nbCorr; i++) // stack the nbCorr 2 x 9 matrices
  {
    int l = 2 * i;
    
    cvmSet(A, l, 0, 0);
    cvmSet(A, l, 1, 0);
    cvmSet(A, l, 2, 0);
    cvmSet(A, l, 3, -cvmGet(x1[i], 0, 0));
    cvmSet(A, l, 4, -cvmGet(x1[i], 1, 0));
    cvmSet(A, l, 5, -cvmGet(x1[i], 2, 0));
    cvmSet(A, l, 6, cvmGet(x2[i], 1, 0) * cvmGet(x1[i], 0, 0));
    cvmSet(A, l, 7, cvmGet(x2[i], 1, 0) * cvmGet(x1[i], 1, 0));
    cvmSet(A, l, 8, cvmGet(x2[i], 1, 0) * cvmGet(x1[i], 2, 0));
    
    l++;
    
    cvmSet(A, l, 0, cvmGet(x1[i], 0, 0));
    cvmSet(A, l, 1, cvmGet(x1[i], 1, 0));
    cvmSet(A, l, 2, cvmGet(x1[i], 2, 0));
    cvmSet(A, l, 3, 0);
    cvmSet(A, l, 4, 0);
    cvmSet(A, l, 5, 0);
    cvmSet(A, l, 6, -cvmGet(x2[i], 0, 0) * cvmGet(x1[i], 0, 0));
    cvmSet(A, l, 7, -cvmGet(x2[i], 0, 0) * cvmGet(x1[i], 1, 0));
    cvmSet(A, l, 8, -cvmGet(x2[i], 0, 0) * cvmGet(x1[i], 2, 0));
  }
  
  DLTAlgorithm(A, H); // obtain the (normalized) homography matrix by DLT
  
  cvReleaseMat(&A);
  
  denormalize(x1, x2, nbCorr, H, T1, T2, HOMOGRAPHY); // denormalize the homography matrix,
                                                      // x1[] and x2[] (in case of later use)
  
  cvReleaseMat(&T1);
  cvReleaseMat(&T2);
}

void DLTAlgorithm(CvMat *A, CvMat *X)
{
  CvMat *S = cvCreateMat(A->rows, A->cols, CV_64FC1);
  CvMat *V = cvCreateMat(A->cols, A->cols, CV_64FC1);
  
  cvSVD(A, S, NULL, V); // perform SVD
    
  for (int i = 0; i < X->rows; i++) // get the solution as the last column of V
    for (int j = 0; j < X->cols; j++)
      cvmSet(X, i, j, cvmGet(V, i * X->cols + j, A->cols - 1));
  
  cvReleaseMat(&S);
  cvReleaseMat(&V);
}

void denormalize(CvMat *x1[], CvMat *x2[], int nbCorr,
                 CvMat *LT, CvMat *ST1, CvMat *ST2, int relationship)
{
  int dimX1 = x1[0]->rows; // is x1[] a set of 2D points or 3D points?
  int dimX2 = x2[0]->rows; // is x2[] a set of 2D points or 3D points?
  
  CvMat* ST1inv = cvCreateMat(dimX1, dimX1, CV_64FC1);
  CvMat* ST2inv = cvCreateMat(dimX2, dimX2, CV_64FC1);
  
  cvInvert(ST1, ST1inv); // compute ST1^(-1)
  cvInvert(ST2, ST2inv); // compute ST2^(-1)
  
  for(int i = 0; i < nbCorr; i++) // denormalize the points of the first and second
                                  // sets with respective denormalizing transformations
  {    
    cvMatMul(ST1inv, x1[i], x1[i]);
    cvMatMul(ST2inv, x2[i], x2[i]);
    
    scaleToInhomogeneous(x1[i]); // scale the homogeneous points so that
    scaleToInhomogeneous(x2[i]); // their last coordinate becomes 1
  }
  
  // the relationship is a homography matrix or a camera matrix : the denormalization is similar
  if (relationship == HOMOGRAPHY || relationship == CAM_PROJ_MAT)
  {
    cvMatMul(ST2inv, LT, LT);
    cvMatMul(LT, ST1, LT); // H or P = ST2^(-1) * LT * ST1
  }
  else if (relationship == FUND_MAT) // the relationship is a fundamental matrix
  {
    CvMat* ST2t = cvCreateMat(dimX2, dimX2, CV_64FC1);
    
    cvTranspose(ST2, ST2t);
    
    cvMatMul(ST2t, LT, LT);
    cvMatMul(LT, ST1, LT); // F = ST2^t * LT * ST1
    
    cvReleaseMat(&ST2t);
  }
  
  norm(LT); // scale the matrix so that its norm becomes 1
    
  cvReleaseMat(&ST1inv);
  cvReleaseMat(&ST2inv);
}