#include "nonlinear.h"

#define sparseLM sba_motstr_levmar // the sparse Levenberg-Marquardt algorithm written by M.
                                   // Lourakis is designed for the bundle adjustment; thus, when
                                   // using it in another context that the bundle adjustment, one
                                   // should use the more generic function name "sparseLM"

void homDataReprErr(double *p, double *hx, int m, int n, void *adata);
void homReprErr(int j, int i, double *aj, double *bi, double *xij, void *adata);

double findDistance(CvMat *x1, CvMat *x2, CvMat *H)
{
  double p[2];
  
  p[0] = cvmGet(x1, 0, 0); // the initial estimate of the perfect match (by the
  p[1] = cvmGet(x1, 1, 0); // homography) of the second-view point in the first view 
  
  double x[4];
  
  x[0] = cvmGet(x1, 0, 0); // the measured (noisy)
  x[1] = cvmGet(x1, 1, 0); // point in the first view;
  x[2] = cvmGet(x2, 0, 0); // the measured (noisy)
  x[3] = cvmGet(x2, 1, 0); // point in the second view
  
  double info[LM_INFO_SZ];
  
  // compute the reprojection error using the given homography
  dlevmar_dif(homDataReprErr, p, x, 2, 4, 500, NULL, info, NULL, NULL, H);
  
  double reprojectionError = info[1]; // retrieve the reprojection error
  
  return (sqrt(reprojectionError) / 2); // return the geomtric distance = 1/2 * sqrt(reproj_error)
}

void optimizeHomography(CvMat *H, CvMat *x1[], CvMat *x2[], int nbCorr)
{
	char* vMask = new char[nbCorr];
//  char vMask[nbCorr];
  
	double* p = new double[9 + 2 * nbCorr];
//  double p[9 + 2 * nbCorr]; // the homography parameters + all points (in the first view) parameters
  
	double* x = new double[4*nbCorr];
//  double x[4 * nbCorr]; // all measured image points (in the two views)
  
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			p[i * 3 + j] = cvmGet(H, i, j); // set the initial parameters of the homography to optimize
  
	for (int i = 0; i < nbCorr; i++)
	{
		vMask[i] = 1;

		CvPoint2D64f point1 = get2DPointf(x1[i]); // get the inhomogeneous coordinates
		CvPoint2D64f point2 = get2DPointf(x2[i]); // of the image points in the two views

		// set the initial parameters of image points in the first view
		p[9 + i * 2] = point1.x;
		p[9 + i * 2 + 1] = point1.y;

		// set the measured image points in each view
		x[i * 4] = point1.x;
		x[i * 4 + 1] = point1.y;
		x[i * 4 + 2] = point2.x;
		x[i * 4 + 3] = point2.y;
	}
  
	double opts[SBA_OPTSSZ]; // thresholds for the sparse Levenberg-Marquardt algorithm

	opts[0] = SBA_INIT_MU;
	opts[1] = SBA_STOP_THRESH;
	opts[2] = SBA_STOP_THRESH;
	opts[3] = SBA_STOP_THRESH;
  
	sparseLM(nbCorr, 1, 0, vMask, p, 9, 2, x, 4,           // optimize the homography matrix
			homReprErr, NULL, NULL, 500, 0, opts, NULL);
  
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			cvmSet(H, i, j, p[i * 3 + j]); // retrieve the optimized homography matrix
  
	norm(H); // scale the matrix so that its norm becomes 1
}

void homDataReprErr(double *p, double *hx, int m, int n, void *adata)
{
  CvMat *xhat = getHomogeneous2DPoint(p[0], p[1]); // estimate of the point in the first view
  CvMat *H = (CvMat *)adata; // get the homography matrix
  
  cvMatMul(H, xhat, xhat); // get the estimate of the perfect match in the second view
  scaleToInhomogeneous(xhat); // scale the homogeneous point so that its last coordinate becomes 1
  
  hx[0] = p[0]; // set the optimized point in the first view; this will
  hx[1] = p[1]; // be compared to the corresponding measurement in the first view
  hx[2] = cvmGet(xhat, 0, 0); // set the computed perfect match point in the second view; this will
  hx[3] = cvmGet(xhat, 1, 0); // be compared to the corresponding measurement in the second view
  
  cvReleaseMat(&xhat);
}

void homReprErr(int j, int i, double *aj, double *bi, double *xij, void *adata)
{
  CvMat *H = cvCreateMat(3, 3, CV_64FC1);
    
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      cvmSet(H, i, j, aj[i * 3 + j]); // get the homography matrix
  
  homDataReprErr(bi, xij, 2, 4, H); // compute the reprojection error of
                                    // the image to image correspondence
      
  cvReleaseMat(&H);
}