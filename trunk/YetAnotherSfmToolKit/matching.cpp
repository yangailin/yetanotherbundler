#include "matching.h"

#define SAMPLE_SIZE 4 // size of a sample of correspondences for the computation of a homography
#define KEEP_INLIERS true // flag specifying that inlying correspondences must be preserved

/// Function prototype
bool isLocalMaximum(int row, int col, CvMat *H, int windowRadius);

void getNeighborhoodWindows(CvPoint p1, CvPoint p2, Frame *frame,
                            int neighbWindowRadius, CvRect neighborhoodWindows[2]);

double computeSSD(CvMat *image1, CvRect window1, CvMat *image2, CvRect window2);

void setCorrespondences(Corner *corr[], int nbElemZone[SAMPLE_SIZE],
                        int offsets[SAMPLE_SIZE], Frame *frame);

void setSample(Corner *sample[SAMPLE_SIZE], Corner *corr[],
               int nbElemZone[SAMPLE_SIZE], int offsets[SAMPLE_SIZE]);

bool areCollinear(Corner *sample[SAMPLE_SIZE], double minDist = 10, int maxCollinearPoints = 2);

void cleanFrame(Frame *frame, bool keepInliers);

void findCorners(Frame *frame, double t, double k, int localMaxWindowRadius)
{
  CvMat *Ix = cvCreateMat(frame->image->height, frame->image->width, CV_32FC1);
  CvMat *Iy = cvCreateMat(frame->image->height, frame->image->width, CV_32FC1);
  CvMat *Ix2 = cvCreateMat(frame->image->height, frame->image->width, CV_32FC1);
  CvMat *Iy2 = cvCreateMat(frame->image->height, frame->image->width, CV_32FC1);
  CvMat *IxIy = cvCreateMat(frame->image->height, frame->image->width, CV_32FC1);
  CvMat *M = cvCreateMat(2, 2, CV_64FC1);
  CvMat *H = cvCreateMat(frame->image->height, frame->image->width, CV_64FC1);
  
  
  cvSobel(frame->grayImage, Ix, 1, 0, 3); // derivate in x : Ix
  cvSobel(frame->grayImage, Iy, 0, 1, 3); // derivate in y ; Iy
    
  cvMul(Ix, Iy, IxIy, 1); // IxIy  
  cvPow(Ix, Ix2, 2);      // Ix^2
  cvPow(Iy, Iy2, 2);      // Iy^2
  
  cvSmooth(Ix2, Ix2, CV_GAUSSIAN, 7, 7, 0, 0);   // gaussian filter on Ix^2
  cvSmooth(Iy2, Iy2, CV_GAUSSIAN, 7, 7, 0, 0);   // gaussian filter on Iy^2
  cvSmooth(IxIy, IxIy, CV_GAUSSIAN, 7, 7, 0, 0); // gaussian filter on IxIy
  
  for (int row = 0; row < frame->image->height; row++)
    for (int col = 0; col < frame->image->width; col++)
    {      
      cvmSet(M, 0, 0, cvmGet(Ix2, row, col));  //  M =  |  Ix^2 IxIy |
      cvmSet(M, 0, 1, cvmGet(IxIy, row, col)); //       |  IxIy Iy^2 |
      cvmSet(M, 1, 0, cvmGet(IxIy, row, col));
      cvmSet(M, 1, 1, cvmGet(Iy2, row, col));
      
      double Tr = cvmGet(M, 0, 0) + cvmGet(M, 1, 1); // compute the trace of the M matrix
                                                     // tr(M) = Ix^2 + Iy^2
                                                     
      double h =  cvDet(M) - k*Tr*Tr; // compute the Harris value
                                      // h = det(M) - kTr^2(M)
      
      if (h > 0 && Tr > t*255)  // h > 0 AND Tr > t : potential corner
        cvmSet(H, row, col, h);
      else                      // not an interesting point
        cvmSet(H, row, col, 0);
    }
    
  for (int row = 0; row < frame->image->height; row++)
    for (int col = 0; col < frame->image->width; col++)
    {
      if (isLocalMaximum(row, col, H, localMaxWindowRadius)) // if the point corresponds
                                                             // to a local maximum of H
      {
        Corner *point = createCorner(col, row); 
        addCorner(point, frame); // add point in the list
      }
    }

  cvReleaseMat(&Ix);
  cvReleaseMat(&Iy);
  cvReleaseMat(&Ix2);
  cvReleaseMat(&Iy2);
  cvReleaseMat(&IxIy);
  cvReleaseMat(&M);
  cvReleaseMat(&H);
}

bool isLocalMaximum(int row, int col, CvMat *H, int windowRadius)
{
  double val = cvmGet(H, row, col); // the potential corner point
  
  if (val == 0)
    return false; // zero value --> not an interesting point
  
  for (int i = row - windowRadius; i <= row + windowRadius; i++)
    for (int j = col - windowRadius; j <= col + windowRadius; j++)
      if ((i >= 0) && (i < H->rows) && (j >= 0) && (j < H->cols))
      {
        double nVal = cvmGet(H, i, j);
        
        if (nVal > val)
          return false;  // not the bigger point in the window --> not an interesting point
      }
  
  return true; // the point is a corner
}

void findCorrespondences(Frame *frame, CvMat *H, double SSDMax, double maxDist,
                         int proximWindowRadius, int neighbWindowRadius)
{  
  Corner *currentPointFrame1 = frame->firstPoint;
  
  while (currentPointFrame1 != NULL) // for each corner point in the first frame
  {
    CvPoint p1 = get2DPoint(currentPointFrame1);
    
    double bestSSD = SSDMax + 1;
    
    Corner *currentPointFrame2 = frame->nextFrame->firstPoint;
    
    while (currentPointFrame2 != NULL) // for each corner point in the second frame
    {
      CvPoint p2 = get2DPoint(currentPointFrame2);

      // look for corresponding points in a window around the point in the second frame
      if ((p2.x >= p1.x - proximWindowRadius) && (p2.x <= p1.x + proximWindowRadius) &&
          (p2.y >= p1.y - proximWindowRadius) && (p2.y <= p1.y + proximWindowRadius))
      {        
        CvRect neighborhoodWindows[2];
        
        // get the neighborhood windows dimension
        getNeighborhoodWindows(p1, p2, frame, neighbWindowRadius, neighborhoodWindows);
        
        // compute the sum of squared differences between the points
        double SSD = computeSSD(frame->grayImage, neighborhoodWindows[0],
                                frame->nextFrame->grayImage, neighborhoodWindows[1]);
        
		/// SSD 값은 낮을수록 좋은것임.
        if (SSD < bestSSD) // if the SSD value is lower than the best value found until now
        {
          bool distOk = true;	/// 일단 distOk 라는 변수 설정함.
          
          if (H != NULL) // if a homography is given
          {
            // find the distance of the point in the second frame and the image by the homography
            // of the point in the first frame
            double dist = findDistance(currentPointFrame1->imagePoint,
                                       currentPointFrame2->imagePoint, H);
            
            if (dist > maxDist)
              distOk = false; // the condition on the distance is not satisfied
          }
          
          // the point is a potential correspondence
          if (currentPointFrame2->matchPrevFrame == NULL && distOk) // no other candidate
          {
            if (currentPointFrame1->matchNextFrame != NULL)
            {
              currentPointFrame1->matchNextFrame->matchPrevFrame = NULL;
              frame->nbMatchPoints--;
            }
              
            currentPointFrame1->matchNextFrame = currentPointFrame2;  
            currentPointFrame2->matchPrevFrame = currentPointFrame1;
            frame->nbMatchPoints++;
            
            currentPointFrame2->SSDMatchPrevFrame = SSD;
            bestSSD = SSD;
          }
          else if (SSD < currentPointFrame2->SSDMatchPrevFrame && distOk) // a dismissed candidate
          {
            if (currentPointFrame1->matchNextFrame != NULL)
            {
              currentPointFrame1->matchNextFrame->matchPrevFrame = NULL;
              frame->nbMatchPoints--;
            }
            
            currentPointFrame2->matchPrevFrame->matchNextFrame = NULL;
            frame->nbMatchPoints--;
            
            currentPointFrame1->matchNextFrame = currentPointFrame2;  
            currentPointFrame2->matchPrevFrame = currentPointFrame1;
            frame->nbMatchPoints++;
            
            currentPointFrame2->SSDMatchPrevFrame = SSD;            
            bestSSD = SSD;
          }
        }
      }
      
      currentPointFrame2 = currentPointFrame2->nextPoint; 
    }
    
    currentPointFrame1 = currentPointFrame1->nextPoint;
  }
}

void getNeighborhoodWindows(CvPoint p1, CvPoint p2, Frame *frame,
                            int neighbWindowRadius, CvRect neighborhoodWindows[2])
{
  int offXBeg = neighbWindowRadius;
  int offXEnd = neighbWindowRadius;
  int offYBeg = neighbWindowRadius;
  int offYEnd = neighbWindowRadius;
  
  if(p1.x - offXBeg < 0)                        // out of the first image
    offXBeg = p1.x;
  if(p1.y - offYBeg < 0)                        // out of the first image
    offYBeg = p1.y;
  if(p1.x + offXEnd > frame->grayImage->cols-1) // out of the first image
    offXEnd = frame->grayImage->cols-1 - p1.x;
  if(p1.y + offYEnd > frame->grayImage->rows-1) // out of the first image
    offYEnd = frame->grayImage->rows-1 - p1.y;
  
  if(p2.x - offXBeg < 0)                                   // out of the second image
    offXBeg = p2.x;
  if(p2.y - offYBeg < 0)                                   // out of the second image
    offYBeg = p2.y;
  if(p2.x + offXEnd > frame->nextFrame->grayImage->cols-1) // out of the second image
    offXEnd = frame->nextFrame->grayImage->cols-1 - p2.x;
  if(p2.y + offYEnd > frame->nextFrame->grayImage->rows-1) // out of the second image
    offYEnd = frame->nextFrame->grayImage->rows-1 - p2.y;
  
  int width = offXBeg + offXEnd + 1;  // the width of the neighborhood window
  int height = offYBeg + offYEnd + 1; // the height of the neighborhood window
  
  neighborhoodWindows[0] = cvRect(p1.x - offXBeg, p1.y - offYBeg, width, height);
  neighborhoodWindows[1] = cvRect(p2.x - offXBeg, p2.y - offYBeg, width, height);
}

double computeSSD(CvMat *image1, CvRect window1, CvMat *image2, CvRect window2)
{
  double SSD = 0;
  
  for (int i = 0; i < window1.height; i++)
    for (int j = 0; j < window1.width; j++)
    {
      double v1 = (double)image1->data.ptr[(window1.y + i)*image1->step + (window1.x + j)];
      double v2 = (double)image2->data.ptr[(window2.y + i)*image1->step + (window2.x + j)];
      
      SSD += pow(v1 - v2, 2);
    }
  
  return SSD/((double)(window1.height * window1.width)); // return the average squared difference
}

int RANSAC(Frame *frame, double t)
{
  if (frame->nbMatchPoints < SAMPLE_SIZE) // not enough correspondences between the two frames
    return 0;
  
  Corner **corr = new Corner*[frame->nbMatchPoints];
//  Corner *corr[frame->nbMatchPoints];
  
  // tables used for distributed spatial sampling
  int nbElemZone[SAMPLE_SIZE];
  int offsets[SAMPLE_SIZE];
  
  // set the array of the correspondences and set the distributed spatial sampling tables
  setCorrespondences(corr, nbElemZone, offsets, frame);
  
  Corner *lastSample[SAMPLE_SIZE];
  
  Homography *bestHomography = NULL; // the best model is set to NULL
  
  double epsilon = 1; // the proportion of outliers is maximal (1)
  double N = 1E100;  // the maximal number of iterations is set to infinity
  double sampleCount = 0;
  
  srand(time(NULL)); // initialize the random number generator
  
  while (N > sampleCount)
  {
    Corner *sample[SAMPLE_SIZE];
    
    setSample(sample, corr, nbElemZone, offsets); // set a random sample
    
    if(!areCollinear(sample) || bestHomography == NULL) // check the non-collinearity constraint
                                                        // unless the best model is NULL
    {
      CvMat *H = cvCreateMat(3, 3, CV_64FC1);
      
      CvMat *x1[SAMPLE_SIZE];
      CvMat *x2[SAMPLE_SIZE];
      
      for (int i = 0; i < SAMPLE_SIZE; i++)
      {
        x1[i] = sample[i]->imagePoint;    
        x2[i] = sample[i]->matchNextFrame->imagePoint;
      }
      
      findHomography(x1, x2, SAMPLE_SIZE, H); // compute a homography by the linear method
                                              // in the minimal case (four correspondences)
      
      bool betterHFound = false;
      
      Homography *homography = createHomography(H);
        
      for (int i = 0; i < frame->nbMatchPoints; i++)
      {
        // compute the distance between the putative matched point and the point predicted by the
        // homography; the distance is half the square root of the reprojection error
        double dist = findDistance(corr[i]->imagePoint, corr[i]->matchNextFrame->imagePoint, H);
        
        if (dist <= t) // check if the correspondence is consistent with the homography
        {
          Inlier *inlier = createInlier(corr[i], dist);
          addInlier(inlier, homography);
        }      
      }
        
      Inlier *currentInlier = homography->firstInlier;
      
      while (currentInlier != NULL) // compute the residual error for the homography
      {
        homography->residualError += pow(2 * currentInlier->residualError, 2);
        
        currentInlier = currentInlier->nextInlier;
      }
      
      // the residual error for the homography is sqrt[sum_i^n(reproj_error_i) / (4 * n)]
      homography->residualError =
        sqrt(homography->residualError) / sqrt(4 * (double)homography->nbInliers);
      
      if (bestHomography == NULL) // no other candidate for best model
      {
        bestHomography = homography;
        betterHFound = true;
      }
      // a dismissed candidate; the new homography has more inliers or an equal number of
      // inliers and a lower residual error
      else if (homography->nbInliers > bestHomography->nbInliers ||
               (homography->nbInliers == bestHomography->nbInliers &&
                homography->residualError < bestHomography->residualError))
      {
        releaseHomography(bestHomography);
        bestHomography = homography;
        betterHFound = true;
      }
      else
        releaseHomography(homography);
      
    
      if (betterHFound) // the proportion of inliers may have changed
      {
        epsilon = 1 - (double)bestHomography->nbInliers / (double)frame->nbMatchPoints;
        
        // recompute the maximal number of iterations; the formula insures that for this number of
        // iterations we have a probability of 0.99 to get a model computed from inliers only   
        N = log(1 - 0.99) / log(1 - pow(1 - epsilon, SAMPLE_SIZE));
      
        if (N < 0) // computational error that may occur and that must be handled
          N = 1E100;
      
        memcpy(lastSample, sample, sizeof(sample));
      }
    }
    
    sampleCount++;
  }
  
  int nbInliers = 0;
  
  if (bestHomography != NULL) // the best model is the desired homography
  {
    for (int i = 0; i < SAMPLE_SIZE; i++)
      lastSample[i]->isFromSample = true;
  
    Inlier *currentInlier = bestHomography->firstInlier;
    
    while (currentInlier != NULL)
    {
      currentInlier->point->isInlier = true;        
      currentInlier = currentInlier->nextInlier;
    }

    frame->H = cvCloneMat(bestHomography->H);
    
    nbInliers = bestHomography->nbInliers;
    
    releaseHomography(bestHomography);
  }
  
  return nbInliers; // return the number of inliers for the computed homography
}

void setCorrespondences(Corner *corr[], int nbElemZone[SAMPLE_SIZE],
                        int offsets[SAMPLE_SIZE], Frame *frame)
{   
  for (int i = 0; i < SAMPLE_SIZE; i++)
    nbElemZone[i] = 0;
  
  // tile the image in four zones
  int zoneHeight = frame->image->height / 2;
  int zoneWidth = frame->image->width / 2;
  
  int count = 0;
  Corner *currentPoint = frame->firstPoint;
  
  while (currentPoint != NULL)
  {
    if (currentPoint->matchNextFrame != NULL) // for each correspondence in the frame
    {
      CvPoint point = get2DPoint(currentPoint);
      
      int zone = (point.y / zoneHeight) * 2 + point.x / zoneWidth; // find its zone in the image
      
      corr[count] = currentPoint; // insert the correspondence in the array
      
      for (int i = count; i > 0; i--) // the array is sorted by the image zone of its elements
      {
        CvPoint prevPoint = get2DPoint(corr[i-1]);
        
        int zonePrev = (prevPoint.y / zoneHeight) * 2 + prevPoint.x / zoneWidth;
                         
        if (zone < zonePrev)
        {
          Corner *tmp = corr[i-1];
          corr[i-1] = corr[i];
          corr[i] = tmp;
        }
        else
          break;
      }
      
      nbElemZone[zone]++; // increment the number of elements of the zone
      count++;
    }
    
    currentPoint = currentPoint->nextPoint;
  }
  
  offsets[0] = 0;
  for (int i = 1; i < SAMPLE_SIZE; i++) // set the offsets of the zone blocks in the array
    offsets[i] = offsets[i-1] + nbElemZone[i-1];
}

void setSample(Corner *sample[SAMPLE_SIZE], Corner *corr[],
               int nbElemZone[SAMPLE_SIZE], int offsets[SAMPLE_SIZE])
{
  int nbez[SAMPLE_SIZE]; // work copy of the table containing the number of elements in each zone
  for(int i = 0; i < SAMPLE_SIZE; i++)
    nbez[i] = nbElemZone[i];
  
  for(int i = 0; i < SAMPLE_SIZE; i++) // select a sample
  {
    int nonEmpty = 0;
    for(int j = 0; j < SAMPLE_SIZE; j++)
      if(nbez[j] > 0)
        nonEmpty++;
    
    int rd1;
    
    if(nonEmpty > 1) // at least two non empty image zones
    {
      int sampler = nonEmpty - 1 + 10*(nonEmpty - 1); // weighted sampler variable
      
      rd1 = rand() % sampler; // get the random index of the zone
      if(rd1 < sampler - (nonEmpty - 1))
        rd1 = 0;
      else
        rd1 = sampler - rd1;
    }
    else
      rd1 = 0;
    
    int k = (int)floor(((double)(i*nonEmpty)) / SAMPLE_SIZE); // adapt the random index
    if(k == nonEmpty)
      k--;
    
    int znb = 0;
    while(nbez[znb] == 0 || k > 0 || rd1 > 0) // get the random zone
    {
      if(nbez[znb] == 0)
        znb = (znb + 1) % SAMPLE_SIZE;
      else
      {
        if(k > 0)
          k--;
        else
          rd1--;
        
        znb = (znb + 1) % SAMPLE_SIZE;
      }
    }
    
    int rd2 = rand() % nbez[znb]; // get the random index of the element in the random zone
    rd2 = offsets[znb] + rd2;
    
    sample[i] = corr[rd2]; // get the random element
    
    Corner *tmp = corr[rd2];
    corr[rd2] = corr[offsets[znb] + nbez[znb] -1];
    corr[offsets[znb] + nbez[znb] -1] = tmp;
    
    nbez[znb]--;
  }
}

bool areCollinear(Corner *sample[SAMPLE_SIZE], double minDist, int maxCollinearPoints)
{
  CvMat *line = cvCreateMat(3, 1, CV_64FC1);
  
  for(int i = 0; i < SAMPLE_SIZE - 1; i++)  // for every line that can be composed
    for(int j = i + 1;j < SAMPLE_SIZE; j++) // from two points of the sample
    {
      cvCrossProduct(sample[i]->imagePoint, sample[j]->imagePoint, line);
      
      normInhomogeneous(line);
      
      int collinearPoints = 2;
      
      for(int k = 0; k < SAMPLE_SIZE; k++) // for any other point of the sample
      {
        if(k != i && k != j)
        {
          // compute the distance from the point to the line
          double dist = fabs(cvDotProduct(sample[k]->imagePoint, line));
          
          if(dist < minDist)
            collinearPoints++;
          
          if(collinearPoints > maxCollinearPoints)
          {
            cvReleaseMat(&line);
            return true; // the sample is collinear
          }
        }
      }
    }
  
  cvReleaseMat(&line);
  return false; // the sample is not collinear
}


int optimHGuidMatchCycle(Frame *frame,int featureExtractor)
{
  cleanFrame(frame, KEEP_INLIERS); // remove all outlying correspondences
  
  int nbInliers;
  int newNbInliers;
  
  int nbCycles = 0;
  
  do // homography optimization - guided matching cycle
  {    
    nbInliers = frame->nbMatchPoints;
    
    if (nbInliers >= 5) // the Levenberg-Marquardt algorithm requires more unknows than parameters
    {
		CvMat **x1 = new CvMat*[nbInliers];
		CvMat **x2 = new CvMat*[nbInliers];
      //CvMat *x1[nbInliers];
      //CvMat *x2[nbInliers];
      
      int count = 0;
      Corner *currentPoint = frame->firstPoint;
      
      while (currentPoint != NULL)
      {
        if (currentPoint->matchNextFrame != NULL) // the point has a correspondence
        {
          x1[count] = currentPoint->imagePoint;
          x2[count] = currentPoint->matchNextFrame->imagePoint;
                  
          count++;
        }
        
        currentPoint = currentPoint->nextPoint;
      }
      
      optimizeHomography(frame->H, x1, x2, nbInliers); // optimize the homography using
                                                       // the Levenberg-Marquardt algorithm
    }
    
    cleanFrame(frame, !KEEP_INLIERS); // remove all correspondences (even inliers)
    
	/// I should change here to SIFT routine.
	if(featureExtractor == HARRIS)
	{
		findCorrespondences(frame, frame->H, 200); // re-evaluate the inlying correspondences
                                               // with the optimized homography
	}
	else if(featureExtractor == SIFT)
	{
		matchSIFT(frame,frame->H);
	}
    
    Corner *currentPoint = frame->firstPoint;
    
    while (currentPoint != NULL) // label points having a correspondence as inliers
    {
      if (currentPoint->matchNextFrame != NULL)
        currentPoint->isInlier = true;
      
      currentPoint = currentPoint->nextPoint;
    }
    
    newNbInliers = frame->nbMatchPoints;
    
	printf("Calculating epochs : %d\n",nbCycles);

    nbCycles++;
  }
  while (nbInliers != newNbInliers); // repeat as long as the number of inliers is not stable
  
  return nbCycles; // return the number of iterations of the cycle
}

void cleanFrame(Frame *frame, bool keepInliers)
{
  Corner *currentPoint = frame->firstPoint;
  
  while (currentPoint != NULL) // for each corner in the frame
  {
    if (!currentPoint->isInlier || !keepInliers) // if the point isn't an inlier or if
                                                 // all correspondences must be removed
    {
      currentPoint->isInlier = false;
      
      if (currentPoint->matchNextFrame != NULL) // if the point is associated
                                                // with a point in the next frame
      {
        // the link is broken
        currentPoint->matchNextFrame->matchPrevFrame = NULL;
        currentPoint->matchNextFrame = NULL;
        
        frame->nbMatchPoints--; // the number of match points (inliers) is decreased
      }
    }
    
    currentPoint = currentPoint->nextPoint;
  }
}
