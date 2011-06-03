#include "data_structures.h"

Sequence *createSequence()
{
  // Memory allocation
  Sequence *sequence = (Sequence *)malloc(sizeof(Sequence));
  
  // Initialization of variables
  sequence->firstFrame = NULL;
  sequence->lastFrame = NULL;  
  sequence->nbCorrBtwnFrames = NULL;
  sequence->nbFrames = 0;
  
  return sequence;
}

Frame *createFrame(IplImage *image)
{
  // Memory allocation
  Frame *frame = (Frame *)malloc(sizeof(Frame));
  
  // Initialization of variables
  frame->image = image;  
  frame->grayImage = cvCreateMat(image->height, image->width, CV_8UC1);  
  CvMat img = cvMat(image->height, image->width, CV_8UC3, image->imageData);
  cvCvtColor(&img, frame->grayImage, CV_BGR2GRAY); // Gray level transform
  frame->nextFrame = NULL;
  frame->firstPoint = NULL;
  frame->lastPoint = NULL;
  frame->H = NULL;
  frame->P = NULL;
  frame->id = 0;
  frame->nbPoints = 0;
  frame->nbMatchPoints = 0;
  frame->isKeyFrame = false;
  
  return frame;
}


Corner *createCorner(int x, int y)
{
  // Memory allocation
  Corner *corner = (Corner *)malloc(sizeof(Corner));
  
  // Initialization of variables
  corner->imagePoint = cvCreateMat(3, 1, CV_64FC1);
  cvmSet(corner->imagePoint, 0, 0, (double)x);
  cvmSet(corner->imagePoint, 1, 0, (double)y);
  cvmSet(corner->imagePoint, 2, 0, 1);
  corner->nextPoint = NULL;
  corner->matchNextFrame = NULL;
  corner->matchPrevFrame = NULL;
  corner->worldPoint = NULL;    
  corner->SSDMatchPrevFrame = 0;
  corner->nbMatchBackward = 0;
  corner->nbMatchForward = 0;
  corner->isInlier = false;
  corner->isFromSample = false;  

  // Normal harris
  corner->featureType = HARRIS;

  //corner->scale = -1.0f;
  //corner->orientation = 0.0f;
  //corner->descriptor = NULL;
  corner->siftKey = NULL;
  
  return corner;
}

Corner* createCornerSIFT(float x,float y, Keypoint siftKey)
{
	// Memory allocation
	Corner *corner = (Corner *)malloc(sizeof(Corner));

	// Initialization of variables
	corner->imagePoint = cvCreateMat(3, 1, CV_64FC1);
	cvmSet(corner->imagePoint, 0, 0, (double)x);
	cvmSet(corner->imagePoint, 1, 0, (double)y);
	cvmSet(corner->imagePoint, 2, 0, 1);
	corner->nextPoint = NULL;
	corner->matchNextFrame = NULL;
	corner->matchPrevFrame = NULL;
	corner->worldPoint = NULL;    
	corner->SSDMatchPrevFrame = 0;
	corner->nbMatchBackward = 0;
	corner->nbMatchForward = 0;
	corner->isInlier = false;
	corner->isFromSample = false;  

	// SIFT feature
	corner->featureType = SIFT;
	
	//corner->scale = scale;
	//corner->orientation = orientation;

	// Descriptor copy
	//corner->descriptor = (unsigned char*)malloc(sizeof(unsigned char)*descriptor_size);
	//memcpy(corner->descriptor,descriptor,descriptor_size);
	corner->siftKey = siftKey;


	return corner;
}

Inlier *createInlier(Corner *point, double residualError)
{
  // Memory allocation
  Inlier *inlier = (Inlier *)malloc(sizeof(Inlier));
  
  // Initialization of variables
  inlier->point = point;  
  inlier->residualError = residualError;  
  inlier->nextInlier = NULL;
  
  return inlier;
}

Homography *createHomography(CvMat *H)
{
  // Memory allocation
  Homography *homography = (Homography *)malloc(sizeof(Homography));
  
  // Initialization of variables
  homography->H = H;
  homography->firstInlier = NULL;
  homography->lastInlier = NULL;
  homography->residualError = 0;
  homography->nbInliers = 0;
  
  return homography;
}

void addFrame(Frame *frame, Sequence *list)
{
  // Add the element at the end of the list
  if (list->firstFrame == NULL)
  {
    list->firstFrame = frame;
    list->lastFrame = frame;
  }
  else
  {
    list->lastFrame->nextFrame = frame;
    list->lastFrame = frame;
  }
  
  frame->id = list->nbFrames; // Set the frame number
  
  list->nbFrames++; // Increment the number of elements
}

void addCorner(Corner *corner, Frame *list)
{
  // Add the element at the end of the list
  if (list->firstPoint == NULL)
  {
    list->firstPoint = corner;
    list->lastPoint = corner;  
  }
  else
  {
    list->lastPoint->nextPoint = corner;
    list->lastPoint = corner;
  }
  
  list->nbPoints++; // Increment the number of elements
}

void addInlier(Inlier *inlier, Homography *list)
{
  // Add the element at the end of the list
  if (list->firstInlier == NULL)
  {    
    list->firstInlier = inlier;
    list->lastInlier = inlier;
  }
  else
  {    
    list->lastInlier->nextInlier = inlier;
    list->lastInlier = inlier;
  }
    
  list->nbInliers++; // Increment the number of elements
}

void releaseSequence(Sequence *sequence)
{
  while (sequence->firstFrame != NULL) // Remove each element of the list
  {
    Frame *toDelete = sequence->firstFrame;
    sequence->firstFrame = toDelete->nextFrame;
    
    releaseFrame(toDelete);
  }
  
  if (sequence->nbCorrBtwnFrames != NULL)
  {
    for (int i = 0; i < sequence->nbFrames; i++)
      free(sequence->nbCorrBtwnFrames[i]);
    
    free(sequence->nbCorrBtwnFrames);
  }
  
  free(sequence);
}

void releaseFrame(Frame *frame)
{
  cvReleaseImage(&frame->image);  
  
  cvReleaseMat(&frame->grayImage);
  
  while (frame->firstPoint != NULL) // Remove each element of the list
  {
    Corner *toDelete = frame->firstPoint;
    frame->firstPoint = toDelete->nextPoint;
    
    releaseCorner(toDelete);
  }
  
  if (frame->H != NULL)
    cvReleaseMat(&frame->H);
  
  if (frame->P != NULL)
    cvReleaseMat(&frame->P);
  
  free(frame);
}

void releaseCorner(Corner *corner)
{
  cvReleaseMat(&corner->imagePoint);
  
  if (corner->worldPoint != NULL)
  {
    cvReleaseMat(&corner->worldPoint);

	/// Descriptor memory allocation check
	//if(corner->descriptor != NULL)
	//{
	//	free(corner->descriptor);
	//}
    
    Corner *currentCorner = corner->matchNextFrame;

    while (currentCorner != NULL) // Set to NULL the 3D correspondence for each match
    {
      currentCorner->worldPoint = NULL;
      currentCorner = currentCorner->matchNextFrame;
    }
  }
    
  free(corner);
}

void releaseHomography(Homography *homography)
{
  cvReleaseMat(&homography->H);
  
  while (homography->firstInlier != NULL) // Remove each element of the list
  {
    Inlier *toDelete = homography->firstInlier;
    homography->firstInlier = toDelete->nextInlier;
    
    releaseInlier(toDelete);
  }
  
  free(homography);
}

void releaseInlier(Inlier *inlier)
{
  free(inlier);
}