#include "visualization.h"

IplImage *createLegend(int nbLines);

void writeLegendLine(IplImage *image, char *text, int line, int col,
               bool dot, CvScalar color = cvScalar(0, 0, 0));

void showMatchingResults(Frame *frame)
{
  Corner *currentPoint = frame->firstPoint;
  
  IplImage *img = cvCloneImage(frame->image);
  IplImage *legend = createLegend(3); // create legend
  
  char msg[50];
  sprintf(msg, "Frame %d", frame->id);
  
  writeLegendLine(legend, msg, 0, 0, false);
  writeLegendLine(legend, ": sample (RANSAC)", 1, 0, true, cvScalar(255, 0, 0));
  writeLegendLine(legend, ": inliers", 1, 210, true, cvScalar(0, 255, 0));
  writeLegendLine(legend, ": outliers", 1, 420, true, cvScalar(0, 0, 255));
  writeLegendLine(legend, "Press \"p\" to pause, any key to continue", 2, 0, false);
  
  while(currentPoint != NULL)
  {
    CvPoint pf1 = get2DPoint(currentPoint);
    
    if (currentPoint->isFromSample) // the point was a part of the last sample randomly chosen in
                                    // the RANSAC algorithm to compute the homography between the
                                    // two views
      cvCircle(img, pf1, 6, cvScalar(255, 0, 0, 0), -1, 8, 0); // the point is encircled in blue
    
    if (currentPoint->isInlier) // the correspondence between the point and its corresponding point
                                // in the next image is consistent with the homography
    {
      CvPoint pf2 = get2DPoint(currentPoint->matchNextFrame);
      cvCircle(img, pf1, 4, cvScalar(0, 255, 0), 1, 8, 0); // draw the point in green
      cvLine(img, pf1, pf2, cvScalar(0, 255, 0), 1, 8, 0); // draw a line starting from the point
                                                           // to the image coordinates of the 
                                                           // corresponding point in the next frame
    }
    else if (currentPoint->matchNextFrame != NULL) // the correspondence between the point and its
                                                   // corresponding point in the next image isn't
                                                   // consistent with the homography
    {
      CvPoint pf2 = get2DPoint(currentPoint->matchNextFrame);
      cvCircle(img, pf1, 4, cvScalar(0, 0, 255), 1, 8, 0); // draw the point in red
      cvLine(img, pf1, pf2, cvScalar(0, 0, 255), 1, 8, 0); // draw a line starting from the point
                                                           // to the image coordinates of the 
                                                           // corresponding point in the next frame
    }
    
    currentPoint = currentPoint->nextPoint;
  }
  
  cvShowImage(VISUALIZATION, img); // show the modified image
  cvShowImage(LEGEND, legend);     // show the legend
  
  int keyInt = cvWaitKey(500); // watch 5 ms for a key pressed
  
  if(keyInt != -1) 
  {
    keyInt &= 0xFFFF; // remove NUMLOCK information (0x100000) 
    if ( (keyInt & 0xFF00) == 0x0000) 
    { 
      char keyChar = keyInt & 0xFF;  // normal keys 
      if (keyChar == 'p') // pause 
        cvWaitKey();
    }
  }
   
  cvReleaseImage(&img);
  cvReleaseImage(&legend);
}

void writeLegendLine(IplImage *image, char *text, int line, int x, bool dot, CvScalar color)
{
  CvFont font; // create font
  cvInitFont(&font, CV_FONT_HERSHEY_COMPLEX_SMALL, 0.7, 0.7, 0, 1, 20); // initialize font
  
  int y = 20 * line; // y coordinate corresponding to the line number
  
  if (dot)
  {
    cvCircle(image, cvPoint(x + 10, y + 10), 5, color, -1, 8, 0); // draws a point on the image
    
    cvPutText(image, text, cvPoint(x + 25, y + 15), &font, cvScalar(0, 0, 0)); // draws text on the
                                                                               // image
  }
  else
    cvPutText(image, text, cvPoint(x + 5, y + 15), &font, cvScalar(0, 0, 0)); // draws text on the
                                                                              // image
}

IplImage *createLegend(int nbLines)
{
  IplImage *legend = cvCreateImage(cvSize(640, 21*nbLines), IPL_DEPTH_8U, 3); // creates the image
  cvZero(legend); // fill the image with black pixels
  cvAddS(legend, cvScalar(255, 255, 255), legend); // change black pixels to white pixels
  
  return legend;  
}