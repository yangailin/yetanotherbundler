#include <stdio.h>
#include "sfm.h"
#include "data_structures.h"
#include "matching.h"
#include "visualization.h"

void SfM(char *seqPath, char *KMatPath)
{
	CvMat *K = (CvMat *)cvLoad(KMatPath); // load the calibration matrix

	if (K != NULL)
	{
		if(K->rows != 3 || K->cols != 3)
		{
		  cvReleaseMat(&K);
		  
		  printf("%s not a valid K matrix\n", KMatPath);
		  return;
		}    
	}
	else
	{
		printf("%s not a valid K matrix\n", KMatPath);
		return;
	}

	////////////////////////////////////////////////////
	// Matching
	////////////////////////////////////////////////////
	Sequence *sequence = createSequence();

	printf("Loading images in %s\n", seqPath);

	char fileName[200];
	IplImage *image;
	Frame *currentFrame;

	while (true) // load the sequence
	{
		sprintf(fileName, "%s/img%.3d.jpeg", seqPath, sequence->nbFrames);

		if (!(image = cvLoadImage(fileName))) // end of sequence
		  break;
		else
		  printf("  - img%.3d.jpeg loaded - ", sequence->nbFrames);

		currentFrame = createFrame(image);

		printf("corner detection... ");
		fflush(stdout);

		findCorners(currentFrame); // detect corner points in the current image

		printf("done (%d corners found)\n", currentFrame->nbPoints);

		addFrame(currentFrame, sequence);
	}

	if (sequence->nbFrames == 0) // no sequence found
	{
		cvReleaseMat(&K);
		releaseSequence(sequence);

		printf("No images in %s\n", seqPath);
		return;
	}

	if (sequence->nbFrames == 1) // not a sequence, just one image
	{
		cvReleaseMat(&K);
		releaseSequence(sequence);

		printf("At least two images are required\n");
		return;
	}

	cvNamedWindow(LEGEND);
	cvMoveWindow(LEGEND, 100, 607);

	cvNamedWindow(VISUALIZATION);
	cvMoveWindow(VISUALIZATION, 100, 100);

	currentFrame = sequence->firstFrame;

	while (currentFrame->nextFrame != NULL) // for each frame of the sequence except the last one
	{
		printf("Matching points between frames %d and %d\n",
		   currentFrame->id, currentFrame->nextFrame->id);

		printf("  - research of putative correspondences... ");
		fflush(stdout);

		findCorrespondences(currentFrame); // find putative correspondences between
									   // this frame and the next frame

		printf("done (%d correspondences found)\n", currentFrame->nbMatchPoints);

		printf("  - computation of a homography using RANSAC... ");
		fflush(stdout);

		int nbInliers = RANSAC(currentFrame); // estimate a homography between this frame
										  // and the next frame using the RANSAC estimator

		printf("done (%d inliers found)\n", nbInliers);

		showMatchingResults(currentFrame);

		printf("  - optimization of the homography and guided matching... ");
		fflush(stdout);

		int nbCycles = optimHGuidMatchCycle(currentFrame); // cycle of optimization of the homography
													   // and research of further point matches

		printf("done (%d inliers found - %d cycle(s))\n", currentFrame->nbMatchPoints, nbCycles);

		showMatchingResults(currentFrame);

		currentFrame = currentFrame->nextFrame;
	}

	showMatchingResults(sequence->lastFrame);
}