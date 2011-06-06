#include <stdio.h>
#include "sfm.h"
#include "data_structures.h"
#include "matching.h"
#include "visualization.h"

void SfM(char *seqPath, char *KMatPath,int featureExtractor)
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
	char siftKeyName[0xff];
	char cmdArgs[0xff];	/// Command argument for SIFT
	int result;

	IplImage *image;
	Frame *currentFrame;

	/************************************************************************************
	 * Extract possible corners in a frame. Peform to all images in the sequence.
	************************************************************************************/
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

		/// Select which corner extractor is implemented.
		if(featureExtractor == HARRIS)
		{
			findCorners(currentFrame); // detect corner points in the current image
		}
		else if(featureExtractor == SIFT) /// SIFT Related function
		{
			/// siftWin32.exe
			sprintf(fileName,"%s/img%.3d.pgm",seqPath,sequence->nbFrames);
			sprintf(siftKeyName,"%s/img%.3d.key",seqPath,sequence->nbFrames);
			sprintf(cmdArgs,"sift_bin\\siftWin32.exe <%s > %s",fileName,siftKeyName);

			// Test wheter the key file exist or not.
			FILE *testfp = fopen(siftKeyName,"r");

			/// 만약에 파일이 있으면 굳이 읽어들이지 않는다.
			if(testfp != NULL)
			{
				/// 파일 용량을 얻어오는 부분
				fseek(testfp, 0L, SEEK_END); 
				long sz = ftell(testfp); 

				/// 파일이 완벽하지 못할때 한번더 수행한다.
				if(sz == 0)
				{
					fclose(testfp);
					result = system(cmdArgs);
				}
			}
			else
			{
				result = system(cmdArgs);
			}

			/// do more job please. as same as findCorners.
			findSIFT(currentFrame,siftKeyName);
		}

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

		if(featureExtractor == HARRIS)
		{
			findCorrespondences(currentFrame); // find putative correspondences between
											   // this frame and the next frame
		}
		else if(featureExtractor == SIFT) /// SIFT Related function
		{
			// do sift related match.
			matchSIFT(currentFrame);
		}

		printf("done (%d correspondences found)\n", currentFrame->nbMatchPoints);

		printf("  - computation of a homography using RANSAC... ");
		fflush(stdout);

		int nbInliers = RANSAC(currentFrame); // estimate a homography between this frame
										  // and the next frame using the RANSAC estimator

		printf("done (%d inliers found)\n", nbInliers);

		showMatchingResults(currentFrame);

		printf("  - optimization of the homography and guided matching... ");
		fflush(stdout);

		/// Here is the problem.
		int nbCycles = optimHGuidMatchCycle(currentFrame,featureExtractor); // cycle of optimization of the homography
													   // and research of further point matches

		printf("done (%d inliers found - %d cycle(s))\n", currentFrame->nbMatchPoints, nbCycles);

		showMatchingResults(currentFrame);

		currentFrame = currentFrame->nextFrame;
	}

	showMatchingResults(sequence->lastFrame);

	//***************************************************************
	//* Reconstruction
	//***************************************************************

	KeyFramesList *keyFrames = createKeyFramesList();

	printf("Selection of key frames... ");
	fflush(stdout);
}