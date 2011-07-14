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
		else if(featureExtractor == SURF)
		{
			sprintf(fileName,"%s/img%.3d.pgm",seqPath,sequence->nbFrames);
			sprintf(siftKeyName,"%s/img%.3d.surf",seqPath,sequence->nbFrames);
			sprintf(cmdArgs,"sift_bin\\surfWINDLLDemo.exe -i %s -o %s",fileName,siftKeyName);
			
			FILE *testfp = fopen(siftKeyName,"r");

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

			findSURF(currentFrame,siftKeyName);
		}

		printf("done (%d corners found)\n", currentFrame->nbPoints);

		addFrame(currentFrame, sequence);

		//////////////////////////////////////////////////////////////////
		/// TODO: In here, save data file of extracted corners structure.
		//////////////////////////////////////////////////////////////////
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
			matchSIFT(currentFrame);
		}
		else if(featureExtractor == SURF)
		{
			matchSIFT(currentFrame);
		}

		printf("done (%d correspondences found)\n", currentFrame->nbMatchPoints);

		printf("  - computation of a homography using RANSAC... ");
		fflush(stdout);

		int nbInliers = RANSAC(currentFrame);	// estimate a homography between this frame
												// and the next frame using the RANSAC estimator

		printf("done (%d inliers found)\n", nbInliers);

		if(featureExtractor == HARRIS)
		{
			sprintf(fileName,"%s/HARRIS_features_RANSAC_%.3d.jpg",seqPath,currentFrame->id);
		}
		else if(featureExtractor == SIFT) /// SIFT Related function
		{
			sprintf(fileName,"%s/SIFT_features_RANSAC_%.3d.jpg",seqPath,currentFrame->id);
		}
		else if(featureExtractor == SURF)
		{
			sprintf(fileName,"%s/SURF_features_RANSAC_%.3d.jpg",seqPath,currentFrame->id);
		}

		/// 특징점 이미지 저장.
		showMatchingResults(currentFrame,fileName);

		/// 여기에서 Optimization 하기 전꺼 저장.
		if(featureExtractor == HARRIS)
		{
			sprintf(fileName,"%s/HARRIS_H_%.3d_%.3d.xml",seqPath,currentFrame->nextFrame->id,currentFrame->id);
		}
		else if(featureExtractor == SIFT)
		{
			sprintf(fileName,"%s/SIFT_H_%.3d_%.3d.xml",seqPath,currentFrame->nextFrame->id,currentFrame->id);
		}
		else if(featureExtractor == SURF)
		{
			sprintf(fileName,"%s/SURF_H_%.3d_%.3d.xml",seqPath,currentFrame->nextFrame->id,currentFrame->id);
		}
		
		cvSave(fileName,currentFrame->H);

		// H_001_002 format. This H transforms X2 = H_{21} X1;
		if(featureExtractor == HARRIS)
		{
			sprintf(fileName,"%s/HARRIS_H_%.3d_%.3d.txt",seqPath,currentFrame->nextFrame->id,currentFrame->id);
		}
		else if(featureExtractor == SIFT)
		{
			sprintf(fileName,"%s/SIFT_H_%.3d_%.3d.txt",seqPath,currentFrame->nextFrame->id,currentFrame->id);
		}
		else if(featureExtractor == SURF)
		{
			sprintf(fileName,"%s/SURF_H_%.3d_%.3d.txt",seqPath,currentFrame->nextFrame->id,currentFrame->id);
		}


		if(saveCvMat2MATLAB(currentFrame->H, fileName) < 0)
		{
			printf("Save Error!\n");
		}
		else
		{
			printf("H matrix for MATLAB is saved!");
		}

		if(featureExtractor == HARRIS)
		{
			sprintf(fileName,"%s/HARRIS_pts_match_from_%.3d_to_%.3d.txt",seqPath,currentFrame->id,currentFrame->nextFrame->id);
		}
		else if(featureExtractor == SIFT)
		{
			sprintf(fileName,"%s/SIFT_pts_match_from_%.3d_to_%.3d.txt",seqPath,currentFrame->id,currentFrame->nextFrame->id);
		}
		else if(featureExtractor == SURF)
		{
			sprintf(fileName,"%s/SURF_pts_match_from_%.3d_to_%.3d.txt",seqPath,currentFrame->id,currentFrame->nextFrame->id);
		}

		if(saveInliersPts2MATLAB(currentFrame,fileName) < 0)
		{
			printf("Save inlier error!\n");
		}
		else
		{
			printf("Inlier list is saved. <x1.x, x1.y, x2.x, x2.y>\n");
		}

		printf("  - optimization of the homography and guided matching... ");
		fflush(stdout);

#if 0
		/// Here is the problem.
		int nbCycles = optimHGuidMatchCycle(currentFrame,featureExtractor); // cycle of optimization of the homography
																			// and research of further point matches

		printf("done (%d inliers found - %d cycle(s))\n", currentFrame->nbMatchPoints, nbCycles);

		//////////////////////////////////////////////////////////////////
		/// TODO: Save obtaiined homography in XML files.
		//////////////////////////////////////////////////////////////////
		if(featureExtractor == HARRIS)
		{
			sprintf(fileName,"%s/Harris_H_%.3d_%.3d_optimized.xml",seqPath,currentFrame->nextFrame->id,currentFrame->id);
		}
		else if(featureExtractor == SIFT)
		{
			sprintf(fileName,"%s/SIFT_H_%.3d_%.3d_optimized.xml",seqPath,currentFrame->nextFrame->id,currentFrame->id);
		}
		
		cvSave(fileName,currentFrame->H);

		// H_001_002 format. This H transforms X2 = H_{21} X1;
		if(featureExtractor == HARRIS)
		{
			sprintf(fileName,"%s/Harris_H_%.3d_%.3d_optimized.txt",seqPath,currentFrame->nextFrame->id,currentFrame->id);
		}
		else if(featureExtractor == SIFT)
		{
			sprintf(fileName,"%s/SIFT_H_%.3d_%.3d_optimized.txt",seqPath,currentFrame->nextFrame->id,currentFrame->id);
		}

		if(saveCvMat2MATLAB(currentFrame->H, fileName) < 0)
		{
			printf("Save Error!\n");
		}
		else
		{
			printf("H matrix for MATLAB is saved!");
		}

		if(featureExtractor == HARRIS)
		{
			sprintf(fileName,"%s/Harris_pts_match_from_%.3d_to_%.3d_optimized.txt",seqPath,currentFrame->id,currentFrame->nextFrame->id);
		}
		else if(featureExtractor == SIFT)
		{
			sprintf(fileName,"%s/SIFT_pts_match_from_%.3d_to_%.3d_optimized.txt",seqPath,currentFrame->id,currentFrame->nextFrame->id);
		}

		if(saveInliersPts2MATLAB(currentFrame,fileName) < 0)
		{
			printf("Save inlier error!\n");
		}
		else
		{
			printf("Inlier list is saved. <x1.x, x1.y, x2.x, x2.y>\n");
		}

		sprintf(fileName,"%s/features_optimized_%.3d.jpg",seqPath,currentFrame->id);

		showMatchingResults(currentFrame,fileName);
#endif



		currentFrame = currentFrame->nextFrame;
	}

	if(featureExtractor == HARRIS)
	{
		sprintf(fileName,"%s/HARRIS_features_RANSAC_%.3d.jpg",seqPath,currentFrame->id);
	}
	else if(featureExtractor == SIFT) /// SIFT Related function
	{
		sprintf(fileName,"%s/SIFT_features_RANSAC_%.3d.jpg",seqPath,currentFrame->id);
	}
	else if(featureExtractor == SURF)
	{
		sprintf(fileName,"%s/SURF_features_RANSAC_%.3d.jpg",seqPath,currentFrame->id);
	}

	showMatchingResults(sequence->lastFrame,fileName);

	//***************************************************************
	//* Reconstruction
	//***************************************************************

	KeyFramesList *keyFrames = createKeyFramesList();

	printf("Selection of key frames... ");
	fflush(stdout);
}