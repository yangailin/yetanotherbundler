#include <stdio.h>
#include <cv.h>
#include <highgui.h>

void setWarpedROI(IplImage* new_frame, IplImage* WarpedFrame, CvMat* homogCollection)
{
	CvMat* corner_upper_left = cvCreateMat(3,1, CV_64FC1);
	cvSet1D(corner_upper_left, 0, cvRealScalar(0));
	cvSet1D(corner_upper_left, 1, cvRealScalar(0));
	cvSet1D(corner_upper_left, 2, cvRealScalar(1));
	
	CvMat* corner_upper_right = cvCreateMat(3,1, CV_64FC1);
	cvSet1D(corner_upper_right, 0, cvRealScalar(new_frame->width));
	cvSet1D(corner_upper_right, 1, cvRealScalar(0));
	cvSet1D(corner_upper_right, 2, cvRealScalar(1));

	CvMat* corner_lower_left = cvCreateMat(3,1, CV_64FC1);
	cvSet1D(corner_lower_left, 0, cvRealScalar(0));
	cvSet1D(corner_lower_left, 1, cvRealScalar(new_frame->height));
	cvSet1D(corner_lower_left, 2, cvRealScalar(1));

	CvMat* corner_lower_right = cvCreateMat(3,1, CV_64FC1);
	cvSet1D(corner_lower_right, 0, cvRealScalar(new_frame->width));
	cvSet1D(corner_lower_right, 1, cvRealScalar(new_frame->height));
	cvSet1D(corner_lower_right, 2, cvRealScalar(1));
	
	
	CvMat* corner_upper_left_warped = cvCreateMat(3,1, CV_64FC1);
	CvMat* corner_upper_right_warped = cvCreateMat(3,1, CV_64FC1);
	CvMat* corner_lower_left_warped = cvCreateMat(3,1, CV_64FC1);
	CvMat* corner_lower_right_warped = cvCreateMat(3,1, CV_64FC1);

	CvMat* translation = cvCreateMat(3,1,CV_64FC1);
	cvSet1D(translation,0,cvRealScalar(cvmGet(homogCollection,0,2)));
	cvSet1D(translation,1,cvRealScalar(cvmGet(homogCollection,1,2)));
	cvSet1D(translation,2,cvRealScalar(0));

	printf("%f,%f,%f\n\n",cvmGet(translation,0,0),cvmGet(translation,1,0),cvmGet(translation,2,0));

	for(int i=0;i<3;i++)
	{
		for(int j=0;j<3;j++)
		{
			printf("%f ",cvmGet(homogCollection,i,j));
		}
		printf("\n");
	}
	printf("\n");

#if 1
	cvMatMulAdd(homogCollection,corner_upper_left,translation,corner_upper_left_warped);
	cvMatMulAdd(homogCollection,corner_upper_right,translation,corner_upper_right_warped);
	cvMatMulAdd(homogCollection,corner_lower_left,translation,corner_lower_left_warped);
	cvMatMulAdd(homogCollection,corner_lower_right,translation,corner_lower_right_warped);
#else
	cvMatMulAdd(homogCollection,corner_upper_left,0,corner_upper_left_warped);
	cvMatMulAdd(homogCollection,corner_upper_right,0,corner_upper_right_warped);
	cvMatMulAdd(homogCollection,corner_lower_left,0,corner_lower_left_warped);
	cvMatMulAdd(homogCollection,corner_lower_right,0,corner_lower_right_warped);
#endif
	
	CvPoint ul = cvPoint((int)(cvGet1D(corner_upper_left_warped, 0).val[0]/cvGet1D(corner_upper_left_warped, 2).val[0]), (int)(cvGet1D(corner_upper_left_warped, 1).val[0]/cvGet1D(corner_upper_left_warped, 2).val[0]));
	//cvCircle(WarpedFrame, ul,6, CV_RGB(255,255,0), 1, CV_AA,0);
	CvPoint ur = cvPoint((int)(cvGet1D(corner_upper_right_warped, 0).val[0]/cvGet1D(corner_upper_right_warped, 2).val[0]), (int)(cvGet1D(corner_upper_right_warped, 1).val[0]/cvGet1D(corner_upper_right_warped, 2).val[0]));
	//cvCircle(WarpedFrame, ur,6, CV_RGB(255,0,0), 1, CV_AA,0);
	CvPoint lr = cvPoint((int)(cvGet1D(corner_lower_right_warped, 0).val[0]/cvGet1D(corner_lower_right_warped, 2).val[0]), (int)(cvGet1D(corner_lower_right_warped, 1).val[0]/cvGet1D(corner_lower_right_warped, 2).val[0]));
	//cvCircle(WarpedFrame, lr,6, CV_RGB(0,255,0), 1, CV_AA,0);
	CvPoint ll = cvPoint((int)(cvGet1D(corner_lower_left_warped, 0).val[0]/cvGet1D(corner_lower_left_warped, 2).val[0]), (int)(cvGet1D(corner_lower_left_warped, 1).val[0]/cvGet1D(corner_lower_left_warped, 2).val[0]));
	//cvCircle(WarpedFrame, ll,6, CV_RGB(0,0,255), 1, CV_AA,0);
	
	CvRect roiRect = cvRect(max(ul.x, ll.x),max(ul.y, ur.y), min(ur.x, lr.x)-max(ul.x, ll.x), min(lr.y, ll.y)-max(ul.y, ur.y));
	//cvRectangle(WarpedFrame,cvPoint(max(ul.x, ll.x),max(ul.y, ur.y)), cvPoint(min(ur.x, lr.x),min(lr.y, ll.y)), CV_RGB(255,255,0), 1, CV_AA,0); 
	
	cvSetImageROI(WarpedFrame, roiRect);
	
			cvReleaseMat(&corner_upper_left);
			cvReleaseMat(&corner_lower_right);
			cvReleaseMat(&corner_lower_left);
			cvReleaseMat(&corner_lower_right);
			cvReleaseMat(&corner_lower_left_warped);
			cvReleaseMat(&corner_lower_right_warped);
			cvReleaseMat(&corner_upper_left_warped);
			cvReleaseMat(&corner_upper_right_warped);
}

int main(int argc, char* argv[])
{
	int frameNum = 29;
	char fileName[0xff];

	IplImage** frames = new IplImage*[frameNum];
	CvMat** H = new CvMat*[frameNum];

	for(int i=0;i<frameNum;i++)
	{
		sprintf(fileName, "data\\img%.3d.jpeg",i);

		if(!(frames[i] = cvLoadImage(fileName)))
		{
			break;
		}

		sprintf(fileName,"data\\img%.3d_H_mat.xml",i);

		if(!(H[i] = (CvMat*)cvLoad(fileName)))
		{
			break;
		}
	}

	CvSize size = cvGetSize(frames[0]);
	size.height*= 2;
	size.width*= 2;

	IplImage *WarpedFrame = cvCreateImage(size,8,3);

	

	cvNamedWindow("Hi");

	//while(true)
	//{
		CvMat *HIdentity = (CvMat *)cvLoad("data\\Hidentity.xml");

		//for(int i=0;i<frameNum;i++)
		for(int i=frameNum-1;i>0;i--)
		{
			//setWarpedROI(frames[i],WarpedFrame,HIdentity);
			//setWarpedROI(frames[i],WarpedFrame,H[i]);

			if ( (i == (frameNum-1)) || (i==1))
			cvWarpPerspective(frames[i],WarpedFrame,HIdentity);
			
			cvMatMulAdd(HIdentity,H[i],0,HIdentity);  // cvMatMulAdd(a,b,c,dst), dst = a*b+c
			//cvMatMulAdd(H[i],HIdentity,0,HIdentity);  // cvMatMulAdd(a,b,c,dst), dst = a*b+c

			///////////////
			/// TOOD:
			/// Normalize!
			///////////////

			cvResetImageROI(WarpedFrame);
			cvShowImage("Hi",WarpedFrame);
			cvWaitKey(1);
		}
	//}

	cvWaitKey(0);

#if 0
 	IplImage* im1 = cvLoadImage("data\\img000.jpeg");
	IplImage* im2 = cvLoadImage("data\\img001.jpeg");

	CvSize size = cvGetSize(im1);
	size.height*= 2;
	size.width*= 2;

	IplImage *WarpedFrame = cvCreateImage(size,8,3);

	// ÀÌ°Å ¹ºÁö ÆÄ¾ÇÇØº¸ÀÚ.
	WarpedFrame->origin = im1->origin;

	CvMat* H = (CvMat *)cvLoad("data\\img000_H_mat.xml");

	CvMat *HIdentity = (CvMat *)cvLoad("data\\Hidentity.xml");

	CvMat *homography_center = cvCreateMat(3,3,CV_64FC1);

	cvSet2D(homography_center,0, 0, cvRealScalar(1));
	cvSet2D(homography_center,0, 1, cvRealScalar(0));
	cvSet2D(homography_center,0, 2, cvRealScalar(WarpedFrame->width/4));
	cvSet2D(homography_center,1, 0, cvRealScalar(0));
	cvSet2D(homography_center,1, 1, cvRealScalar(1));
	cvSet2D(homography_center,1, 2, cvRealScalar(WarpedFrame->height/4));
	cvSet2D(homography_center,2, 0, cvRealScalar(0));
	cvSet2D(homography_center,2, 1, cvRealScalar(0));
	cvSet2D(homography_center,2, 2, cvRealScalar(1));

	cvWarpPerspective(im1,WarpedFrame,HIdentity);

	setWarpedROI(im2,WarpedFrame,H);

	cvWarpPerspective(im2,WarpedFrame,H);

	cvNamedWindow("Hi");

	cvResetImageROI(WarpedFrame);

	cvShowImage("Hi",WarpedFrame);

	cvWaitKey(0);
#endif
	return 0;
}