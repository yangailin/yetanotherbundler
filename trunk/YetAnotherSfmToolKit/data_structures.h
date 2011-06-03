#ifndef STRUCTURES_H_
#define STRUCTURES_H_

#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

/// 특징점 추출기에 대한 옵션
#define	HARRIS		0
#define SIFT		1

/* Data structure for a keypoint.  Lists of keypoints are linked
   by the "next" field.
*/
typedef struct KeypointSt {
  float row, col;             /* Subpixel location of keypoint. */
  float scale, ori;           /* Scale and orientation (range [-PI,PI]) */
  unsigned char *descrip;     /* Vector of descriptor values */
  struct KeypointSt *next;    /* Pointer to next keypoint in list. */
} *Keypoint;

typedef struct sequence_t
{
  /// head of the list of frames
  struct frame_t *firstFrame; 
    
  /// tail of the list of frames
  struct frame_t *lastFrame;
  
  /// array containing the number of correspondences between each pair of frames
  int **nbCorrBtwnFrames;
  
  /// number of frames in the sequence
  int nbFrames; 
  
} Sequence;

typedef struct frame_t
{
  /// frame image
  IplImage *image;
  
  /// gray level matrix corresponding to the image
  CvMat *grayImage;  
  
  /// next frame in the sequence
  struct frame_t *nextFrame;
  
  /// head of the list of Harris corners
  struct corner_t *firstPoint;
  
  /// tail of the list of Harris corners
  struct corner_t *lastPoint;
  
  /// homography matrix
  CvMat *H;
  
  /// camera projection matrix
  CvMat *P;
  
  /// frame number in the sequence
  int id;
  
  /// number of Harris corners
  int nbPoints;
  
  /// number of correspondences between the frame and the next frame
  int nbMatchPoints;
  
  /// flag specifying if the frame is a key frame or not
  bool isKeyFrame;
  
} Frame;

typedef struct corner_t
{
  /// 2D (i.e. image) coordinates of the corner
  CvMat *imagePoint;
  
  /// next corner of the frame
  struct corner_t *nextPoint;
  
  /// matching corner in the next frame
  struct corner_t *matchNextFrame;
  
  /// matching corner in the previous frame
  struct corner_t *matchPrevFrame;
  
  /// 3D (i.e. world) coordinates of the corner
  CvMat *worldPoint;
  
  /// SSD value between the corner and the matching corner in the previous frame
  double SSDMatchPrevFrame;
  
  /// persistence of the corner in the previous frames of the sequence
  int nbMatchBackward;
  
  /// persistence of the corner in the next frames of the sequence
  int nbMatchForward;
  
  /// true if the corner matches with a corner in the next frame consistently with a homography
  bool isInlier;
  
  /// true if the corner has been selected in the random sample used to compute a homography
  bool isFromSample;

  /// For the SIFT and its variant (feature transform algorithm)
  //float scale;
  //float orientation;
  //unsigned char *descriptor;

  int featureType; /// Harris, SIFT, etc.

  Keypoint siftKey;
  
} Corner;

typedef struct homography_t
{
  /// homography matrix
  CvMat *H;
  
  /// head of the list of inliers
  struct inlier_t *firstInlier;
  
  /// tail of the list of inliers
  struct inlier_t *lastInlier;
  
  /// residual error of the homography
  double residualError;
  
  /// number of inliers
  int nbInliers;
  
} Homography;

typedef struct inlier_t
{
  /// inlier corner point (the correspondence with its match is consistent with the homography)
  struct corner_t *point;
  
  /// residual error of the inlier
  double residualError;
  
  /// next element of the list of inliers
  struct inlier_t *nextInlier;
  
} Inlier;


/// Function prototypes
Sequence *createSequence();
Frame *createFrame(IplImage *image);
struct corner_t *createCorner(int x, int y);
struct corner_t *createCornerSIFT(float x,float y,Keypoint siftKey);
struct inlier_t *createInlier(struct corner_t *point, double residualError);
struct homography_t *createHomography(CvMat *H);

void addFrame(Frame *frame, Sequence *list);
void addCorner(struct corner_t *corner, struct frame_t *list);
void addInlier(struct inlier_t *inlier, struct homography_t *list);

void releaseSequence(struct sequence_t *sequence);
void releaseFrame(struct frame_t *frame);
void releaseCorner(struct corner_t *corner);
void releaseHomography(struct homography_t *homography);
void releaseInlier(struct inlier_t *inlier);

#endif