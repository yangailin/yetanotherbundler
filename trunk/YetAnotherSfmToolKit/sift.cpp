#include "sift.h"

/* Given a pair of images and their keypoints, pick the first keypoint
   from one image and find its closest match in the second set of
   keypoints.  Then write the result to a file.
*/
void matchSIFT(Frame *frame, CvMat *H)
{
	//Keypoint k, match;
	//Keypoint minkey = NULL;
	int count = 0;
	int dsq, distsq1 = 100000000, distsq2 = 100000000;

	Corner *correspondence = NULL;

	Corner *currentPointFrame1 = frame->firstPoint;

	while (currentPointFrame1 != NULL) // for each corner point in the first frame
	{
		Keypoint k1 = currentPointFrame1->siftKey;

		/// follow the list!
		Corner *currentPointFrame2 = frame->nextFrame->firstPoint;


		/// Initialization
		distsq1 = 100000000, distsq2 = 100000000;

		while (currentPointFrame2 != NULL) // for each corner point in the second frame
		{
			Keypoint k2 = currentPointFrame2->siftKey;

			/// do something useful
			dsq = DistSquared(k1, k2);

			if (dsq < distsq1)
			{
				distsq2 = distsq1;
				distsq1 = dsq;
				//minkey = k2;
				correspondence = currentPointFrame2;
			} 
			else if (dsq < distsq2) 
			{
				distsq2 = dsq;
			}

			currentPointFrame2 = currentPointFrame2->nextPoint; 
		}
		
		/* Check whether closest distance is less than 0.6 of second. */
		if (10 * 10 * distsq1 < 6 * 6 * distsq2)
		{
			// Matched. minkey is correspondence
			// the point is a potential correspondence
			if (correspondence->matchPrevFrame == NULL) // no other candidate
			{
				if (currentPointFrame1->matchNextFrame != NULL)
				{
				  currentPointFrame1->matchNextFrame->matchPrevFrame = NULL;
				  frame->nbMatchPoints--;
				}

				currentPointFrame1->matchNextFrame = correspondence;  
				correspondence->matchPrevFrame = currentPointFrame1;
				frame->nbMatchPoints++;
			}
		}

		currentPointFrame1 = currentPointFrame1->nextPoint;

		printf("%d\n",count++);
	}

#if 0
	Corner *currentPointFrame1 = frame->firstPoint;
	Corner *currentPointFrame2 = frame->nextFrame->firstPoint;

	for (k= currentPointFrame1->siftKey ; k != NULL; k = k->next)
	{
		if (H != NULL) // if a homography is given
        {
			// do something useful.
		}

		match = CheckForMatch(k, currentPointFrame2->siftKey);

		if (match != NULL) /// 검색된 점이 있는 경우, match 변수는 Keypoint 구조체임.
		{
			// 다음 프레임 정보에, 이전 영상과의 연관이 없다. 매칭이 찾아진적이 없는 경우.
			if(currentPointFrame2->matchPrevFrame == NULL)
			{
				// 현재 프레임에서, 다음영상과의 매칭이 있다?, 뭔가 잘못됨. 리셋.
				if (currentPointFrame1->matchNextFrame != NULL)
				{
					currentPointFrame1->matchNextFrame->matchPrevFrame = NULL;
					frame->nbMatchPoints--;
				}

				/// 관계설정.
				currentPointFrame1->matchNextFrame = currentPointFrame2;  /// 현재프레임에서 다음영상의 매칭
				currentPointFrame2->matchPrevFrame = currentPointFrame1;  /// 다음 프레임에서 이전 영상과의 매칭
				frame->nbMatchPoints++;	/// 찾았다고 등록함.
			}

			count++;
		}
	}
#endif
}

void findSIFT(Frame *frame,char* keyFileName)
{
	Keypoint k = NULL;
	Keypoint kPt = NULL;

	kPt = ReadKeyFile(keyFileName);

	for (k= kPt; k != NULL; k = k->next)
	{
		Corner *point = createCornerSIFT(k->col,k->row,kPt);
		addCorner(point, frame); // add point in the list
	}
}

Keypoint ReadKeyFile(char *filename)
{
    FILE *file;

    file = fopen (filename, "r");
    if (! file)
	FatalError("Could not open file: %s", filename);

    return ReadKeys(file);
}

/* Read keypoints from the given file pointer and return the list of
   keypoints.  The file format starts with 2 integers giving the total
   number of keypoints and the size of descriptor vector for each
   keypoint (currently assumed to be 128). Then each keypoint is
   specified by 4 floating point numbers giving subpixel row and
   column location, scale, and orientation (in radians from -PI to
   PI).  Then the descriptor vector for each keypoint is given as a
   list of integers in range [0,255].

*/
Keypoint ReadKeys(FILE *fp)
{
    int i, j, num, len, val;
    Keypoint k, keys = NULL;

    if (fscanf(fp, "%d %d", &num, &len) != 2)
	FatalError("Invalid keypoint file beginning.");

    if (len != 128)
	FatalError("Keypoint descriptor length invalid (should be 128).");

    for (i = 0; i < num; i++) {
      /* Allocate memory for the keypoint. */
      k = (Keypoint) malloc(sizeof(struct KeypointSt));
      k->next = keys;
      keys = k;
      k->descrip = (unsigned char*)malloc(len);

      if (fscanf(fp, "%f %f %f %f", &(k->row), &(k->col), &(k->scale),
		 &(k->ori)) != 4)
	FatalError("Invalid keypoint file format.");

      for (j = 0; j < len; j++) {
	if (fscanf(fp, "%d", &val) != 1 || val < 0 || val > 255)
	  FatalError("Invalid keypoint file value.");
	k->descrip[j] = (unsigned char) val;
      }
    }
    return keys;
}


/* This searches through the keypoints in klist for the two closest
   matches to key.  If the closest is less than 0.6 times distance to
   second closest, then return the closest match.  Otherwise, return
   NULL.
*/
Keypoint CheckForMatch(Keypoint key, Keypoint klist)
{
    int dsq, distsq1 = 100000000, distsq2 = 100000000;
    Keypoint k, minkey = NULL;

    /* Find the two closest matches, and put their squared distances in
       distsq1 and distsq2.
    */
    for (k = klist; k != NULL; k = k->next) {
      dsq = DistSquared(key, k);

      if (dsq < distsq1) {
	distsq2 = distsq1;
	distsq1 = dsq;
	minkey = k;
      } else if (dsq < distsq2) {
	distsq2 = dsq;
      }
    }

    /* Check whether closest distance is less than 0.6 of second. */
    if (10 * 10 * distsq1 < 6 * 6 * distsq2)
      return minkey;
    else return NULL;
}


/* Return squared distance between two keypoint descriptors.
*/
int DistSquared(Keypoint k1, Keypoint k2)
{
    int i, dif, distsq = 0;
    unsigned char *pk1, *pk2;

    pk1 = k1->descrip;
    pk2 = k2->descrip;

    for (i = 0; i < 128; i++) {
      dif = (int) *pk1++ - (int) *pk2++;
      distsq += dif * dif;
    }
    return distsq;
}

void FatalError(char *fmt, ...)
{
    va_list args;

    va_start(args, fmt);
    fprintf(stderr, "Error: ");
    vfprintf(stderr, fmt, args);
    fprintf(stderr,"\n");
    va_end(args);
    exit(1);
}