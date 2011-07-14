#include "surf.h"

void findSURF(Frame *frame,char* keyFileName)
{
	Keypoint k = NULL;
	Keypoint kPt = NULL;

	kPt = ReadSURFKeyFile(keyFileName);

	/// 모든 링크드리스트를 방문하면서, corner를 등록한다.
	for (k= kPt; k != NULL; k = k->next)
	{
		//printf("%x\n",&k->next);
		Corner *point = createCornerSIFT(k->col,k->row,k);
		addCorner(point, frame); // add point in the list
	}
}

Keypoint ReadSURFKeyFile(char *filename)
{
    FILE *file;

    file = fopen (filename, "r");
    if (! file)
	FatalError("Could not open file: %s", filename);

    return ReadSURFKeys(file);
}

Keypoint ReadSURFKeys(FILE *fp)
{
	 int i, j, num, len, val;
	 float desc_val;
    Keypoint k, keys = NULL;
	float x,y,a,b,c;

	if (fscanf(fp, "%d\n", &len) != 1)
		FatalError("Invalid keypoint file beginning.");

	if (fscanf(fp, "%d\n", &num) != 1)
		FatalError("Invalid keypoint file beginning.");

	for (i = 0; i < num; i++) 
	{
		/* Allocate memory for the keypoint. */
		k = (Keypoint) malloc(sizeof(struct KeypointSt));
		k->next = keys;
		keys = k;

		k->descrip = (float*)malloc(len*sizeof(float));
		k->descriptor_size = len;

		//if (fscanf(fp, "%f %f %f %f %f", &(k->row), &(k->col), &dummy, &(k->scale),&(k->ori)) != 5)
		if (fscanf(fp, "%f %f %f %f %f",&x,&y,&a,&b,&c) != 5)
			FatalError("Invalid keypoint file format.");

		float det = sqrt((a-c)*(a-c) + 4.0*b*b);
		float e1 = 0.5*(a+c + det);
		float e2 = 0.5*(a+c - det);
		float l1 = (1.0/sqrt(e1));
		float l2 = (1.0/sqrt(e2));
		float sc = sqrt( l1*l2 );

		k->row = y;
		k->col = x;
		k->scale = sc/2.5;

		for (j = 0; j < len; j++) 
		{
			if (fscanf(fp, "%f", &desc_val) != 1)
				FatalError("Invalid keypoint file value.");

			k->descrip[j] = desc_val;
		}
	}

	/// Hyon Lim add this.
	fclose(fp);

	/// Linked list of sift desciptor that contains, loc, scale, ori, 128-dim is returned.
    return keys;
}