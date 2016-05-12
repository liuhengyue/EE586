#ifndef MORPHOPERATIONS_H_
#define MORPHOPERATIONS_H_
#include "string.h"
#define buffWidth 720/4
#define buffHeight 480/4
int morphDilation(unsigned char *image, unsigned char *outImage, int mask_type, int morphThreshold);
int morphErosion(unsigned char *image, unsigned char *outImage, int mask_type, int morphThreshold);
#endif /*MORPHOPERATIONS_H_*/
