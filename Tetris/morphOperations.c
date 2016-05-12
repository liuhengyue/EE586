#include "morphOperations.h"

int morphDilation(unsigned char *image, unsigned char *outImage, int mask_type, int morphThreshold){

int w = buffWidth; // the rows and the cols of image
int h = buffHeight;

int i, j, a, b; // various counters
int max; // for dilation operation
int count; // count the hits
//Uint8 outImage  = image; // initialize the output image
short mask[3][3];
short edmask1[3][3] = {{0, 1, 0},{0, 1, 0},{0, 1, 0}};
short edmask2[3][3] = {{0, 0, 0},{1, 1, 1},{0, 0, 0}};
short edmask3[3][3] = {{0, 1, 0},{1, 1, 1},{0, 1, 0}};
short edmask4[3][3] = {{1, 1, 1},{1, 1, 1},{1, 1, 1}};

// dilation1 based on the masks
if (morphThreshold == 0 && mask_type != 0){

	// choose the mask according to the mask_type
	switch(mask_type){
	  case 1:
	     memcpy(mask,edmask1,sizeof(mask));
	     break;
	  case 2:
	     memcpy(mask,edmask2,sizeof(mask));
	     break;
	  case 3:
	     memcpy(mask,edmask3,sizeof(mask));
	     break;
	  case 4:
	     memcpy(mask,edmask4,sizeof(mask));
	     break;
	  default:
	     //printf("\nInvalid mask type, using mask 4");
	     memcpy(mask,edmask4,sizeof(mask));
	     break;
	}

  for(i = 1; i < h-1; ++i){
    for(j = 1; j < w-1; ++j){
      max  = 0;
      for(a = -1; a <=1; a++){
        for(b = -1; b <= 1; b++){
          if(mask[a+1][b+1] == 1){
            if(image[(i+a)*w+(j+b)] > max)
              max = image[(i+a)*w+(j+b)];
          }
        }
      }
      outImage[i*w+j] = max;
    }
  } 
}

// dilation2 based on the counts of hit
else if (morphThreshold != 0 && mask_type == 0){
	int value = 255; // the up value of binary image 
  	for(i = 1; i < h-1; ++i){
    	for(j = 1; j < w-1; ++j){
      		count  = 0;
      		for(a = -1; a <=1; a++){
        		for(b = -1; b <= 1; b++){
          			if(mask[a+1][b+1] == 1){
            			if(image[(i+a)*w+(j+b)] == value)
              				count++ ;
          		}
        	}
      	}
      if(count > morphThreshold) outImage[i*w+j] = value;
    }
  } 
}
else return -1;

	

return 0;
}
//Morphological erosion
int morphErosion(unsigned char *image, unsigned char *outImage, int mask_type, int morphThreshold){

int w = buffWidth; // the rows and the cols of image
int h = buffHeight;

int i, j, a, b; // various counters
int min; // for erosion
int count; // count the hits
short mask[3][3];
short edmask1[3][3] = {{0, 1, 0},{0, 1, 0},{0, 1, 0}};
short edmask2[3][3] = {{0, 0, 0},{1, 1, 1},{0, 0, 0}};
short edmask3[3][3] = {{0, 1, 0},{1, 1, 1},{0, 1, 0}};
short edmask4[3][3] = {{1, 1, 1},{1, 1, 1},{1, 1, 1}};
// erosion based on mask
if (morphThreshold == 0 && mask_type != 0){

  // choose the mask according to the mask_type
  switch(mask_type){
    case 1:
	     memcpy(mask,edmask1,sizeof(mask));
	     break;
	  case 2:
	     memcpy(mask,edmask2,sizeof(mask));
	     break;
	  case 3:
	     memcpy(mask,edmask3,sizeof(mask));
	     break;
	  case 4:
	     memcpy(mask,edmask4,sizeof(mask));
	     break;
    default:
       //printf("\nInvalid mask type, using mask 4");
       memcpy(mask,edmask4,sizeof(mask));
       break;
  }

  for(i = 1; i < h-1; ++i){
    for(j = 1; j < w-1; ++j){
      min  = 255;
      for(a = -1; a <=1; a++){
        for(b = -1; b <= 1; b++){
          if(mask[a+1][b+1] == 1){
            if(image[(i+a)*w+(j+b)] < min)
              min = image[(i+a)*w+(j+b)];
          }
        }
      }
      outImage[i*w+j] = min;
    }
  } 
}

// erosion based on counts
else if (morphThreshold != 0 && mask_type == 0){
  for(i = 1; i < h-1; ++i){
    for(j = 1; j < w-1; ++j){
      count  = 0;
      for(a = -1; a <=1; a++){
        for(b = -1; b <= 1; b++){
          if(mask[a+1][b+1] == 1){
            if(image[(i+a)*w+(j+b)] == 0)
              count++;
          }
        }
      }
      if(count > morphThreshold) outImage[i*w+j] = 0;
    }
  } 
}

else{
  //printf("\nInvalid threshold and mask_type");
  return -1;
}

return 0;
}

