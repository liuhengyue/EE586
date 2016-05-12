#include "main.h"

//#include<opencv2/opencv.hpp>
extern Int16 video_loopback_test();

// size for buffer_in: 720 * 480 / 2, the reason is explained below. 
#define Pixels 172800
#define PI 3.14159265358979323846
// Resolution 720 * 480 (NTSC mode)
#define vWidth 720
#define vHeight 480
#define BufferNum 30

// Define a space on memory for save the information input and output (Interface data)
Uint32 buffer_out[Pixels]; //from 0x80000000
Uint32 buffer_in[Pixels]; //from 0x800A8C00, which is the same as 4 (bytes for integer) * Pixels
#include "img.cof"
#include "score.cof"//storing number 0-9 and image "score.jpg"
//Define colors
#define BLK 444603008
#define WHITE 3951094656
#define RED 1391481434
#define GREEN 2434961718
#define BLUE 695085552
#define YELLOW 3532837392
#define MAG 1809738698
#define CYAN 2853218982
//Define downsampling rate
#define DOWN 4
#define SIZE 345600/16 //dowsized buffer size
// Intermediate buffer to hold one channel values
Uint8 Buffer_input[vWidth*vHeight];
// Intermediate buffer to processing metadata
Uint8 downsized_buff[vWidth*vHeight/(DOWN*DOWN)];
//downsampled output
Uint8 downsized_out[vWidth*vHeight/(DOWN*DOWN)];
// output buffer to hold processed image
Uint8 Buffer_output[vWidth*vHeight];
//downsampled buffer
//Uint8 downsamples[Pixels / 2];
// Define internal buffer
//Uint32 internal_buffer1[Pixels / BufferNum];
//Uint8 internal_buffer2[2 * Pixels / BufferNum *3];

//Image processing variables
int segmentation_T;
int command;
//kalman filter COG 
kalman1_state statex;
kalman1_state statey;
//kalman filter COF
kalman1_state stateCOFx;
kalman1_state stateCOFy;
//angle
kalman1_state stateAngle;

struct CvPoint
	{
	int x;
	int y;
	};
struct CvPoint COG, COF, preCOG;//hand centroid of gravity, and cnetroid of fingertips
int accumShift=0;//accumulated movement distance
int preAngle=0;//keep record of previous rotation angle
int accumAngle=0;//angle accumulated in a sequence of frames
short fingerDetected=0;//indicator of finger detection
//contour tracing
int pChainCodex[SIZE];
int pChainCodey[SIZE];
float myAngle[SIZE];
int fingertipx[SIZE];
int fingertipy[SIZE];
int handStart_x=10;
int handStart_y=10;
float fingerAngle[100];

//Game engine variables
Uint32 field_left[200];//={444603008};
Uint32 field_right[200];
Uint32 try_left1[200];
Uint32 try_right1[200];
Uint32 disp_left[200];
Uint32 disp_right[200];
Uint32 try_right2[200];
Uint32 field_wr1[200];
Uint32 field_wr2[200];
int block_preL[16]={0};//preview, left
int block_preR[16]={0};
int block_curL[16]={0};
int block_curR[16]={0};//current, right
int block_sudden[16]={0};
int name, name_curL, name_curR, name_preL, name_preR;//0 to 6 for IOJLTSZ
int play, play_L,play_R, timer, lines_L, lines_R, delay, combo_L, combo_R, score_L, score_R;
int sum, b, b2, changeL, changeR, checkR, holect; int shifts, shifts2;
int value1, value2, val_max, y_min, hp, hp2; int target_x, bonus_L, bonus_R;
//^ field values, target_x is the desired drop location
Int16 dip0, dip1, dip2, dip3;
int miniblock[16]={0};//want this global
int block_temp[16]={0};
int x_posL=0; int y_posL=0;
int x_posR0=0; int y_posR0=0;//current position
int x_posR1=0; int y_posR1=0;//depth1
int x_posR2=0; int y_posR2=0;//for depth 2

// Define the position of the data (refer to linker.cmd)
// Internal memory L2RAM ".l2ram" 
// External memory DDR2 ".ddr2"
#pragma DATA_SECTION(buffer_out,".ddr2")
#pragma DATA_SECTION(buffer_in,".ddr2")
#pragma DATA_SECTION(Buffer_input, ".ddr2")
//#pragma DATA_SECTION(Buffer_midput, ".ddr2")
#pragma DATA_SECTION(img, ".ddr2")
#pragma DATA_SECTION(score, ".ddr2")
#pragma DATA_SECTION(num0, ".ddr2")
#pragma DATA_SECTION(num1, ".ddr2")
#pragma DATA_SECTION(num2, ".ddr2")
#pragma DATA_SECTION(num3, ".ddr2")
#pragma DATA_SECTION(num4, ".ddr2")
#pragma DATA_SECTION(num5, ".ddr2")
#pragma DATA_SECTION(num6, ".ddr2")
#pragma DATA_SECTION(num7, ".ddr2")
#pragma DATA_SECTION(num8, ".ddr2")
#pragma DATA_SECTION(num9, ".ddr2")
#pragma DATA_SECTION(field_left, ".ddr2")
#pragma DATA_SECTION(field_right, ".ddr2")
#pragma DATA_SECTION(try_left1, ".ddr2")
#pragma DATA_SECTION(try_right1, ".ddr2")
#pragma DATA_SECTION(try_left2, ".ddr2")
#pragma DATA_SECTION(try_right2, ".ddr2")
#pragma DATA_SECTION(field_wr1, ".ddr2")
#pragma DATA_SECTION(field_wr2, ".ddr2")
#pragma DATA_SECTION(block_preL, ".ddr2")
#pragma DATA_SECTION(block_preR, ".ddr2")
#pragma DATA_SECTION(block_curL, ".ddr2")
#pragma DATA_SECTION(block_curR, ".ddr2")
#pragma DATA_SECTION(miniblock, ".ddr2")
#pragma DATA_SECTION(block_temp, ".ddr2")
#pragma DATA_SECTION(pChainCodex, ".ddr2")
#pragma DATA_SECTION(pChainCodey, ".ddr2")
#pragma DATA_SECTION(fingertipx, ".ddr2")
#pragma DATA_SECTION(fingertipy, ".ddr2")
#pragma DATA_SECTION(myAngle, ".ddr2")
#pragma DATA_SECTION(fingerAngle, ".ddr2")

// buffer_in represents one input frame which consists of two interleaved frames.
// Each 32 bit data has the information for two adjacent pixels in a row.
// Thus, the buffer holds 720/2 integer data points for each row of 2D image and there exist 480 rows.
//
// Format: yCbCr422 ( y1 | Cr | y0 | Cb )
// Each of y1, Cr, y0, Cb has 8 bits
// For each pixel in the frame, it has y, Cb, Cr components
//
// You can generate a lookup table for color conversion if you want to convert to different color space such as RGB.
// Could refer to http://www.fourcc.org/fccyvrgb.php for conversion between yCbCr and RGB
// 

void ycbcr2rgb(Uint32 *buff1, int width, int height, Uint8 *buff2){
	int i,j,temp;
	Uint8 cast[4];
	Uint32 output;
	//convert Uint32 YUV 422 to 4 Uint8 YUV
	for(i=0; i<height; ++i){
		for (j=0; j<width/2; ++j) {
				cast[0]=buff1[i * width/2 + j];//cr
				cast[1]=buff1[i * width/2 + j] >> 8;//y0
				cast[2]=buff1[i * width/2 + j] >> 16;//cb
				cast[3]=buff1[i * width/2 + j] >> 24;//y1
				//YCbCr to RGB
				//Pixel[0] R G B
				temp=1.164*(cast[1]-16)+1.596*(cast[0]-128);
				if(temp>255) buff2[i * width*3 + 6*j]=255;
				else if(temp<0) buff2[i * width*3 + 6*j]=0;
				else buff2[i * width*3 + 6*j]=temp;
				
				temp=1.164*(cast[1]-16)-0.813*(cast[0]-128)-0.391*(cast[2]-128);
				if(temp>255) buff2[i * width*3 + 6*j+1]=255;
				else if(temp<0) buff2[i * width*3 + 6*j+1]=0;
				else buff2[i * width*3 + 6*j+1]=temp;
				
				temp=1.164*(cast[1]-16)+2.018*(cast[2]-128);
				if(temp>255) buff2[i * width*3 + 6*j+2]=255;
				else if(temp<0) buff2[i * width*3 + 6*j+2]=0;
				else buff2[i * width*3 + 6*j+2]=temp;
				//Pixel[1] R G B
				temp=1.164*(cast[3]-16)+1.596*(cast[0]-128);
				if(temp>255) buff2[i * width*3 + 6*j+3]=255;
				else if(temp<0) buff2[i * width*3 + 6*j+3]=0;
				else buff2[i * width*3 + 6*j+3]=temp;
				
				temp=1.164*(cast[3]-16)-0.813*(cast[0]-128)-0.391*(cast[2]-128);
				if(temp>255) buff2[i * width*3 + 6*j+4]=255;
				else if(temp<0) buff2[i * width*3 + 6*j+4]=0;
				else buff2[i * width*3 + 6*j+4]=temp;
				
				temp=1.164*(cast[3]-16)+2.018*(cast[2]-128);
				if(temp>255) buff2[i * width*3 + 6*j+5]=255;
				else if(temp<0) buff2[i * width*3 + 6*j+5]=0;
				else buff2[i * width*3 + 6*j+5]=temp;
		}
		
	}
}

void rgb2ycbcr(Uint8 *buff2, int width, int height, Uint32 *buff1){
	int i,j;
	Uint8 cast[4];
	Uint32 output;
	for(i=0; i<height; ++i){
		for (j=0; j<width/2; ++j) {
				//RGB to YCbCr
				cast[0]=0.257*buff2[i*width*3+6*j]+0.504*buff2[i*width*3+6*j+1]+0.098*buff2[i*width*3+6*j+2]+16;//Y0
				cast[1]=-0.148*buff2[i*width*3+6*j]-0.291*buff2[i*width*3+6*j+1]+0.439*buff2[i*width*3+6*j+2]+128;//Cb0
				cast[2]=0.439*buff2[i*width*3+6*j]-0.368*buff2[i*width*3+6*j+1]-0.071*buff2[i*width*3+6*j+2]+128;//Cr0
				cast[3]=0.257*buff2[i*width*3+6*j+3]+0.504*buff2[i*width*3+6*j+4]+0.098*buff2[i*width*3+6*j+5]+16;//Y1
				//output
				buff1[i*width/2+j]=cast[2]|(cast[0]<<8)|(cast[1]<<16)|(cast[3]<<24);
		}
		
	}
}

void Yuv422to1Channel(int ch, Uint8 *buff){
	int i,j;
	Uint8 cast[4];
	Uint32 output;
	//convert Uint32 YUV 422 to 4 Uint8 YUV
	for(i=0; i<vHeight; ++i){
		for (j=0; j<vWidth/2; ++j) {
			
			//buffer_out[i * vWidth/2 + j] = buffer_in[i * vWidth/2 + j];
			//if((i<imgHeight) && (j<imgWidth/2)){
				cast[0]=buffer_in[i * vWidth/2 + j];//cr
				cast[1]=buffer_in[i * vWidth/2 + j] >> 8;//y0
				cast[2]=buffer_in[i * vWidth/2 + j] >> 16;//cb
				cast[3]=buffer_in[i * vWidth/2 + j] >> 24;//y1
				//Keep Y channel only to get graylevel image
				//cast[0]=(unsigned char)128; 
				//cast[2]=(unsigned char)128;
				//convert back to Uint32 cr|y0|cb|y1
				//output=cast[0]|(cast[1]<<8)|(cast[2]<<16)|(cast[3]<<24);
				//buffer_out[i * vWidth/2 + j]=output;
				if(ch==0){//Y
					buff[i*vWidth+2*j]=cast[1];
					buff[i*vWidth+2*j+1]=cast[3];
				}
				
				if(ch==1){//Cb
					buff[i*vWidth+2*j]=cast[2];
					buff[i*vWidth+2*j+1]=cast[2];
				}
				if(ch==2){//Cr
					buff[i*vWidth+2*j]=cast[0];
					buff[i*vWidth+2*j+1]=cast[0];
				}
				
			//}
			//else
				//buffer_out[i * vWidth/2 + j]=BLK;

		}
	}
}
int otsu2 (Uint8 *image,int width, int height){
	int i,j;
	// histogram
	float histogram[256] = {0};
	for(i = 0; i < height; i++){
		unsigned char* p = (unsigned char*)(image + width*i);
		for(j = 0; j < width; j++){
			histogram[*p++]++;
		}
	}

	//normalize histogram
	int size = height*width;
	for(i = 0; i < 256; i++){
		histogram[i] = histogram[i]/size;
	}

	//average pixel value
	float avgValue = 0;
	for(i = 0; i<256; i++){
		avgValue+=i*histogram[i];
	} 

	int threshold;
	float maxVariance = 0;
	float w=0,u=0;
	for(i=0; i < 256; i++){
		w+=histogram[i];
		u+=i*histogram[i];

		float t = avgValue*w-u;
		float variance = t*t/(w*(1-w));
		if(variance>maxVariance){
			maxVariance = variance;
			threshold = i;
		}
	}
	return threshold;
	
}
void downSampling(int p,Uint8 *buff1, Uint8 *buff2){
	int i,j;
	for(i=0; i<vHeight/p; i++){
		for (j=0; j<vWidth/p; j++) {
			buff2[i*vWidth/p+j]=buff1[i*vWidth*p+j*p];
		}
	}
}
			
void display(Uint8 *buff,Uint32 *buff2,int width, int height, int x,int y){
	//memset(buff2,128|(0<<8)|(128<<16)|(0<<24),Pixels*sizeof(Uint32));
	int i,j;
	for(i=0; i<height; i++){
		for(j=0; j<width/2; j++){
				if(buff[i*width+2*j]==1 || buff[i*width+2*j+1]==1 ){
					buff2[(height-i-1+y)*vWidth/2+width/2-j+x]=90|(82<<8)|(240<<16)|(82<<24);//red
					if(j+1<width/2) buff2[(height-i-1+y)*vWidth/2+width/2-j+1+x]=90|(82<<8)|(240<<16)|(82<<24);
					if(j-1>-1) buff2[(height-i-1+y)*vWidth/2+width/2-j-1+x]=90|(82<<8)|(240<<16)|(82<<24);
					if(i+1<height) buff2[(height-i+y)*vWidth/2+width/2-j+x]=90|(82<<8)|(240<<16)|(82<<24);
					if(i-1>-1) buff2[(height-i+y-2)*vWidth/2+width/2-j+x]=90|(82<<8)|(240<<16)|(82<<24);
				}
				else if(buff[i*width+2*j]==2 || buff[i*width+2*j+1]==2 ){
					buff2[(height-i-1+y)*vWidth/2+width/2-j+x]=54|(145<<8)|(34<<16)|(145<<24);//green
					if(j+1<width/2) buff2[(height-i-1+y)*vWidth/2+width/2-j+1+x]=54|(145<<8)|(34<<16)|(145<<24);
					if(j-1>-1) buff2[(height-i-1+y)*vWidth/2+width/2-j-1+x]=54|(145<<8)|(34<<16)|(145<<24);
					if(i+1<height) buff2[(height-i+y)*vWidth/2+width/2-j+x]=54|(145<<8)|(34<<16)|(145<<24);
					if(i-1>-1) buff2[(height-i-2+y)*vWidth/2+width/2-j+x]=54|(145<<8)|(34<<16)|(145<<24);
				}
				else buff2[(height-i-1+y)*vWidth/2+width/2-j+x]=128|(buff[i*width+2*j]<<8)|(128<<16)|(buff[i*width+2*j+1]<<24);
		}
	}
}
void RotationDetect(){
	//rotation calculation
	int x1=COG.x-COF.x;
	int y1=COG.y-COF.y;
	int angle=acos(x1 / sqrt(x1*x1 + y1*y1)) * 180 / PI;
	int step=angle-preAngle;
	//step=step/abs(step);
	//int angle=acos((x1*x2 + y1*y2) / sqrt((x1*x1 + y1*y1)*(x2*x2 + y2*y2))) * 180 / PI;
	
	accumAngle+=step;
	if(step>30 || step<-30 || preAngle==0 || abs(angle-90)<5){//|| (abs(COF.x-90) < 2) || abs(angle-90)<3
		accumAngle=0;
	}
	//update angle and locations
	preAngle=angle;
	
	//right left motion
	int shift_x=COG.x-preCOG.x;
	//int shift_y=COF.y-preCOF.y;
	if(shift_x>35 || shift_x<-30) accumShift=0;//if the shift is too large, reset accumulation
	
	preCOG=COG;
	printf("COG-COF angle: %d.\n", accumAngle);
	//2 conditions: one is accumulated angle, another is current angle is large enough
	if((accumAngle>22)&& (angle>95)){ // 
		command=4;accumAngle=-2;
		}
	else if((accumAngle<-18) && (angle<88)){ //
		command=3; accumAngle=2;//clockwise rotation preAngle=0;
		}
	//2 conditions: one is accumulated shift, another is current COG x is in the movement area
	else if(accumShift>6){// && COG.x>80
		command=0; accumAngle=0;//preAngle=0;
		if(shift_x>0) accumShift=4;//if moving left, make moving left easier
		else accumShift=-2;//if moving back, reset accumulation
	}
	else if(accumShift<-6){// && COG.x<110
		command=1;preAngle=0; accumAngle=0;
		if(shift_x<0) accumShift=-4;//if moving right, make moving right easier
		else accumShift=2;
	} 
	else command=2;
	accumShift+=shift_x;

	//else accumAngle+=step;
}
int ContourTracing(unsigned char *buff, unsigned char *buff2, int width,int height, kalman1_state *angleState){

	memset(buff2,0,SIZE*sizeof(unsigned char));

    const int k = 10;//k for the kcurvature
    struct CvPoint startPoint={0,0};// = {width/2,height/2};

    short bFindStartpoint = 0;
    int i=0;
    int j=0;
    int numLoop=0;
    unsigned char * ptr,*dst;
    //stack<int> board;//odd x coordinate even for y
    //int *board=(int*)malloc(sizeof(int)*720*480/2);
    
    //current point
    struct CvPoint currentPoint = {0,0};
    //Moore neighbors directions
    int directions[8][2] = {{0,1},{1,1},{1,0},{1,-1},{0,-1},{-1,-1},{-1,0},{-1,1}};
    int beginDirection = 0;
    short bFindBoardpoint = 0;//if it is boundary point 
    
    //find start point

    for (i = 30 ; i< height-30 && bFindStartpoint == 0; i++)
    {
        for (j = 30 ; j< width-30 && bFindStartpoint == 0; j++)
        {
            ptr = (buff + i*width + j);
            if (*ptr == 255)
            {
                startPoint.x = j;
                startPoint.y = i;

                bFindStartpoint = 1;
                //cout<<"x:  " << j <<"y :  " <<i <<endl;
            }
        }
    }
    //handStart_x=10;//abs(startPoint.x-10)+1;
    //handStart_y=10;//abs(startPoint.y-10)+1;

    int contourlength = 0; //the length of contour
    //start tracing
    currentPoint = startPoint;
    bFindStartpoint = 0;
    beginDirection = 0;
    pChainCodex[contourlength] = startPoint.x;//record the contour trace
    pChainCodey[contourlength] = startPoint.y;

    while (!bFindStartpoint)
    {
    	
        bFindBoardpoint = 0;
        //within neighbors
        while (!bFindBoardpoint)
        {
        	//prevent stuck inside of the loop
        	numLoop++;
    		if(numLoop>1500) return -1;
            //boundary condition
            ptr = (buff + (currentPoint.y + directions[beginDirection][1])* width + currentPoint.x + directions[beginDirection][0]);
            if(ptr-buff > -1 && ptr-buff < width*height){
	            if (*ptr == 255)
	            {
	                bFindBoardpoint = 1;
	                currentPoint.x += directions[beginDirection][0];
	                currentPoint.y += directions[beginDirection][1];
	//                a = currentPoint.x;
	//                printf("The x is %d \n", a);
	                
	                contourlength++;
	
	                pChainCodex[contourlength] = currentPoint.x;//record the contour trace
	                pChainCodey[contourlength] = currentPoint.y;
	
	                //mark the contour as white
	                dst  = buff2 + currentPoint.y * width + currentPoint.x;
	                *dst = 255;
	                
	                if (currentPoint.x == startPoint.x  && currentPoint.y == startPoint.y )
	                {
	                    //if(contourlength<k){ break;}
	                    //else 
	                    //	bFindStartpoint = 1;
	                    bFindStartpoint = 1;
	                }
	                //change the tracing direction
	                beginDirection -= 2;
	                if (beginDirection < 0)
	                {
	                    beginDirection += 8;
	                }
	                
	                
	                
	            }
	            else
	            {
	            	//*dst = 0;
	                beginDirection++;
	                beginDirection = beginDirection%8;
	            }
            }
            else return -1;
        }
        //cout<<"currentPoint    "<<currentPoint.x <<"     "<< currentPoint.y<<endl;
    }
            //over 50% points are contour, then not hand
    if(contourlength<k || contourlength>width*height/2){
    	return -1;
    }
//	if(contourlength>1000){
//		handStart_x=startPoint.x;
//		handStart_y=startPoint.y;
//	}
     //calculate the positions of finger tips
    
    float avgAngle=0;
    float kalmanAngle=0;
    int anglelength = 1;
    float distanceA, distanceB, distanceC;
    // calculate the first
    distanceC = sqrt(pow((pChainCodex[contourlength-1] - pChainCodex[1]),2) + pow((pChainCodey[contourlength-1] - pChainCodey[1]),2));
    distanceA = sqrt(pow((pChainCodex[contourlength-1] - pChainCodex[0]),2) + pow((pChainCodey[contourlength-1] - pChainCodey[0]),2));
    distanceB = sqrt(pow((pChainCodex[0] - pChainCodex[1]),2) + pow((pChainCodey[0] - pChainCodey[1]),2));
    myAngle[0] = acos((distanceC*distanceC-distanceA*distanceA-distanceB*distanceB)/(-2*distanceA*distanceB))*180/PI;
    
    int count_fingers = 0;//count for the fingertips around 10
    for (j = k; j < contourlength-k; j = j+k){
        distanceC = sqrt(pow((pChainCodex[j-k] - pChainCodex[j+k]),2) + pow((pChainCodey[j-k] - pChainCodey[j+k]),2));
        distanceA = sqrt(pow((pChainCodex[j-k] - pChainCodex[j]),2) + pow((pChainCodey[j-k] - pChainCodey[j]),2));
        distanceB = sqrt(pow((pChainCodex[j] - pChainCodex[j+k]),2) + pow((pChainCodey[j] - pChainCodey[j+k]),2));
        myAngle[anglelength] = acos((distanceC*distanceC-distanceA*distanceA-distanceB*distanceB)/(-2*distanceA*distanceB))*180/PI;
        //printf("The angle %d is %f \n", anglelength,myAngle[anglelength]);
        dst  = (unsigned char *)buff2 + pChainCodey[j] * width + pChainCodex[j];
        //record small angle and eliminate boundary points
        if (myAngle[anglelength] < 105 && pChainCodey[j]<height-5 && pChainCodey[j]>5 && pChainCodex[j]<width-5 && pChainCodex[j]>5){
            fingertipx[count_fingers] = pChainCodex[j];
            fingertipy[count_fingers] = pChainCodey[j];
            avgAngle+=myAngle[anglelength];
            count_fingers++;
            if(count_fingers>k) return 0;
            //dst  = (unsigned char *)image2->imageData + 180 * image2->widthStep + 10;
            //*dst is binary image with 0 and 255, define 1 as the fingertips
            *dst = 1;
        }
        //else *dst=0;
        anglelength++;
    }
    if(count_fingers>5){ 
	    fingerDetected=1;
	    //get the average location of fingertips COF
	    int avgX=0, avgY=0;
	    for(j=0;j<count_fingers;j++){
	    	avgX+=fingertipx[j];
	    	avgY+=fingertipy[j];
	    }
	    avgX=(float)avgX/count_fingers;
	    avgY=(float)avgY/count_fingers;
	    COF.x=kalman1_filter(&stateCOFx,avgX);
	    COF.y=kalman1_filter(&stateCOFy,avgY);
	    buff2[COF.y*width+COF.x]=2;
	    
		avgAngle=avgAngle/count_fingers;
		if(count_fingers==0) avgAngle=0;
		kalmanAngle= kalman1_filter(angleState, avgAngle);
		//printf("average angle of fingertips: %.2f.\n", avgAngle);
	    //printf("Kalman tracked average angle: %.2f.\n", kalmanAngle);
    }
    //if fingers disappeared, then reset records
    else{
    	 accumShift=0;
    	 accumAngle=0;
    	 preAngle=0;
    }
    return 0;  
}	

void test(int count){
	//ycbcr2rgb(buffer_in, vWidth, vHeight, Buffer_input);
	//rgb2ycbcr(Buffer_input, vWidth, vHeight, buffer_out);
	/*
	int i,j;
	for(i=0;i<BufferNum; ++i) {
		for (j=0; j<Pixels/BufferNum; ++j) {
			internal_buffer1[j] =  buffer_in[i* Pixels/BufferNum + j]; 
		}
		ycbcr2rgb(internal_buffer1, vWidth, vHeight/BufferNum, internal_buffer2);
		rgb2ycbcr(internal_buffer2, vWidth, vHeight/BufferNum, internal_buffer1);
		
		for (j=0; j<Pixels/BufferNum; ++j) {
			buffer_out[i* Pixels/BufferNum + j] =  internal_buffer1[j]; 
		}		
	}
	*/
	int i,j;
	//1 means channel Cb
	Yuv422to1Channel(1,Buffer_input);

	if(count==20){
		//otsu and binarization
		segmentation_T=otsu2(Buffer_input,vWidth,vHeight);
		for(i=0; i<vHeight; ++i){
			for(j=0; j<vWidth; ++j){
				if(Buffer_input[i*vWidth+j]<segmentation_T){
					Buffer_output[i*vWidth+j]=0;
				}
				else Buffer_output[i*vWidth+j]=255;
		
			}
		}
		//dilation
		//morphErosion(Buffer_midput, Buffer_midput2, 4, 0 );
		//morphDilation(Buffer_midput2, Buffer_output, 3, 0 );
		//display(Buffer_output);
	}
	
	//Uint8 result=152;
	//int i,j;
	//cry0cby1
	/*
	for(i=0; i<vHeight/2; ++i){
		for(j=0; j<vWidth/2; ++j){
			if(downsamples[i*vWidth/2+j]<segmentation_T){
				buffer_out[i*vWidth/2+j]=128|(16<<8)|(128<<16)|(16<<24);
			}
			else if(i<vHeight/2 && j<vWidth/4 && downsamples[i*vWidth/2+j]>=segmentation_T){
				buffer_out[i*vWidth/2+j]=128|(235<<8)|(128<<16)|(235<<24);
			}
			else buffer_out[i*vWidth/2+j]=0|(235<<8)|(128<<16)|(235<<24);
		}
	}
	*/
	/*//RGB plan
	int i;
	ycbcr2rgb(buffer_in, vWidth, vHeight, Buffer_input);
	int result=otsu2(2,Buffer_input);
	for(i=0; i<vHeight*vWidth; ++i){
		if(Buffer_input[3*i+2]<result){
			Buffer_input[3*i]=0;
			Buffer_input[3*i+1]=0;
			Buffer_input[3*i+2]=0;
		}
		else{
			Buffer_input[3*i]=255;
			Buffer_input[3*i+1]=255;
			Buffer_input[3*i+2]=255;
		}
	}
	rgb2ycbcr(Buffer_input, vWidth, vHeight, buffer_out);
	*/
}
void test2(){
	int i,j;
	//1 means channel Cb
	Yuv422to1Channel(1,Buffer_input);
	downSampling(DOWN,Buffer_input,downsized_buff);
	//otsu and binarization
	segmentation_T=otsu2(downsized_buff,vWidth/DOWN,vHeight/DOWN);
	for(i=0; i<vHeight/DOWN; ++i){
		for(j=0; j<vWidth/DOWN; ++j){
			if(downsized_buff[i*vWidth/DOWN+j]<segmentation_T){
				downsized_buff[i*vWidth/DOWN+j]=0;
			}
			else downsized_buff[i*vWidth/DOWN+j]=255;
	
		}
	}
	morphErosion(downsized_buff,downsized_out , 4, 0 );
	morphDilation(downsized_out, downsized_buff, 3, 0 );
	display(downsized_buff,buffer_out,vWidth/DOWN,vHeight/DOWN,0,0);
	
}

void test3(){

	int i,j;
	//1 means channel Cb
	Yuv422to1Channel(1,Buffer_input);
	downSampling(DOWN,Buffer_input,downsized_buff);
	//otsu and binarization
	segmentation_T=otsu2(downsized_buff,vWidth/DOWN,vHeight/DOWN);
	for(i=0; i<vHeight/DOWN; ++i){
		for(j=0; j<vWidth/DOWN; ++j){
			if(downsized_buff[i*vWidth/DOWN+j]<segmentation_T){
				downsized_buff[i*vWidth/DOWN+j]=0;
			}
			else downsized_buff[i*vWidth/DOWN+j]=255;
	
		}
	}
	morphErosion(downsized_buff,downsized_out , 4, 0 );
	morphDilation(downsized_out, downsized_buff, 3, 0 );
	ContourTracing(downsized_buff, downsized_out, vWidth/DOWN,vHeight/DOWN,&stateAngle);
	//display(downsized_out,buffer_out,vWidth/DOWN, vHeight/DOWN);
}

void track(kalman1_state *statex,kalman1_state *statey){
	int i,j;
// tracking the center of palm with by the 1D kalman filter
	unsigned int xCenter = 0; //initialize the Center
	unsigned int yCenter = 0;
	int bwArea = 0;
	for(i = 0; i<vHeight/DOWN; ++i){
		for(j = 0; j < vWidth/DOWN; ++j){
			bwArea += downsized_out[i*vWidth/DOWN+j]/255;
			xCenter += j*downsized_out[i*vWidth/DOWN+j]/255;
			yCenter += i*downsized_out[i*vWidth/DOWN+j]/255;
		}
	}

	xCenter = ((float)xCenter)/bwArea;
	yCenter = ((float)yCenter)/bwArea;
	
	// 1D Kalman filter for the Center of the hand add the kalmanfilter.c and kalmanfilter.h to the workplace

	// initialize the kalman filter
	float xOut = 0;
	float yOut = 0;

//	kalman1_state statex;
//	kalman1_state statey;
//    kalman1_init(&statex, xCenter, 5e-2);
//    kalman1_init(&statey, yCenter, 5e-2); // the initialization of kalman should be outside the recurence

    // prediction of position of
    xOut = kalman1_filter(statex, xCenter);
    yOut = kalman1_filter(statey, yCenter);
    //mark the center point
	COG.x=xOut;
	COG.y=yOut;
	downsized_out[COG.y*vWidth/DOWN+COG.x]=2;
    //printf("Central point of hand: ( %d, %d).\n", xCenter,yCenter);
    //printf("Kalman tracked point of hand: ( %d, %d).\n", (int)xOut,(int)yOut);
//    if(xOut>100 && xOut<220 && yOut> 120 && yOut<200) command=1;
//    else if(xOut>460 && xOut<580 && yOut> 120 && yOut<200) command=0;
//    else command=2;
//    
    }

// Copy data from input buffer to output buffer
void Copy(void){
	Int32 i, j;
	Uint32 temp;
	
	// Copy data from input buffer to output buffer and 
	// draw green square box at the center of view
	for(i=0; i<vHeight; ++i)
		for (j=0; j<vWidth/2; ++j) {
			buffer_out[i * vWidth/2 + j] = buffer_in[i * vWidth/2 + j];
		}
	
	
	// Use block-based processing inside internal memory to speed up
	// Demo code for block separation: BufferNum blocks along row direction
//	for(i=0;i<BufferNum; ++i) {
//		for (j=0; j<Pixels/BufferNum; ++j) {
//			internal_buffer1[j] =  buffer_in[i* Pixels/BufferNum + j]; 
//		}
//		ycbcr2rgb(internal_buffer1, vWidth, vHeight/BufferNum, internal_buffer2);
//		rgb2ycbcr(internal_buffer2, vWidth, vHeight/BufferNum, internal_buffer1);
//		
//		for (j=0; j<Pixels/BufferNum; ++j) {
//			buffer_out[i* Pixels/BufferNum + j] =  internal_buffer1[j]; 
//		}		
//	}
}


void DisplayImage(Uint32 *cof, int h, int w, int x, int y){
	Int32 i, j;
	for(i=0; i<h; ++i)
		for (j=0; j<w/2; ++j) {
			//buffer_out[i * vWidth/2 + j] = buffer_in[i * vWidth/2 + j];
			//if((i<imgHeight) && (j<imgWidth/2))
			buffer_out[(i+y) * vWidth/2 + j+x]=cof[i * w/2 + j];
			//else
			//	buffer_out[(i+y) * vWidth/2 + j+x]=BLA;
		}
	
}

void DisplayScore(){
	DisplayImage(score,scoreHeight, scoreWidth, 150, 125);
	int curLoc1=score_R;
	int curLoc2=score_L;
	int i=1, j=1;
	switch(curLoc1%10){
		case 0:
			DisplayImage(num0,digitHeight, digitWidth, 200-digitWidth/2, 165);break;
		case 1:
			DisplayImage(num1, digitHeight, digitWidth,200-digitWidth/2, 165);break;
		case 2:
			DisplayImage(num2, digitHeight, digitWidth,200-digitWidth/2, 165);break;
		case 3:
			DisplayImage(num3, digitHeight, digitWidth,200-digitWidth/2, 165);break;
		case 4:
			DisplayImage(num4, digitHeight, digitWidth,200-digitWidth/2, 165);break;
		case 5:
			DisplayImage(num5, digitHeight, digitWidth,200-digitWidth/2, 165);break;
		case 6:
			DisplayImage(num6, digitHeight, digitWidth,200-digitWidth/2, 165);break;
		case 7:
			DisplayImage(num7, digitHeight, digitWidth,200-digitWidth/2, 165);break;
		case 8:
			DisplayImage(num8, digitHeight, digitWidth,200-digitWidth/2, 165);break;
		case 9:
			DisplayImage(num9, digitHeight, digitWidth,200-digitWidth/2, 165);break;
		default:
			DisplayImage(num0, digitHeight, digitWidth,200-digitWidth/2, 165);break;
			 
	}
	switch(curLoc2%10){
		case 0:
			DisplayImage(num0,digitHeight, digitWidth, 150-digitWidth/2, 165);break;
		case 1:
			DisplayImage(num1, digitHeight, digitWidth,150-digitWidth/2, 165);break;
		case 2:
			DisplayImage(num2, digitHeight, digitWidth,150-digitWidth/2, 165);break;
		case 3:
			DisplayImage(num3, digitHeight, digitWidth,150-digitWidth/2, 165);break;
		case 4:
			DisplayImage(num4, digitHeight, digitWidth,150-digitWidth/2, 165);break;
		case 5:
			DisplayImage(num5, digitHeight, digitWidth,150-digitWidth/2, 165);break;
		case 6:
			DisplayImage(num6, digitHeight, digitWidth,150-digitWidth/2, 165);break;
		case 7:
			DisplayImage(num7, digitHeight, digitWidth,150-digitWidth/2, 165);break;
		case 8:
			DisplayImage(num8, digitHeight, digitWidth,150-digitWidth/2, 165);break;
		case 9:
			DisplayImage(num9, digitHeight, digitWidth,150-digitWidth/2, 165);break;
		default:
			DisplayImage(num0, digitHeight, digitWidth,150-digitWidth/2, 165);break;
			 
	}
	while(curLoc1/10!=0){
		i++;
		curLoc1/=10;	
		switch(curLoc1%10){
		case 0:
			DisplayImage(num0, digitHeight, digitWidth,200-i*digitWidth/2, 165);break;
		case 1:
			DisplayImage(num1, digitHeight, digitWidth,200-i*digitWidth/2, 165);break;
		case 2:
			DisplayImage(num2, digitHeight, digitWidth,200-i*digitWidth/2, 165);break;
		case 3:
			DisplayImage(num3, digitHeight, digitWidth,200-i*digitWidth/2, 165);break;
		case 4:
			DisplayImage(num4, digitHeight, digitWidth,200-i*digitWidth/2, 165);break;
		case 5:
			DisplayImage(num5, digitHeight, digitWidth,200-i*digitWidth/2, 165);break;
		case 6:
			DisplayImage(num6, digitHeight, digitWidth,200-i*digitWidth/2, 165);break;
		case 7:
			DisplayImage(num7, digitHeight, digitWidth,200-i*digitWidth/2, 165);break;
		case 8:
			DisplayImage(num8, digitHeight, digitWidth,200-i*digitWidth/2, 165);break;
		case 9:
			DisplayImage(num9, digitHeight, digitWidth,200-i*digitWidth/2, 165);break;
			 
		}
	}
	while(curLoc2/10!=0){
		j++;
		curLoc2/=10;	
		switch(curLoc2%10){
		case 0:
			DisplayImage(num0, digitHeight, digitWidth,150-j*digitWidth/2, 165);break;
		case 1:
			DisplayImage(num1, digitHeight, digitWidth,150-j*digitWidth/2, 165);break;
		case 2:
			DisplayImage(num2, digitHeight, digitWidth,150-j*digitWidth/2, 165);break;
		case 3:
			DisplayImage(num3, digitHeight, digitWidth,150-j*digitWidth/2, 165);break;
		case 4:
			DisplayImage(num4, digitHeight, digitWidth,150-j*digitWidth/2, 165);break;
		case 5:
			DisplayImage(num5, digitHeight, digitWidth,150-j*digitWidth/2, 165);break;
		case 6:
			DisplayImage(num6, digitHeight, digitWidth,150-j*digitWidth/2, 165);break;
		case 7:
			DisplayImage(num7, digitHeight, digitWidth,150-j*digitWidth/2, 165);break;
		case 8:
			DisplayImage(num8, digitHeight, digitWidth,150-j*digitWidth/2, 165);break;
		case 9:
			DisplayImage(num9, digitHeight, digitWidth,150-j*digitWidth/2, 165);break;
			 
		}
	}		
}

// Modify the image repositioning the pixels
// This reposition consists in invert the image having the center like invert point
void Inv_1(void){
	Int32 i,j;
	Uint32 temp;
	
	j=Pixels-1;
	i=0;
	do{
		temp=buffer_in[j];
		buffer_out[i]=temp;
		i++;
		j--;
	} while (i < Pixels);
}

// Flip horizontally the image
void Inv_2(void){
	Int32 i,j;
	Uint32 temp;
	
	j=Pixels/vHeight-1;
	i = 0;

	do{
		temp=buffer_in[j];
		buffer_out[i]=temp;
		i++;
		j--;

		if(i%(Pixels/vHeight)==0){ j=i+Pixels/vHeight-1;}
	} while (i < Pixels);
}

// Flip vertically the image
void Inv_3(void){
	Int32 i,j;
	Uint32 temp;

	i = 0;
	j=Pixels-Pixels/vHeight;

	do{
		temp=buffer_in[j];
		buffer_out[i]=temp;
		i++;
		j++;

		if(i%(Pixels/vHeight)==0){j=Pixels-i-Pixels/vHeight;}
	} while (i < Pixels);
	
}


	
	/*
	  void Inv_4(void){
	Int32 i,j;
	Uint8 temp[vHeight*vWidth/(DOWN*DOWN)];
	int i=0;
	for(i=0; i<vHeight*vWidth/(DOWN*DOWN);i++){
		temp=downsized_out[i]
	}
	j=vWidth/DOWN-1;
	i = 0;
		do{
		temp=downsized_out[j];
		downsized_out[i]=temp;
		i++;
		j--;

		if(i%(vWidth/DOWN)==0){j=i+vWidth/DOWN-1;}
	} while (i < vHeight*vWidth/(DOWN*DOWN));
	}*/

/************************************************
 * 												*
 * 					GAME ENGINE					*
 * 												*
 * 												*
 ***********************************************/
// Create Block
int Create_Block(){
	
	int b = rand() % 7;//0 to 6, local use
	int i=0;
	//int block[416]={0};//16x26
	for (i=0;i<16;i++)
	miniblock[i]=0;
	
	
	switch (b){
		case 0: //I
		//{0,1,0,0,0,1,0,0,0,1,0,0,0,1,0 ,0};
			miniblock[1]=1;
			miniblock[5]=1;
			miniblock[9]=1;
			miniblock[13]=1;
			name=CYAN;
			break;
			
		case 1: //O
			//miniblock[]={0,0,0,0,0,1,1,0,0,1,1,0,0,0,0,0};
			miniblock[9]=1;
			miniblock[10]=1;
			miniblock[13]=1;
			miniblock[14]=1;
			name=YELLOW;
			break;
		case 2: //J
			//miniblock[]={0,0,0,0,0,1,0,0,0,1,0,0,1,1,0,0};
			miniblock[6]=1;
			miniblock[10]=1;
			miniblock[13]=1;
			miniblock[14]=1;
			name=BLUE;
			break;
		case 3: //L
			//miniblock[]={0,0,0,0,1,0,0,0,1,0,0,0,1,1,0,0};
			miniblock[5]=1;
			miniblock[9]=1;
			miniblock[13]=1;
			miniblock[14]=1;
			name=WHITE;
			break;
		case 4: //v or T
			//miniblock[]={0,0,0,0,0,0,0,0,1,1,1,0,0,1,0,0};
			miniblock[8]=1;
			miniblock[9]=1;
			miniblock[10]=1;
			miniblock[13]=1;
			name=MAG;
			break;
		case 5: //s
			//miniblock[]={0,0,0,0,0,1,1,0,1,1,0,0,0,0,0,0};
			miniblock[5]=1;
			miniblock[9]=1;
			miniblock[10]=1;
			miniblock[14]=1;
			name=GREEN;
			break;
		case 6: //z
			//miniblock[]={0,0,0,0,1,1,0,0,0,1,1,0,0,0,0,0};
			miniblock[6]=1;
			miniblock[9]=1;
			miniblock[10]=1;
			miniblock[13]=1;
			name=RED;
			break;
			default:break;
			
	}
	return b;
}

void Display_Block(int x,int y){
	//interpolation here
	int i=0;
	
	
	//field[10]=2088800000; field[11]=2088800000; field[12]=2088800000; field[13]=2088800000;
	//field[40]=2088800000; field[41]=2088800000; field[67]=2088800000; field[109]=2088800000;
	int bcount1=0; int bcount2=0; int bcount3=0;
	
	for (bcount1=0; bcount1<Pixels/3; bcount1++){
		buffer_out[bcount2+bcount3*vWidth/2]=try_left1[bcount2/12 + 10*(bcount3/24)];
		buffer_out[bcount2+bcount3*vWidth/2+960]=try_right1[bcount2/12 + 10*(bcount3/24)];
		//240x480-> 10x20
		bcount2++;
		if (bcount2>=(vWidth/6)){//same as ==, but safer
			bcount3++;
			bcount2=0;
		}
	}
	bcount1=0; bcount2=0; bcount3=0;
	for (bcount1=0; bcount1<4608;bcount1++){//(pixel/37.5)
		buffer_out[bcount2+bcount3*vWidth/2+492]=444603008+946878426*block_preL[bcount2/12 + 4*(bcount3/24)];
		buffer_out[bcount2+bcount3*vWidth/2+540]=444603008+1644196992*block_preR[bcount2/12 + 4*(bcount3/24)];//scrn cntr
		bcount2++;
		if (bcount2>=(vWidth/15)){
			bcount3++;
			bcount2=0;
		}
	}
/*		
	int i,j;
	for(i=0; i<vHeight; ++i)
		for (j=0; j<vWidth/2; ++j) {
			//buffer_out[i * vWidth/2 + j] = buffer_in[i * vWidth/2 + j];
//			buffer_out[i * vWidth/2 + j]=444603008;
//			if((i<4)&&(j<2)){
//				if(miniblock[i*4+j]==0){
//					buffer_out[i * vWidth/2 + j]=444603008;
//				}
//				else
//					buffer_out[i * vWidth/2 + j]=2088800000;//461380480 2088800000
//			}
			if((j>x-11)&&(j<x+10)&&(i==y))
					buffer_out[i * vWidth/2 + j + x*vWidth/2 + y]=2088800000;
			else if((i>y-21)&&(i<y)&&(j==x))
					buffer_out[i * vWidth/2 + j + x*vWidth/2 + y]=2088800000;
			else
				buffer_out[i * vWidth/2 + j]=444603008;		

		}
*/
}


void Rotate_Block(int* x,int y, int horiz){
	// 0  1  2  3
	// 4  5  6  7
	// 8  9 10 11
	//12 13 14 15
	int i=0; int j=0; int k=0;
	j=x[9]+x[10]+x[13]+x[14];
	if (j==4){//name
		for (i=0; i<16; i++){
			block_temp[i]=x[i];}
		}
	else{//rotate about position 9, CC or CCW
		i=x[12]+x[13]+x[14]+x[15];
		k=x[0]+x[4]+x[8]+x[12];
		block_temp[0]=x[0];
		block_temp[1]=x[11];
		block_temp[2]=x[2];
		block_temp[3]=x[3];
		block_temp[4]=x[12];
		block_temp[5]=x[8];
		block_temp[6]=x[4];
		block_temp[7]=x[7];
		block_temp[8]=x[13];
		block_temp[9]=x[9];
		block_temp[10]=x[5];
		block_temp[11]=x[1];
		block_temp[12]=x[14];
		block_temp[13]=x[10];
		block_temp[14]=x[6];
		block_temp[15]=x[15];
		j=block_temp[12]+block_temp[13]+block_temp[14]+block_temp[15];
		//shift to align bottom
			if (i>0&&j<=0){
				y++;//yposition
			}
			else if (i<=0 &&j>0){
				y--;
			}
		i=block_temp[0]+block_temp[4]+block_temp[8]+block_temp[12];
			if (i>0&&k<=0){
				horiz++;//yposition
			}
			else if (i<=0 &&k>0){
				horiz--;
			}
			
			for (i=0; i<16; i++){
			x[i]=block_temp[i];}
			}
}

void Move_AI(){//field right ideal move
	int i, j, k, orient;
	val_max=-1000000000; value2=0; hp=0; hp2=0;
	for (b=0;b<4;b++){
		if (block_curR[1]==1){
			shifts=10;}
		else if (block_curR[11]==1){
			shifts=7;}
		else if (((block_curR[4]+block_curR[8]+block_curR[12])>0)&&((block_curR[6]+block_curR[10]+block_curR[14])>0)){
			shifts=8;}
		else{
			shifts=9;
		} 
		x_posR1=0;
		if ((block_curR[4]+block_curR[8]+block_curR[12])==0)
			x_posR1--;
		while (shifts>0){
			value1=0; hp=0;
			for (i=0;i<200;i++){
				try_right1[i]=field_right[i];
				if (((i%10)==0)||((i%10)==9)){
        			field_wr1[i]=1;
        		}
			 	else{
			 	     field_wr1[i]=0;
			 	}
			 	if (i>189){
			 		field_wr1[i]++;
			 	}  	
			}
			//drop block
			checkR=0;y_posR1=0;
			while (checkR!=1){
        		y_posR1++;
        		for (i=0;i<16;i++){
        			j=i>>2;
        			k=x_posR1+10*y_posR1+i-4*j+10*j;
        				if (block_curR[i]==1){
        					if (k>199){
        					checkR=1;
        					y_posR1--;
        					break;
        					}
        					else if (field_right[k]>444603008){//check block collision
      							checkR=1;
      							y_posR1--;
      							break;	
       						}
       						
       					}
        		}
			}
			//create field weight
        	for (i=0;i<200;i++){
        		if (field_right[i]>444603008){
        			if (((i%10)>0)&&((i%10)<9)){
        					field_wr1[i-1]++;
        					field_wr1[i+1]++;	
        			}
        			if ((i%10)==0){
        					field_wr1[i+1]++;
        			}
        			if ((i%10)==9){
        					field_wr1[i-1]++;
        			}
        			if(i>9){
        				field_wr1[i-10]++;
        			}
        		}
        	}
        	for (i=0;i<16;i++){
        		j=i>>2;
        		k=x_posR1+10*y_posR1+i-4*j+10*j;
        		if (k>199){
        			break;
        			}
        		if (block_curR[i]==1){	
        			try_right1[k]=444603008+1644196992*block_curR[i];
        			value1=value1+field_wr1[k];
        			hp=hp+19-k/10;
       			}
        		else{
        			try_right1[k]=field_right[k];
        		}
        	
        	}
        	value1=value1-hp>>1;//height penalty
        	
			//now calculate depth1 score
		
			for (j=0;j<20;j++){
				k=0;
				for (i=0;i<10;i++){
					if(try_right1[10*j+i]==2088800000){
						k++;}
				}
				if (k==10){//remove row bonus
					value1=value1+40-y_min;
				}
			}
			//clean field
			for (j=0;j<20;j++){
        		sum=0;
        		for (i=0;i<10;i++){
        			if (try_right1[i+10*j]==2088800000){
        				sum++;
        			}
        		}
        		if (sum==10){//full row
        			if (j==0){//rare case
        				for (i=0;i<10;i++){
        					try_right1[i+10*j]=444603008;
        				}
        			}
        			else{
        				k=j;
        				while (k>1){
        						for (i=0;i<10;i++){
        							try_right1[i+10*k]=try_right1[i+10*k-10];	
        						}
        						k=k-1;;
        				}
        			}
        		}
        	}
        	//penalize holes after cleaning
        	holect=0;
			for (i=0;i<10;i++){
				k=20; j=19;
				while (j>0){
					if (try_right1[i+10*j]>444603008){
							k=j;	
						}
						j--;
				}
				while (k<20){
					if (try_right1[i+10*k]==444603008){
						if (try_right1[i+10*k]>444603008){
							holect++;
						}
					}
					k++;
				}
			}
			value1=value1-10*holect;
		
        	//depth2
        	for (b2=0;b2<4;b2++){
				if (block_preR[1]==1){
					shifts2=10;}
				else if (block_preR[11]==1){
					shifts2=7;}
				else if (((block_preR[4]+block_preR[8]+block_preR[12])>0)&&((block_preR[6]+block_preR[10]+block_preR[14])>0)){
					shifts2=8;}
				else{
					shifts2=9;
				} 
				x_posR2=0;
				if ((block_preR[4]+block_preR[8]+block_preR[12])==0)
					x_posR2--;
				while (shifts2>0){
					value2=value1; hp=0;
					for (i=0;i<200;i++){
						try_right2[i]=try_right1[i];
						if (((i%10)==0)||((i%10)==9)){
        					field_wr2[i]=1;
        				}
			 			else{
			 	     		field_wr2[i]=0;
			 			}
			 			if (i>189){
			 				field_wr2[i]++;
			 			}  	
					}
					//drop block
					checkR=0;y_posR2=0;
					while (checkR!=1){
        				y_posR2++;
        				for (i=0;i<16;i++){
        					j=i>>2;
        					k=x_posR2+10*y_posR2+i-4*j+10*j;
        						if (block_preR[i]==1){
        							if (k>199){
        							checkR=1;
        							y_posR2--;
        							break;
        							}
        							else if (try_right1[k]>444603008){//check block collision
      									checkR=1;
      									y_posR2--;
      									break;	
       								}
		       						
       							}
        				}
					}
					//create field weight
        			for (i=0;i<200;i++){
        				if (try_right1[i]>444603008){
        					if (((i%10)>0)&&((i%10)<9)){
		        					field_wr2[i-1]++;
        							field_wr2[i+1]++;	
        					}
        					if ((i%10)==0){
		        					field_wr2[i+1]++;
        					}
        					if ((i%10)==9){
		        					field_wr2[i-1]++;
        					}
        					if(i>9){
        						field_wr2[i-10]++;
        					}
        				}
        			}
        			for (i=0;i<16;i++){
        				j=i>>2;
        				k=x_posR2+10*y_posR2+i-4*j+10*j;
        				if (k>199){
        					break;
        					}
        				if (block_preR[i]==1){	
        					try_right2[k]=444603008+1644196992*block_preR[i];
        					value2=value2+field_wr2[k];
        					hp=hp+19-k/10;
       					}
        				else{
        					try_right2[k]=try_right1[k];
        				}
		        	
        			}
        			value2=value2-hp>>1;//height penalty
        	
					//now calculate depth2 score
					for (j=0;j<20;j++){
						k=0;
						for (i=0;i<10;i++){
							if(try_right2[10*j+i]==2088800000){
								k++;}
						}
						if (k==10){//remove row bonus
							value2=value2+40-y_min;
						}
					}
					
					//clean field
					for (j=0;j<20;j++){
        				sum=0;
        				for (i=0;i<10;i++){
        					if (try_right2[i+10*j]==2088800000){
        						sum++;
	        				}
        				}
	        			if (sum==10){//full row
    	    				if (j==0){//rare case
        						for (i=0;i<10;i++){
        							try_right2[i+10*j]=444603008;
	        					}
        					}
        					else{
        						k=j;
        						while (k>1){
        							for (i=0;i<10;i++){
        								try_right2[i+10*k]=try_right2[i+10*k-10];	
        							}
        							k=k-1;;
        						}
        					}
        				}
        			}
					
					
					holect=0;
					for (i=0;i<10;i++){
						k=20; j=19;
						while (j>0){
							if (try_right2[i+10*j]>444603008){
								k=j;	
							}
							j--;
						}
						while (k<20){
							if (try_right2[i+10*k]==444603008){
								if (try_right2[i+10*k-10]>444603008){
									holect++;
								}
							}
							k++;
						}
					}
					value2=value2-10*holect;
					if (value2>val_max){
						val_max=value2;
						//add store pos.orient
						target_x=x_posR1;
						x_posR0=x_posR1;
						orient=b;	
					}
					x_posR2++;
					shifts2--;	
				}
				Rotate_Block(block_preR,y_posR2,x_posR2);
        	}
		        
			
			
			x_posR1++;
			shifts--;
		}
		Rotate_Block(block_curR,y_posR1,x_posR1);	
	}
	while(orient>0){
		Rotate_Block(block_curR,y_posR0,x_posR0);
		orient--;
	}
	
	
}

void game_init ()
{
	int i;
	for(i=0;i<200;i++){//initialize fields
		field_left[i]=444603008;
		field_right[i]=444603008;
		try_left1[i]=444603008;
		try_right1[i]=444603008;
		disp_left[i]=BLK;
		disp_right[i]=BLK;
		try_right2[i]=444603008;
		field_wr1[i]=0;
		field_wr2[i]=0;
	}
	for (i=0; i<Pixels; i++){//Initialize display
		buffer_out[i]=444603008;
	}
	Create_Block();//first four blocks
	for (i=0;i<16;i++){
		block_curL[i]=miniblock[i];
	}
	Create_Block();
	for (i=0;i<16;i++){
		block_curR[i]=miniblock[i];
	}
	/*	for (i=0;i<16;i++)
	block_curR[i]=0;
	block_curR[5]=1;
	block_curR[9]=1;
	block_curR[10]=1;
	block_curR[14]=1;
	*/ 
	Create_Block();
	for (i=0;i<16;i++){
		block_preL[i]=miniblock[i];
	}
	Create_Block();
	for (i=0;i<16;i++){
		block_preR[i]=miniblock[i];
	}
	int play=1; int play_L=1; int play_R=1;
	int timer=0;
	sum=0; lines_L=0; lines_R=0, y_min=20; command=2; delay=0;
	score_L=0; score_R=0; bonus_L=0; bonus_R=0;
}

void penalty(Uint32* side, int amt, int color)
{
	//side is which field, amt is how many blocks to penalize
	int x=0; int y=0; int i,j,q,z;
	if (color==1){
		z=1644196992;
	}
	else{
		z=946878426;
	}
	
	while (amt>0){
		Create_Block();
        for (i=0;i<16;i++){
        	block_sudden[i]=miniblock[i];
        }
        j=rand()%4;
        while (j>0)
        {
        	Rotate_Block(block_sudden,y,x);//filler variables x,y
        	j--;
        }
        if (block_sudden[1]==1){
			q=10;}
		else if (block_sudden[11]==1){
			q=7;}
		else if (((block_sudden[4]+block_sudden[8]+block_sudden[12])>0)&&((block_sudden[6]+block_sudden[10]+block_sudden[14])>0)){
			q=8;}
		else{
			q=9;
		}
		j=rand()%q;
		if ((block_sudden[4]+block_sudden[8]+block_sudden[12])==0)
			x--;
		checkR=0;
		while (checkR!=1)
		{
			y++;
			for (i=0;i<16;i++){
        		j=i>>2;
        		int k=x+10*y+i-4*j+10*j;
	        	if (block_sudden[i]==1){
        			if (k>199){
        			checkR=1;
        			y--;
        			break;
        			}
        			else if (side[k]>444603008){//check block collision
	      				checkR=1;
      					y--;
      					break;	
       				}
		 						
       			}
        	}
		}
		for (i=0;i<16;i++){
        		j=i>>2;
        		int k=x+10*y+4+i-4*j+10*j;
        		if (block_sudden[i]==1){
        			if (side[k]>444603008){//still obstructed->no viable move
        				//play=0;
        			}
        			side[k]=444603008+z*block_sudden[i];
        		}
        		else{
        			side[k]=side[k];
        		}
        	}
        	amt--;
	}
}
void main( void )
   {
	srand(time(NULL));//for create block, do not declare more than once (changes 1/s)
	int i=0; int j=0; 
	int genL=0; int genR=0;//generate new block
	/* Initialize BSL */
	EVMDM6437_init();
    /* Initialize the DIP Switches & LEDs if needed */
    EVMDM6437_DIP_init( );
    EVMDM6437_LED_init( );
    	
	// Initialize video input/output 
	video_loopback_test();
	game_init();
	
	//counter for segmentation
	int seg_count=5;
	timer=0; 
	Move_AI();
	//Kalman filter initialization
    kalman1_init(&statex, 0, 5e9);
    kalman1_init(&statey, 0, 5e9); // the initialization of kalman should be outside the recurence
    kalman1_init(&stateAngle, 0, 5e9);
    kalman1_init(&stateCOFx, 0, 5e9);
    kalman1_init(&stateCOFy, 0, 5e9);
    //COF.x=0;
    //COF.y=0;
	while (play){		
        /* Will return DIP_DOWN or DIP_UP */
        dip0 = EVMDM6437_DIP_get( DIP_0 );
        dip1 = EVMDM6437_DIP_get( DIP_1 );
        dip2 = EVMDM6437_DIP_get( DIP_2 );
        dip3 = EVMDM6437_DIP_get( DIP_3 );
        
        
        // Run different procedures depending on the DIP switches pressed.
        if ( dip0 == DIP_DOWN ){
	        //Copy();
	        test(20);
	        
	        display(Buffer_output,buffer_out,vWidth,vHeight,0,0);
	        //rgb2ycbcr(downsized_output, vWidth/2, vHeight/2, buffer_out);
	        track(&statex, &statey);
        }
        else if ( dip1 == DIP_DOWN )
        	{
        		fingerDetected=0;
        	 	test3();
        	 
        	 	if(fingerDetected==1) {
        	 		track(&statex, &statey);
        	 		RotationDetect();
        	 	}
        	 	//Inv_4();
        	 	display(downsized_out,buffer_out,vWidth/DOWN, vHeight/DOWN,120,360);
        		int changeL=0; changeR=0;
//        		test(20);
//        		track(&statex, &statey);
        		j=0;
        		//if (timer2>20){ 
        		int arg1=0; int arg2=0; int arg3=0;
   				for (i=0;i<4;i++){
   					j=i<<2;
   					int k=x_posL+10*y_posL+10*j;
   					if (field_left[k]>444603008)
   						arg1=1;
   					k=x_posL+10*y_posL+10*j+2;
   					if (field_left[k]>444603008)
   						arg2=1;	
       				k=x_posL+10*y_posL+10*j+3;
       				if (field_left[k]>444603008)
       					arg3=1;	
       			}
        			if (command == 0){//replace w. gesture command
        				j=-1;//x shift left
        			}
        			else if (command ==1){
        				j=1;//right
        			}
        			else if (command==2){
        				j=0;
        			}
        			else if (command == 3){//create rotate command, clockwise
        				Rotate_Block(block_curL,y_posL,x_posL);
        				
					i=block_curL[12]+block_curL[13]+block_curL[14];
        				if ((y_posL+(i+2)/3)>17){
        					y_posL--;
        				}
        				//check collision
        				j=0;
        			}
        			else if (command == 4){//create ccw rotate command
        				Rotate_Block(block_curL,y_posL,x_posL);
        				Rotate_Block(block_curL,y_posL,x_posL);
        				Rotate_Block(block_curL,y_posL,x_posL);
        				
					i=block_curL[12]+block_curL[13]+block_curL[14];
        				if ((y_posL+(i+2)/3)>17){
        					y_posL--;
        				}
        				//check collision
        				j=0;
        			}
        			else if (command == 5){//create lower block command
        				y_posL++;
        				i=block_curL[12]+block_curL[13]+block_curL[14];
        				if ((y_posL+(i+2)/3)>17){
        					y_posL--;
        				}
        				j=0;
        			}
        			else if (command ==6){//from persistent command 5, or separate full drop cmd
        				while (changeL!=1){
        					y_posL++;
        					for ((i=0&&changeL!=1);i<16;i++){//can use changeL!=1 if separate from rt field
        						j=i>>2;
        						int k=x_posL+10*y_posL+4+i-4*j+10*j;
        							if (block_curL[i]==1){
        								if (field_left[k]>444603008){//check block collision
        									changeL=1;	
        									break;
        								}
        								else
        								try_left1[k]=444603008+946878426*block_curL[i];
        							}
        							else{
        							try_left1[k]=field_left[k];}
        					}
        					
        				}
        				
        			}
        			else if (command==7)
        			{
        				game_init();
        			}
        			delay++;
        			y_posL=y_posL+delay/4;//moving down
        			y_posR0=y_posR0+delay/4;
        			if (delay>=4)
        			delay=0;
        			//timer2=0;
        		//}
        		if (timer>2){//remove if if removing if is faster
        			x_posL=x_posL+j;
        			int b=0;
        			for (i=0;i<16;i++){
        				b=i>>2;
        				block_temp[i]=field_left[x_posL+10*y_posL+4+i-4*b+10*b];//window
        				if (block_temp[i]+block_curL[i]>1391481434){
        					x_posL=x_posL-j;
        					break;
        				}
        				block_temp[i]=field_left[x_posL+10*y_posL-6+i-4*b+10*b];
        				if (block_temp[i]+block_curL[i]>1391481434){
        					x_posL=x_posL-j;
        					break;
        				}
        			}
        			j=0;
        			timer=0;
        		}	
        			timer++;
    	    		j=block_curL[4]+block_curL[8]+block_curL[12];
	        		if ((x_posL-(j+2)/3)<-5){
        				x_posL++;
        			}
        			j=block_curL[6]+block_curL[10]+block_curL[14];
        			if ((x_posL+(j+2)/3)+block_curL[11]>4){
        				x_posL--;
        			}

        		
        		
        		
        		
        		for (i=0;i<200;i++)
        			try_left1[i]=field_left[i];
        		for ((i=0&&changeL!=1);i<16;i++){//can use changeL!=1 if separate from rt field
        			j=i>>2;
        			int k=x_posL+10*y_posL+4+i-4*j+10*j;
        				if (block_curL[i]==1){
        					if (field_left[k]>444603008){//check block collision
        						changeL=1;	
        						break;
        					}
        					else
        					try_left1[k]=444603008+946878426*block_curL[i];
        				}
        				else{
        				try_left1[k]=field_left[k];}
        		}
        		for (i=0;i<200;i++)
        			try_right1[i]=field_right[i];
        		for ((i=0&&changeR!=1);i<16;i++){//can use changeL!=1 if separate from rt field
        			j=i>>2;
        			int k=x_posR0+10*y_posR0+i-4*j+10*j;
        				if (block_curR[i]==1){
        					if (field_right[k]>444603008){//check block collision
        						changeR=1;	
        						break;
        					}
        					else
        					try_right1[k]=444603008+1644196992*block_curR[i];
        				}
        				else{
        				try_right1[k]=field_right[k];}
        		}
        		
        		i=block_curL[12]+block_curL[13]+block_curL[14];
        		if ((y_posL+(i+2)/3)>17){
        			changeL=1;
        		}
        		i=block_curR[12]+block_curR[13]+block_curR[14];
        		if ((y_posR0+(i+2)/3)>17){
        			changeR=1;
        		}
        		if (changeL==1){
        			y_posL--;
        			genL=1;	
        			for (i=0;i<16;i++){
        				j=i>>2;
        				int k=x_posL+10*y_posL+4+i-4*j+10*j;
        				if (block_curL[i]==1){
        					if (field_left[k]>444603008){//still obstructed->no viable move
        						//play_L=0;
        						//genL=0;
        					}
        					field_left[k]=444603008+946878426*block_curL[i];
        				}
        				else{
        					field_left[k]=field_left[k];
        				}
        			}
        			if (bonus_L>0)
        			penalty(field_left, bonus_L,0);
        			bonus_L=0;
        			for (i=0;i<200;i++)
        			try_left1[i]=field_left[i];
        			        			
        		}
        		if (changeR==1){
        			y_posR0--;
        			genR=1;
        			for (i=0;i<16;i++){
        				j=i>>2;
        				int k=x_posR0+10*y_posR0+i-4*j+10*j;
        				if (block_curR[i]==1){
        					if (field_right[k]>444603008){//still obstructed->no viable move
        						//play_R=0;
        						//genR=0;
        					}
        					field_right[k]=444603008+1644196992*block_curR[i];
        				}
        				else{
        					field_right[k]=field_right[k];
        				}
        			}
        			if (bonus_R>0)
        			penalty(field_right, bonus_R,1);	
        			bonus_R=0;
        			
        			j=199;
        			for (i=0;i<200;i++)
        			{
        				try_right1[i]=field_right[i];
        				if ((field_right[i]>444603008)&&(i<j))
        				{
        					y_min=i/10;
        					j=i;
        				}
        			}
        		}
        		
        			
        		
        		//update field
        		Display_Block(10,20);//show field
        		//check fields for full lines
        		combo_L=0; combo_R=0;
        		for (j=0;j<20;j++){
        			sum=0;
        			for (i=0;i<10;i++){
        				if (field_left[i+10*j]==1391481434){
        					sum++;
        				}
        			}
        			if (sum==10){//full row, can add score.linect here
        				if (j==0){//rare case
        					for (i=0;i<10;i++){
        						field_left[i+10*j]=0;
        					}
        				}
        				else{
        					int b=j;
        					while (b>1){
        							for (i=0;i<10;i++){
        								field_left[i+10*b]=field_left[i+10*b-10];	
        							}
        							b=b-1;;
        					}
        				}
        				lines_L++;
        				combo_L++;
        			}
        			sum=0;
        			for (i=0;i<10;i++){
        				if (field_right[i+10*j]==2088800000){
        					sum++;
        				}
        			}
        			if (sum==10){//full row, can add score.linect here
        				if (j==0){//rare case
        					for (i=0;i<10;i++){
        						field_right[i+10*j]=0;
        					}
        				}
        				else{
        					int b=j;
        					while (b>1){
        							for (i=0;i<10;i++){
        								field_right[i+10*b]=field_right[i+10*b-10];	
        							}
        							b=b-1;;
        					}
        				}
        				lines_R++;
        				combo_R++;
        			}
        		}
        		if (combo_L==1){
        			score_L=score_L+10;
        			bonus_R++;
        		}
        		else if (combo_L==2){
        			score_L=score_L+15;
        			bonus_R++;
        		}
        		else if (combo_L==3){
        			score_L=score_L+40;
        			bonus_R++;
        		}
        		else if (combo_L==4){
        			score_L=score_L+60;
        			bonus_R++;
        		}
        		if (combo_R==1){
        			score_R=score_R+10;
        		}
        		else if (combo_R==2){
        			score_R=score_R+15;
        		}
        		else if (combo_R==3){
        			score_R=score_R+40;
        		}
        		else if (combo_L==4){
        			score_R=score_R+60;
        			bonus_L++;
        		}
        		
        	   
        	Display_Block(10,20);//show field w. removed lines  
        	//display scores
        	DisplayScore();//DisplayImage(num1, 120, 120);  
        	if (dip3==DIP_DOWN){
        		game_init();
        		//preAngle=0; accumAngle=0;
        	} 
        	
	}
        else if ( dip2 == DIP_DOWN ) 	
			DisplayImage(score,scoreHeight, scoreWidth, 130, 125);
			//Yuv422to1Channel(0,Buffer_input);
        	//Inv_2();
        else if ( dip3 == DIP_DOWN ){
        	 fingerDetected=0;
        	 test3();     	 
        	 if(fingerDetected==1) {
        	 	track(&statex, &statey);
        	 	RotationDetect();
        	 }
        	 display(downsized_out,buffer_out,vWidth/DOWN, vHeight/DOWN,120,360);
			 
       }

        	
        	//Inv_3();

        	//next        
       // if (timer>120){
       // 	genR=1;
       // }
        timer++;
        if (genL==1){
        	Create_Block();
        	for (i=0;i<16;i++){
        		block_curL[i]=block_preL[i];
        		block_preL[i]=miniblock[i];
        	}
        	name_curL=name_preL;
        	name_preL=name;
        	genL=0;
        	timer=0;
        	
        	x_posL=0; y_posL=0;
        	
        }
        if (genR==1){
        	Create_Block();
        	for (i=0;i<16;i++){
        		block_curR[i]=block_preR[i];
        		block_preR[i]=miniblock[i];
        	}
        	name_curR=name_preR;
        	name_preR=name;
        	genR=0;
        	timer=0;
        	x_posR0=0; y_posR0=0;
        	Move_AI();
        }
	}
}