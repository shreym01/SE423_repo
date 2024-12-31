#include <stdio.h>
//#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <math.h>
#include <sys/select.h>
#include <termios.h>
//#include <stropts.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/mman.h>
#include <linux/fb.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <ctype.h>
#include <sys/types.h>
#include <math.h>
#include <tgmath.h>


#define MAP_SIZE 4096UL
#define MAP_MASK (MAP_SIZE - 1)

#include <time.h>
#include <memory>
using namespace cv;

extern "C"
{
#include "serial_dev.h"
}

using namespace cv;
using namespace std;

// Simple structure to hold details about the framebuffer device after it has
// been opened.
typedef struct {
	// Pointer to the pixels. Try to not write off the end of it.
	uint16_t * buffer;
	// The file descriptor for the device.
	int fd;
	// Number of bytes in the buffer. To work out how many elements are in the buffer, divide this by 4 (i.e. sizeof(uint32_t))
	size_t screen_byte_size;
	// Structs providing further information about the buffer configuration. See https://www.kernel.org/doc/Documentation/fb/api.txt
	struct fb_fix_screeninfo fix_info;
	struct fb_var_screeninfo var_info;
} screen_t;

// Because you can't have too many variants of NULL
static const screen_t NULL_SCREEN = {0};

// Indicates if the passed screen_t struct is valid.
#define valid_screen(s) ((s).buffer != NULL)

/**
 * Opens the first frame buffer device and returns a screen_t struct
 * for accessing it. If the framebuffer isn't in the expected format
 * (32 bits per pixel), NULL_SCREEN will be returned.
 */
screen_t open_fb() {
	const char * const SCREEN_DEVICE = "/dev/fb0";
	int screen_fd = open(SCREEN_DEVICE, O_RDWR);
	if (screen_fd == -1) {
		printf("ERROR: Failed to open %s\n", SCREEN_DEVICE);
		return NULL_SCREEN;
	}

	struct fb_var_screeninfo var_info = {0};
	if (ioctl(screen_fd, FBIOGET_VSCREENINFO, &var_info) == -1) {
		printf("ERROR: Failed to get variable screen info\n");
		return NULL_SCREEN;
	}

	struct fb_fix_screeninfo fix_info = {0};
	if (ioctl(screen_fd, FBIOGET_FSCREENINFO, &fix_info) == -1) {
		printf("ERROR: Failed to get fixed screen info\n");
		return NULL_SCREEN;
	}

//	if (var_info.bits_per_pixel != 32) {
//		printf("ERROR: Only support 32 bits per pixel. Detected bits per pixel: %d\n", var_info.bits_per_pixel);
//		return NULL_SCREEN;
//	}

	const size_t screen_byte_size = var_info.xres * var_info.yres * var_info.bits_per_pixel / 8;
	uint16_t * const buffer = (uint16_t *)mmap(NULL, screen_byte_size, PROT_READ | PROT_WRITE, MAP_SHARED, screen_fd, 0 /* offset */);

    printf("\n\nXres = %d, Yres = %d, bits = %d\n\n",var_info.xres,var_info.yres,var_info.bits_per_pixel);

	screen_t screen = {
		.buffer = buffer,
		.fd = screen_fd,
		.screen_byte_size = screen_byte_size,
		.fix_info = fix_info,
		.var_info = var_info
	};
	return screen;
}

/**
 * Closes the framebuffer when you are finished with it. Don't try
 * to access things in the struct after calling this or else a
 * kitten will die.
 */
void close_fb(screen_t *screen) {
	munmap(screen->buffer, screen->screen_byte_size);
	close(screen->fd);
	*screen = NULL_SCREEN;
}

//#define NUMFRAME_ROWS 576
//#define NUMFRAME_COLS 720
#define NUMFRAME_ROWS 1080
#define NUMFRAME_COLS 1920

//#define FRAMEOFFSET1_ROWS 120
//#define FRAMEOFFSET1_COLS 40
//#define FRAMEOFFSET2_ROWS 120
//#define FRAMEOFFSET2_COLS 380
#define FRAMEOFFSET1_ROWS 20
#define FRAMEOFFSET1_COLS 40
#define FRAMEOFFSET2_ROWS 280
#define FRAMEOFFSET2_COLS 40

union float_char
{
    float val[2];
    char bytes[8];
};

//#define SHOW_IMAGE 
char SERFILE[] = "/dev/ttyAMA0";
#define SERIALBUFFSIZE 1024
float cx; //position of the center(in percentage)
float cy; //position of the center(in percentage)
union float_char to_send;
union float_char received;
int frame1_count = 0;
//int frame2_count = 0;
int max_index=0;
float x = 0;
float y = 0;
float size = 0;
float x_1,y_1;
float x_2,y_2;
float size_1,size_2;
float distance1 = 0;
float stdistance;
int avecount = 0;
float distancearray[5];
std::vector<cv::KeyPoint> keypoints1;
//std::vector<cv::KeyPoint> keypoints2;
int charsread = 0;

int fd;  // variables used to map to physical memory addresses
void *map_base;
off_t target;
int32_t *myuart2;

int celements = 320*3;  // b,g,r elements in each column




/*
 * setup_serial()
 *   sets the serial port up at 115200 baud
 */
void setup_serial()
{
    sd_setup(SERFILE); //starts non-blocking
    sd_ioflush();
}

/////////////// Color Detection //////////////////////
char asterisk[] = "*";
char exclamation[] = "!";

unsigned char *data_bgr;
int r=0;
int c=0;
int nr;
int nc;
uint16_t blue=0, green=0, red=0;
int dantoggle = 1;

int main()
{
    	/* ****************************************************************/
	/*  Memory map to physical memory spaces of the UART2 Registers*/	
	// if((fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1) {
	// 	printf("/dev/mem could not be opened.\n");
	// 	exit(1);
	// } else {
	// 	printf("/dev/mem opened.\n");
	// }
	// fflush(stdout);
	
	// //target = 0x7e201400;  //UART2 of Raspberry pi 4.0 only  !!!!!
	// target = 0xFE201000;
	// //target = 0x7e200000;
	// /* Map one page for shared memory structure*/
	// map_base = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, target);
	// if(map_base == (void *) -1) {
	// 	printf("Memory map failed.\n");
	// 	exit(1);
	// } else {
	// 	printf("gpio Struct mapped at address %p.\n", map_base);
	// }
	// fflush(stdout);
	// myuart2 = (int32_t *) map_base;
	
	// printf("\n\n\n\nIntDiv=%d,FracDiv=%d\n\n\n\n",myuart2[265],myuart2[266]);

	


	to_send.val[0] = 0.1234;
    to_send.val[1] = 0.4567;
    printf("Initializing serial port driver %s...\n", "/dev/ttyAMA1");
    setup_serial();
    printf("...OK\n");

    //sd_set_blocking();
    printf(".\n");
    sd_ioflush();

    // myuart2[256+12] &= ~0x301; // Clear enable bits
    // usleep(100000);
    // myuart2[256+11] &= ~0x10;  // clear FIFO    
    // usleep(100000);

    // usleep(100000);
	// myuart2[256+9] = 1;
	// myuart2[256+10] = 28;//59;  // 28 is 2,083,333  59 is 1,562,500
    // usleep(100000);
    // myuart2[256+11] |= 0x10;  //enable fifo
    // usleep(100000);
    // myuart2[256+12] |= 0x301; // Set enable bits
    // usleep(100000);
    

	
	// printf("\n\n\n\nIntDiv=%d,FracDiv=%d\n\n\n\n",myuart2[265],myuart2[266]);
	//printf("\n\n\n\nIntDiv=%d,FracDiv=%d\n\n\n\n",myuart2[9],myuart2[10]);
	// exit(1);

    screen_t screen = open_fb();
    if (!valid_screen(screen)) {
        printf("ERROR: Failed to open screen\n");
        return 1;
    }

    nr = 240;
    nc = 320;
    for(r=0; r<nr; r++) {  
        for(c=0; c<nc;c++) {
            blue = 0;
            green = 0;
            red = 255;
            //screen.buffer[(r+FRAMEOFFSET1_ROWS)*NUMFRAME_COLS + (FRAMEOFFSET1_COLS + c)] = (int)((red<<16) | (green<<8) | blue); //32bit
            screen.buffer[(r+FRAMEOFFSET1_ROWS)*NUMFRAME_COLS + (FRAMEOFFSET1_COLS + c)] =   ((red & 0xFFF8)<<11) | ((green & 0xFFFC)<<5) | (blue >> 3);
            
        }
    } //--------------end pixel processing---------------------


    printf("Starting\n");
    VideoCapture cap1(0);
    cap1.set(CAP_PROP_FRAME_WIDTH,320);
    cap1.set(CAP_PROP_FRAME_HEIGHT,240);

//  setting frame rate at 
    double fps = cap1.get(CAP_PROP_FPS);
    printf("fps= %.3f \n",fps);


    // set the second camera:
//    VideoCapture cap2(2);
//    cap2.set(CAP_PROP_FRAME_WIDTH,320);
//    cap2.set(CAP_PROP_FRAME_HEIGHT,240);

    Mat img;

    //below value selected with trackbar
    int hmin = 71, smin = 17, vmin = 24;
    int hmax = 112, smax = 221, vmax = 255;


    // //create trackbar to select values, hue max is 179, saturation and value are 255
#ifdef SHOW_IMAGE
    namedWindow("Trackbars", (640, 200));
    createTrackbar("Hue Min", "Trackbars", &hmin, 179);
    createTrackbar("Hue Max", "Trackbars", &hmax, 179);
    createTrackbar("Sat Min", "Trackbars", &smin, 255);
    createTrackbar("Sat Max", "Trackbars", &smax, 255);
    createTrackbar("Val Min", "Trackbars", &vmin, 255);
    createTrackbar("Val Max", "Trackbars", &vmax, 255);
#endif

	cv::SimpleBlobDetector::Params params;
	params.minDistBetweenBlobs = 40.0f;
	params.filterByInertia = false;
	params.filterByConvexity = false;
	params.filterByColor = false;
	params.filterByCircularity = false;
	params.filterByArea = true;
	params.minArea = 35.0f;
	params.maxArea = 100000.0f;
    Mat frame1, frame1_HSV, frame1_threshold, frame1_key;
//    Mat frame2, frame2_HSV, frame2_threshold, frame2_key;
    Ptr<SimpleBlobDetector> blob_detector = SimpleBlobDetector::create(params);



    while (true)
    {
        frame1_count++;
		// printf("Framecount1 = %d\n");
        cap1 >> frame1;

        printf("frame 1    size0 = %d, size1 = %d type = %d\n\n\n",frame1.size[0],frame1.size[1],frame1.type());
        if (frame1.empty())
        {
            break;
        }
        // Convert from BGR to HSV colorspace
        cvtColor(frame1, frame1_HSV, COLOR_BGR2HSV);
        // // Detect the object based on HSV Range Values
        Scalar lower(hmin, smin, vmin); //set lower bound
        Scalar upper(hmax, smax, vmax); //set upper bound
        inRange(frame1_HSV, lower, upper, frame1_threshold);
        blob_detector->detect(frame1_threshold, keypoints1);
// camera #2
//        frame2_count++;
		// printf("Framecount2 = %d\n");
//        cap2 >> frame2;
//        if (frame2.empty())
//        {
//            break;
//        }
//        // Convert from BGR to HSV colorspace
//        cvtColor(frame2, frame2_HSV, COLOR_BGR2HSV);
//        // // Detect the object based on HSV Range Values
//        // Scalar lower(hmin, smin, vmin); //set lower bound
//        // Scalar upper(hmax, smax, vmax); //set upper bound
//        inRange(frame2_HSV, lower, upper, frame2_threshold);
//        blob_detector->detect(frame2_threshold, keypoints2);
#ifdef SHOW_IMAGE
        drawKeypoints(frame1,keypoints1,frame1_key,Scalar::all(-1),DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        // drawKeypoints(frame2,keypoints2,frame2_key,Scalar::all(-1),DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
#endif
#ifdef SHOW_IMAGE
        // drawKeypoints(frame1,keypoints1,frame1_key,Scalar::all(-1),DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
//        drawKeypoints(frame2,keypoints2,frame2_key,Scalar::all(-1),DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
#endif



        //data_bgr=(unsigned char *)(frame1.imageData); 
        if (dantoggle == 1) {
            dantoggle = 2;
            red = 255;
            blue = 0;
            green = 0;
        } else if (dantoggle == 2) {
            dantoggle = 3;
            red = 0;
            blue = 255;
            green = 0;
        } else {
            dantoggle = 1;
            red = 0;
            blue = 0;
            green = 255;
        }

        printf("dantoggle = %d\n\n",dantoggle);
        data_bgr = frame1.ptr<uchar>(0);

        for(r=0; r<240; r++) {  
			for(c=0; c<320;c++) {
				red = data_bgr[r*320*3 + 3*c];
				green = data_bgr[r*320*3 + 3*c+1];
				blue = data_bgr[r*320*3 + 3*c+2];
				// rgb2hsv(red,green,blue,&h,&s,&v);

				// //--------------process each thresholded pixel here------------------
				// if(((h>=0 && h<=13) || (h>=245 && h <= 255)) && (s>=sl && s<=sh) && (v>=vl && v<=vh)) {
					
				// 	//set pixel to green in the bgr image
				// 	green = 255;
				// 	blue = 0; 
				// 	red = 0;
				// 	numpixels1++;
				// 	rowsum += r;
				// 	colsum += c;
				// 	imagedata[DanMat.cols * r + c ] = 255;
				// } else {
				// 	imagedata[DanMat.cols * r + c ] = 0;
				// }
				
                
                //screen.buffer[(r+FRAMEOFFSET1_ROWS)*NUMFRAME_COLS + (FRAMEOFFSET1_COLS + c)] = ((red & 0xFFF8)<<11) | ((green & 0xFFFC)<<5) | (blue >> 3);
				screen.buffer[(r+FRAMEOFFSET1_ROWS)*NUMFRAME_COLS + (FRAMEOFFSET1_COLS + c)] = ((red & 0xFFF8)<<11) | ((green & 0xFFFC)<<5) | (blue >> 3);
 				
			}
		} //--------------end pixel processing---------------------
        
        for (int i=0; i<keypoints1.size(); i++){
            x = keypoints1[i].pt.x; 
            y = keypoints1[i].pt.y;
            size = keypoints1[i].size;
            x_1 = x;
            y_1 = y;
            size_1 = size;                
            if (size > keypoints1[max_index].size) {
                max_index = i;
            }
            printf("keypoints1 %d ,x %.3f, y %.3f, size %f \n",keypoints1.size(),x,y,size);
        }
                //  camer #2
//        for (int i=0; i<keypoints2.size(); i++){
//            x = keypoints2[i].pt.x; 
//            y = keypoints2[i].pt.y;
//            size = keypoints2[i].size;
//            x_2 = x;
//            y_2 = y;
//            size_2 = size;
//            if (size > keypoints2[max_index].size) {
//                max_index = i;
//            }
            
//            printf("keypoints2 %d ,x %.3f, y %.3f, size %f \n",keypoints2.size(),x,y,size);
//        }
        avecount++;
        int pos = avecount % 5;
        distancearray[pos] = fabs(x_1-x_2);

        for(int n = 0; n<5 ; n++){
            stdistance += distancearray[n];
        }

        distance1 = stdistance/5;
        printf("distance in pixel %.3f \n",distance1);
        stdistance = 0;
  


    


        if(keypoints1.size()>0) {
            to_send.val[0] = keypoints1[max_index].pt.x;
            to_send.val[1] = keypoints1[max_index].pt.y;
            sd_write(asterisk);
            sd_write(asterisk);
            sd_writen(to_send.bytes,8);
            //usleep(5000);
            sd_write(exclamation);
            sd_write(exclamation);
            //usleep(5000);
            charsread = sd_readn(received.bytes,8);
			if (charsread != 8) {
				// printf("charsread = %d\n",charsread);0
				while(sd_readn(received.bytes,8) > 0) {} 
			} else {
				printf("value1 = %.3f value2 = %.3f\n",received.val[0],received.val[1]);
			}

// printf("\n\n\n\nIntDiv=%d,FracDiv=%d\n\n\n\n",myuart2[265],myuart2[266]);
        }

      
//  camera #2
//        if(keypoints2.size()>0) {
//            to_send.val[0] = keypoints2[max_index].pt.x;
//            to_send.val[1] = keypoints2[max_index].pt.y;
//            sd_write(asterisk);
//            sd_write(asterisk);
//            sd_writen(to_send.bytes,8);
//            //usleep(5000);
//            sd_write(exclamation);
//            sd_write(exclamation);
//            //usleep(5000);
//            charsread = sd_readn(received.bytes,8);
//			if (charsread != 8) {
//				// printf("charsread = %d\n",charsread);
//				while(sd_readn(received.bytes,8) > 0) {} 
//			} else {
//				printf("value1 = %.3f value2 = %.3f\n",received.val[0],received.val[1]);
//			}
//
// printf("\n\n\n\nIntDiv=%d,FracDiv=%d\n\n\n\n",myuart2[265],myuart2[266]);
//
//
//        }

        // Show the frames
#ifdef SHOW_IMAGE
        // imshow("image", frame);
        imshow("mask1", frame1_threshold);
        imshow("mark1", frame1_key);
        // imshow("image", frame);
//        imshow("mask2", frame2_threshold);
//        imshow("mark2", frame2_key);
#endif
        waitKey(1);
        char key = (char)waitKey(30);
        frame1_count++;
//        frame2_count++;
        //if (frame_count >= 8) {
        //     break;
        //}
        //break;
        if (key == 'q' || key == 27)
        {
            break;
        }
    }
    return 0;
}
