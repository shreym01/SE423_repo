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

union float_char
{
    float val[2];
    char bytes[8];
};

#define SHOW_IMAGE 
char SERFILE[] = "/dev/ttyAMA1";
#define SERIALBUFFSIZE 1024
float cx; //position of the center(in percentage)
float cy; //position of the center(in percentage)
union float_char to_send;
union float_char received;
int frame1_count = 0;
int frame2_count = 0;
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
std::vector<cv::KeyPoint> keypoints2;
int charsread = 0;

int fd;  // variables used to map to physical memory addresses
void *map_base;
off_t target;
int32_t *myuart2;




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



    printf("Starting\n");
    VideoCapture cap1(0);
    cap1.set(CAP_PROP_FRAME_WIDTH,320);
    cap1.set(CAP_PROP_FRAME_HEIGHT,240);

//  setting frame rate at 
    double fps = cap1.get(CAP_PROP_FPS);
    printf("fps= %.3f \n",fps);


    // set the second camera:
    VideoCapture cap2(2);
    cap2.set(CAP_PROP_FRAME_WIDTH,320);
    cap2.set(CAP_PROP_FRAME_HEIGHT,240);

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
    Mat frame2, frame2_HSV, frame2_threshold, frame2_key;
    Ptr<SimpleBlobDetector> blob_detector = SimpleBlobDetector::create(params);



    while (true)
    {
        frame1_count++;
		// printf("Framecount1 = %d\n");
        cap1 >> frame1;
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
        frame2_count++;
		// printf("Framecount2 = %d\n");
        cap2 >> frame2;
        if (frame2.empty())
        {
            break;
        }
        // Convert from BGR to HSV colorspace
        cvtColor(frame2, frame2_HSV, COLOR_BGR2HSV);
        // // Detect the object based on HSV Range Values
        // Scalar lower(hmin, smin, vmin); //set lower bound
        // Scalar upper(hmax, smax, vmax); //set upper bound
        inRange(frame2_HSV, lower, upper, frame2_threshold);
        blob_detector->detect(frame2_threshold, keypoints2);
#ifdef SHOW_IMAGE
        drawKeypoints(frame1,keypoints1,frame1_key,Scalar::all(-1),DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        // drawKeypoints(frame2,keypoints2,frame2_key,Scalar::all(-1),DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
#endif
#ifdef SHOW_IMAGE
        // drawKeypoints(frame1,keypoints1,frame1_key,Scalar::all(-1),DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        drawKeypoints(frame2,keypoints2,frame2_key,Scalar::all(-1),DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
#endif


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
        for (int i=0; i<keypoints2.size(); i++){
            x = keypoints2[i].pt.x; 
            y = keypoints2[i].pt.y;
            size = keypoints2[i].size;
            x_2 = x;
            y_2 = y;
            size_2 = size;
            if (size > keypoints2[max_index].size) {
                max_index = i;
            }
            
            printf("keypoints2 %d ,x %.3f, y %.3f, size %f \n",keypoints2.size(),x,y,size);
        }
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
        if(keypoints2.size()>0) {
            to_send.val[0] = keypoints2[max_index].pt.x;
            to_send.val[1] = keypoints2[max_index].pt.y;
            sd_write(asterisk);
            sd_write(asterisk);
            sd_writen(to_send.bytes,8);
            //usleep(5000);
            sd_write(exclamation);
            sd_write(exclamation);
            //usleep(5000);
            charsread = sd_readn(received.bytes,8);
			if (charsread != 8) {
				// printf("charsread = %d\n",charsread);
				while(sd_readn(received.bytes,8) > 0) {} 
			} else {
				printf("value1 = %.3f value2 = %.3f\n",received.val[0],received.val[1]);
			}

// printf("\n\n\n\nIntDiv=%d,FracDiv=%d\n\n\n\n",myuart2[265],myuart2[266]);


        }

        // Show the frames
#ifdef SHOW_IMAGE
        // imshow("image", frame);
        imshow("mask1", frame1_threshold);
        imshow("mark1", frame1_key);
        // imshow("image", frame);
        imshow("mask2", frame2_threshold);
        imshow("mark2", frame2_key);
#endif
        waitKey(1);
        char key = (char)waitKey(30);
        frame1_count++;
        frame2_count++;
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
