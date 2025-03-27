#ifndef MAIN_H
#define MAIN_H
#include<stdio.h>
#include<signal.h>
#include<pthread.h>
#include <sched.h> // for sched_yield()
#include <assert.h>
#include "serial_dev.h"
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <semaphore.h>
#include <sys/mman.h>
#include <sys/ioctl.h>



#define GS_DEVICE_RESET 0
#define GS_DEVICE_RUNNING 1

#define GS_DEV_MTU 255               // DSP MTU (max packet size) also
                                     // used to determine statically allocated
                                     // buffers and read/write buffers

// for serial port access
static int gs_quit = 0;
static int gs_device_state = 0;     // 0 is off(reset) 1 is running
int mygetch(void);
int _kbhit();
#define GS_IN_BUF_LEN GS_DEV_MTU*10
#define GS_OUT_BUF_LEN GS_DEV_MTU*10

#define GS_SENDNOW 1
//#define GS_SENDNOW 0 will defer writing till buffer is ready

#endif // MAIN_H
