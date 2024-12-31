#ifndef DSPUARTCOMM_H
#define DSPUARTCOMM_H
#include <stdio.h>
#include <signal.h>
#include <pthread.h>
#include "netapi.h"
#include <sched.h> // for sched_yield()
#include <assert.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/stat.h>
#include <math.h>
#include <dirent.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/signal.h>
#include <termios.h>

#include "serial_dev.h"


#define GS_DEVICE_RESET 0
#define GS_DEVICE_RUNNING 1

// Dan Block changed from 12345 to 10001
#define GS_PORT_COMS 10001
#define GS_PORT_CTRL 12346
#define GS_MAXTCPCON 8
#define GS_MAXCTRLCON 4
#define GS_MAXINPKTS 10          // How many packets to remember
#define GS_MAXOUTPKTS 10         // before sending/consuming

#define GS_CTRL_SOCK 1
#define GS_COMS_SOCK 2
#define GS_UDPCOMS_SOCK 2
//#warn Dan Block changed from 255 to 99999
#define GS_DEV_MTU 99999               // DSP MTU (max packet size) also
                                     // used to determine statically allocated
                                     // buffers and read/write buffers

static int gs_ctrl_skt;                 // device control listening socket
static int gs_coms_skt;                 // device coms listening socket
static int gs_udpcoms_skt;                  // udp coms socket
static int gs_num_coms_con = 0;         // number of connections
static int gs_num_ctrl_con = 0 ;        // only one allowed for control 
static int gs_next_open_coms_con = 0;
static int gs_next_open_ctrl_con = 0;


static int gs_port_coms = GS_PORT_COMS; // Listening port
static int gs_port_ctrl = GS_PORT_CTRL; // Listening port
pthread_mutex_t gs_mutex = PTHREAD_MUTEX_INITIALIZER; 

static int gs_exit = 0;                                   
static int gs_quit = 0;
static int gs_device_state = 0;     // 0 is off(reset) 1 is running


//#define GS_IN_BUF_LEN GS_DEV_MTU*10
#define GS_IN_BUF_LEN 256*10
#define GS_OUT_BUF_LEN GS_DEV_MTU*10
#define GS_IP_HDR_LEN 6     // 4 (address) + 1 (type) + 1 (length of data)   


/* Structure for a packet (avoids circular character buffers) */
typedef struct ip_pkt {
  int skt;
  int remote_ip;
  char data[GS_DEV_MTU+GS_IP_HDR_LEN];
  int inpos;
  int outpos;
  int empty;      // 0 if not sent/received,  1 if empty
  int next;       // doubly linked list          
  int prev;
} ip_pkt_struct;


int coms_cons_idx[GS_MAXTCPCON];
int ctrl_cons_idx[GS_MAXCTRLCON];

#define GS_SENDNOW 1
//#define GS_SENDNOW 0 will defer writing till buffer is ready

#endif // DSPUARTCOMM_H
