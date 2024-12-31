#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <semaphore.h>
#include <sys/mman.h>
#include <signal.h>
#include <pthread.h>
#include <sched.h> // for sched_yield()
#include <assert.h>
#include <sys/resource.h>
#include <math.h>
#include <dirent.h>
#include <sys/time.h>
#include <errno.h>
#include <string.h>
#include <stdint.h>
#include <sys/signal.h>
#include <termios.h>
#include <ctype.h>
#include "netapi.h"
#include "serial_dev.h"
#include <math.h>
#include <sys/select.h>
//#include <stropts.h>


#define SERFILE "/dev/ttyAMA1"
#define BUFFSIZE 1024 * 8
#define UDP_PORT 3500
#define MSG_BUFFER 1024

int packet_size;
int gs_udpcoms_skt;                  // udp coms socket
unsigned int printpause = 0;
int recvd;		// Bytes received from socket
char mesg[MSG_BUFFER]; 	// Buffer to hold message from socket

// Types and Variables for LVCOMApp COM  *************************
#define LVNUM_TOFROM_FLOATS 8
#define SENDTO_LVCOMAPP_SEM_MUTEX_NAME "/sem-LVCOMApp-sendto"
#define SENDTO_LVCOMAPP_SHARED_MEM_NAME "/sharedmem-LVCOMApp-sendto"
#define READFROM_LVCOMAPP_SEM_MUTEX_NAME "/sem-LVCOMApp-readfrom"
#define READFROM_LVCOMAPP_SHARED_MEM_NAME "/sharedmem-LVCOMApp-readfrom"
//union of char and floats to LabView for reading data from F28x and Sending to LVCOMApp
// or reading data from LVCOMApp and Sending to F28x
typedef union {
    char data_char[4*LVNUM_TOFROM_FLOATS];
    float data_flts[LVNUM_TOFROM_FLOATS];
} int_ToFromLV_union;

struct shared_memory_sendto_LVCOMApp
{
  int_ToFromLV_union new_ToLV;
};

struct shared_memory_readfrom_LVCOMApp
{
  int_ToFromLV_union new_FromLV;
};

char LVto28x[4*LVNUM_TOFROM_FLOATS];
// End Types and Variables for LVCOMApp COM  *************************


// Types and Variables for AstarApp COM  *************************
#define SENDTO_ASTARAPP_SEM_MUTEX_NAME "/sem-AstarApp-sendto-pose-obstacle"
#define SENDTO_ASTARAPP_SHARED_MEM_NAME "/sharedmem-AstarApp-sendto-pose-obstacle"
#define READFROM_ASTARAPP_SEM_MUTEX_NAME "/sem-AstarApp-readfrom-new-path"
#define READFROM_ASTARAPP_SHARED_MEM_NAME "/sharedmem-AstarApp-readfrom-new-path"
//structure for pose and obstacle from F28
/*
total length is 2*4 + 2*2 + 22*1 = 34 bytes
*/
typedef struct pose_obstacle{
    float x; //4
    float y;
	short destrow; //astar end point row
	short destcol; //astar end point col
	char mapCondensed[22];
} pose_obstacle;

//union of char and pose_obstacle for reading data
typedef union {
    char obs_data_char[34];
    pose_obstacle cur_obs;
} int_pose_union;


struct shared_memory_sendto_AstarApp
{
  int_pose_union new_pose;
};

struct shared_memory_readfrom_AstarApp
{
  char read_path[81];   // Path up to 80 long.  81st char is a status char.  char81 = [bit7=NA
  																						//bit6=NA
																					 	//bit5=NA
																						//bit4=NA
																						//bit3=start or stop are obstacles
																						//bit2=start or stop outside map
																						//bit1=1 start == endpoint
																						//bit0=1 Need to reset Map]
};
// End Types and Variables for AstarApp COM  *************************

//LINUXCMDApp
#define CMDNUM_FROM_FLOATS 11
#define RECVFROM_LINUXCMDAPP_SEM_MUTEX_NAME "/sem-LINUXCMDApp-recvfrom"
#define RECVFROM_LINUXCMDAPP_SHARED_MEM_NAME "/sharedmem-LINUXCMDApp-recvfrom"

typedef union {
    char data_char[4*CMDNUM_FROM_FLOATS];
    float data_flts[CMDNUM_FROM_FLOATS];
} int_FromCMD_union;

struct shared_memory_recvfrom_LINUXCMDApp
{
  int_FromCMD_union new_FromCMD;
};

struct shared_memory_recvfrom_LINUXCMDApp *shared_mem_ptr_recvfrom_LINUXCMDApp;
sem_t *recvfrom_LINUXCMDApp_mutex_sem;
int recvfrom_LINUXCMDApp_fd_shm;
char CMDto28x[4*CMDNUM_FROM_FLOATS];
//LINUXCMDApp

typedef union {
	float floatData[5];
	char charData[20];
} optiData_t;


// Print system error and exit
void error (char *msg)
{
    perror (msg);
    exit (1);
}

int_pose_union received; //data get from F28
char sbuf[BUFFSIZE];
int numCharsRead = 0;
int currentChar = 0;
int startChar = 0;
char data = '0';
char mychar;
int receiving_state = 0;
int receiving_count = 0;
int sem_count_send = 0; //for sem_getvalue
char planned_path[81];
optiData_t rxData;
char Sendarray[40];  // Only need 20 so oversized on purpose

/*
 * setup_serial()
 *   sets the serial port up at 115200 baud
 */
int setup_serial()
{    
  sd_setup(SERFILE); //starts non-blocking

  sd_ioflush();
}

int gs_quit = 0;
int gs_exit = 0;
int firsttime = 1;
/*  
* gs_killapp()
*   ends application safely
*
*/
void gs_killapp(int s)
{
	printf("\nTerminating\n");
	gs_quit = 1;
	gs_exit = 1;
	close(gs_udpcoms_skt);
	if(gs_udpcoms_skt > 0) close(gs_udpcoms_skt);
	return;
}

/*
* init_server()
*   Starts up listening socket
*/
int init_server()
{
	// udp: 
	if (firsttime) {
		gs_udpcoms_skt = udpsock();
		sock_set_nonblocking(gs_udpcoms_skt);      
		sockbind(gs_udpcoms_skt, UDP_PORT);
		printf("UDP Listening on port %d\n.", UDP_PORT);
	}
}

int run_client()
{
	int i = 0;
	int j = 0;
	static struct sockaddr_in cliaddr;
	unsigned int len = sizeof(cliaddr); 

  sched_yield();
  
  // read from the socket
  recvd = recvfrom(gs_udpcoms_skt,mesg,MSG_BUFFER,0,(struct sockaddr *)&cliaddr,&len);
  
  // data from optitrack python program
  if(recvd == packet_size)
  {
    void * ptr = mesg;
    for (i = 0; i < packet_size; i++) {
      rxData.charData[i] = *((char *)ptr);
      ptr += sizeof(char);
    }

    if (printpause > 10) {
      printf("OptiData: %.3f %.3f %.3f %.3f %.3f\n",rxData.floatData[0],rxData.floatData[1],rxData.floatData[2],rxData.floatData[3],rxData.floatData[4]);
      printpause = 0;
    }
    printpause++;
    
    Sendarray[0] = 'O';
    Sendarray[1] = 'p';
    Sendarray[2] = 't';
    Sendarray[3] = 'i';
    Sendarray[4] = ':';
    for (i=0; i < 20; i++) {
      Sendarray[i + 5] = rxData.charData[i];
    }
    sd_writen(Sendarray,25);
  } 
}

//blocking get keyboard input
int mygetch(void)
{
  struct termios oldt,
      newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
}

//non-blocking keyboard input detection
int _kbhit()
{
  static const int STDIN = 0;
  static int initialized = 0;

  if (!initialized)
  {
    // Use termios to turn off line buffering
    struct termios term;
    tcgetattr(STDIN, &term);
    term.c_lflag &= ~ICANON;
    tcsetattr(STDIN, TCSANOW, &term);
    setbuf(stdin, NULL);
    initialized = 1;
  }

  int bytesWaiting;
  ioctl(STDIN, FIONREAD, &bytesWaiting);
  return bytesWaiting;
}

int main()
{
  printf("Setting signal handler...\n");
  signal(SIGKILL, gs_killapp);
  signal(SIGINT, gs_killapp);
  printf("...OK\n");
  printf("Initializing serial port driver %s...\n", "/dev/ttyAMA1");
  setup_serial();
  printf("...OK\n");

  sd_set_nonblocking();
  printf(".\n");
  sd_ioflush();

  packet_size = 5*4;  // x,y,theta,number,framecount
  printf("Packet size is: %d\n",packet_size);
  printf("Initializing listening connection...\n");
  init_server();                                                  
  firsttime = 0;
  printf("UDP server...OK\n");
  
  int i = 0; //count variable


  // AstarApp Shared Memory  ************************
  struct shared_memory_sendto_AstarApp *shared_mem_ptr_sendto_AstarApp;
  struct shared_memory_readfrom_AstarApp *shared_mem_ptr_readfrom_AstarApp;
  sem_t *sendto_AstarApp_mutex_sem;
  sem_t *readfrom_AstarApp_mutex_sem;
  int sendto_AstarApp_fd_shm;
  int readfrom_AstarApp_fd_shm;
  //create the semaphore for send robot pose and obstacles to AstarApp 
  if ((sendto_AstarApp_mutex_sem = sem_open(SENDTO_ASTARAPP_SEM_MUTEX_NAME, O_CREAT, 0660, 0)) == SEM_FAILED)
      error("Error sendto AstarApp sem_open");
  //create the semaphore for read path 
  if ((readfrom_AstarApp_mutex_sem = sem_open(READFROM_ASTARAPP_SEM_MUTEX_NAME, O_CREAT, 0660, 0)) == SEM_FAILED)
      error("Error readfrom AstarApp sem_open");
  // create shared memory for send
  if ((sendto_AstarApp_fd_shm = shm_open(SENDTO_ASTARAPP_SHARED_MEM_NAME, O_RDWR | O_CREAT | O_EXCL, 0660)) == -1)
      error("Error sendto AstarApp shm_open");
  //set the size of the shared memory
  if (ftruncate(sendto_AstarApp_fd_shm, sizeof(struct shared_memory_sendto_AstarApp)) == -1)
      error("Error sendto AstarApp ftruncate");
  //map the memory to virtual address
  if ((shared_mem_ptr_sendto_AstarApp = mmap(NULL, sizeof(struct shared_memory_sendto_AstarApp), PROT_READ | PROT_WRITE, MAP_SHARED,
                              sendto_AstarApp_fd_shm, 0)) == MAP_FAILED)
      error("Error sendto AstarApp mmap");
  // create shared memory for read
  if ((readfrom_AstarApp_fd_shm = shm_open(READFROM_ASTARAPP_SHARED_MEM_NAME, O_RDWR | O_CREAT | O_EXCL, 0660)) == -1)
      error("Error readfrom AstarApp shm_open");
  //set the size of the shared memory
  if (ftruncate(readfrom_AstarApp_fd_shm, sizeof(struct shared_memory_readfrom_AstarApp)) == -1)
      error("Error readfrom AstarApp ftruncate");
  //map the memory to virtual address
  if ((shared_mem_ptr_readfrom_AstarApp = mmap(NULL, sizeof(struct shared_memory_readfrom_AstarApp), PROT_READ | PROT_WRITE, MAP_SHARED,
                              readfrom_AstarApp_fd_shm, 0)) == MAP_FAILED)
      error("Error readfrom AstarApp mmap");
// End AstarApp Shared Memory  ************************




// LVCOMApp Shared Memory  ************************
  struct shared_memory_sendto_LVCOMApp *shared_mem_ptr_sendto_LVCOMApp;
  struct shared_memory_readfrom_LVCOMApp *shared_mem_ptr_readfrom_LVCOMApp;
  sem_t *sendto_LVCOMApp_mutex_sem;
  sem_t *readfrom_LVCOMApp_mutex_sem;
  int sendto_LVCOMApp_fd_shm;
  int readfrom_LVCOMApp_fd_shm;
  //create the semaphore for send floats to LVCOMApp 
  if ((sendto_LVCOMApp_mutex_sem = sem_open(SENDTO_LVCOMAPP_SEM_MUTEX_NAME, O_CREAT, 0660, 0)) == SEM_FAILED)
      error("Error sendto LVCOMApp sem_open");
  //create the semaphore for read path 
  if ((readfrom_LVCOMApp_mutex_sem = sem_open(READFROM_LVCOMAPP_SEM_MUTEX_NAME, O_CREAT, 0660, 0)) == SEM_FAILED)
      error("Error readfrom LVCOMApp sem_open");
  // create shared memory for send
  if ((sendto_LVCOMApp_fd_shm = shm_open(SENDTO_LVCOMAPP_SHARED_MEM_NAME, O_RDWR | O_CREAT | O_EXCL, 0660)) == -1)
      error("Error sendto LVCOMApp shm_open");
  //set the size of the shared memory
  if (ftruncate(sendto_LVCOMApp_fd_shm, sizeof(struct shared_memory_sendto_LVCOMApp)) == -1)
      error("Error sendto LVCOMApp ftruncate");
  //map the memory to virtual address
  if ((shared_mem_ptr_sendto_LVCOMApp = mmap(NULL, sizeof(struct shared_memory_sendto_LVCOMApp), PROT_READ | PROT_WRITE, MAP_SHARED,
                              sendto_LVCOMApp_fd_shm, 0)) == MAP_FAILED)
      error("Error sendto LVCOMApp mmap");
  // create shared memory for read
  if ((readfrom_LVCOMApp_fd_shm = shm_open(READFROM_LVCOMAPP_SHARED_MEM_NAME, O_RDWR | O_CREAT | O_EXCL, 0660)) == -1)
      error("Error readfrom LVCOMApp shm_open");
  //set the size of the shared memory
  if (ftruncate(readfrom_LVCOMApp_fd_shm, sizeof(struct shared_memory_readfrom_LVCOMApp)) == -1)
      error("Error readfrom LVCOMApp ftruncate");
  //map the memory to virtual address
  if ((shared_mem_ptr_readfrom_LVCOMApp = mmap(NULL, sizeof(struct shared_memory_readfrom_LVCOMApp), PROT_READ | PROT_WRITE, MAP_SHARED,
                              readfrom_LVCOMApp_fd_shm, 0)) == MAP_FAILED)
      error("Error readfrom LVCOMApp mmap");
// End LVCOMApp Shared Memory  ************************

// LINUXCMDApp Shared Memory  ************************
  //create the semaphore for recv floats from LINUXCMDApp 
  if ((recvfrom_LINUXCMDApp_mutex_sem = sem_open(RECVFROM_LINUXCMDAPP_SEM_MUTEX_NAME, O_CREAT, 0660, 0)) == SEM_FAILED)
      error("Error recvfrom LINUXCMDApp sem_open");
  // create shared memory for 28x recv
  if ((recvfrom_LINUXCMDApp_fd_shm = shm_open(RECVFROM_LINUXCMDAPP_SHARED_MEM_NAME, O_RDWR | O_CREAT | O_EXCL, 0660)) == -1)
      error("Error recvfrom LINUXCMDApp shm_open");
  //set the size of the shared memory
  if (ftruncate(recvfrom_LINUXCMDApp_fd_shm, sizeof(struct shared_memory_recvfrom_LINUXCMDApp)) == -1)
      error("Error recvfrom LINUXCMDApp ftruncate");
  //map the memory to virtual address
  if ((shared_mem_ptr_recvfrom_LINUXCMDApp = mmap(NULL, sizeof(struct shared_memory_recvfrom_LINUXCMDApp), PROT_READ | PROT_WRITE, MAP_SHARED,
                              recvfrom_LINUXCMDApp_fd_shm, 0)) == MAP_FAILED)
      error("Error recvfrom LINUXCMDApp mmap");
// End LINUXCMDApp Shared Memory  ************************


  //stop when ctrl+c or 'q'
  while (!gs_quit)
  {
    int kb_state = _kbhit(); //check if hit keyboard
    if (kb_state == 1)
    {
      //read the entered character
      mychar = mygetch();
      if (mychar == 'q')
      { //exit if hit 'q'
        gs_quit = 1;
      }
    }

    run_client();
    ///////////////////////////////////////////////////////////////////////
    // Read from serial port and print everything upto the return character
    if (currentChar >= numCharsRead) {
        numCharsRead = sd_readn(sbuf, BUFFSIZE);  //nonblocking
        if (numCharsRead >=0){
            // printf("%d \n",numCharsRead);
        }
        startChar = 0;
    } else {
        startChar = currentChar;
    }
    //use as an interrupt to read ever character received
    if (numCharsRead > 0)
    {
      if (numCharsRead >= BUFFSIZE)
      {
        printf("Overflow without return\n");
        break;
      }
      
      for (currentChar = startChar; currentChar < numCharsRead; currentChar++)
      {
        data = sbuf[currentChar];
        if (receiving_state == 0) {
            if (data =='*') {
                receiving_state = 1; //check for the second *
            }
        } else if (receiving_state == 1) {
          if (data == '*') {
              receiving_state = 2; //if received second *, check the next 34 readings for AstarApp
              receiving_count = 0; 
          } else if (data == '$') {
              receiving_state = 12;  // LVCOMApp
              receiving_count = 0;
          } else {
              receiving_state = 0;
          }
        } else if (receiving_state == 2) { // F28379D is sending obstacle locations to AstarApp
          received.obs_data_char[receiving_count] = data; //put data in char array
          shared_mem_ptr_sendto_AstarApp->new_pose.obs_data_char[receiving_count] = data;//put data into shared memory
          receiving_count++;
          if (receiving_count == 34) {
            receiving_count = 0;
            //printout x, y, destrow, destcol
            printf("%.4f, %.4f, %d,%d \n",received.cur_obs.x,received.cur_obs.y,received.cur_obs.destrow,received.cur_obs.destcol);
            receiving_state = 0;
            if (sem_getvalue(sendto_AstarApp_mutex_sem,  &sem_count_send) == 0) {
              if (sem_post(sendto_AstarApp_mutex_sem) == -1){
                error("Error AstarApp sem_post: send_mutex");
              }
              //printf("posted\n");
            } else {
				printf("sendto_AstarApp_mutex_sem Not ready!\n");
			}	
          }
        } else if (receiving_state == 12) { // F28379D is sending 8 float values to LVCOMApp 
          shared_mem_ptr_sendto_LVCOMApp->new_ToLV.data_char[receiving_count] = data;//put data into shared memory
          receiving_count++;
          if (receiving_count == 4*LVNUM_TOFROM_FLOATS) {
            receiving_count = 0;
            receiving_state = 0;
            printf("angle:%.3f,x:%.3f,y:%.3f\n",shared_mem_ptr_sendto_LVCOMApp->new_ToLV.data_flts[2],shared_mem_ptr_sendto_LVCOMApp->new_ToLV.data_flts[0],shared_mem_ptr_sendto_LVCOMApp->new_ToLV.data_flts[1]);

            if (sem_getvalue(sendto_LVCOMApp_mutex_sem,  &sem_count_send) == 0) {
              if (sem_post(sendto_LVCOMApp_mutex_sem) == -1){
                error("Error serial_COMandOpti sem_post: send_mutex");
              }
              //printf("posted\n");
            } else {
			  printf("sendto_LVCOMApp_mutex_sem Not ready!\n");
			}
          }
        }
      }
    }

    //check if AstarApp readfrom semaphore posted, can get path data
    if (sem_trywait(readfrom_AstarApp_mutex_sem) == 0) {
      for (i = 0; i < 81; i++) {
        planned_path[i] = shared_mem_ptr_readfrom_AstarApp->read_path[i];
      }
      sd_write("*");// Data from AstarApp to F28379D
      sd_write("*");
      sd_writen(planned_path, 81);
    }


    //check if LVNumApp readfrom semaphore posted, can get data
    if (sem_trywait(readfrom_LVCOMApp_mutex_sem) == 0) {
      for (i = 0; i < 4*LVNUM_TOFROM_FLOATS; i++) {
        LVto28x[i] = shared_mem_ptr_readfrom_LVCOMApp->new_FromLV.data_char[i];
      }
      sd_write("*");// Data from LVCOMApp to F28379D
      sd_write("$");
      sd_writen(LVto28x,4*LVNUM_TOFROM_FLOATS);
    }

    //check if LINUXCMDApp readfrom semaphore posted, can get data
    if (sem_trywait(recvfrom_LINUXCMDApp_mutex_sem) == 0) {
      for (i = 0; i < 4*CMDNUM_FROM_FLOATS; i++) {
        CMDto28x[i] = shared_mem_ptr_recvfrom_LINUXCMDApp->new_FromCMD.data_char[i];
      }
      sd_write("*");// Data from LINUXCMDApp to F28379D
      sd_write("!");
      sd_writen(CMDto28x,4*CMDNUM_FROM_FLOATS);
    }
    sched_yield();
  }
  shm_unlink(SENDTO_ASTARAPP_SHARED_MEM_NAME);
  shm_unlink(READFROM_ASTARAPP_SHARED_MEM_NAME);
  sem_unlink(SENDTO_ASTARAPP_SEM_MUTEX_NAME);
  sem_unlink(READFROM_ASTARAPP_SEM_MUTEX_NAME);

  shm_unlink(SENDTO_LVCOMAPP_SHARED_MEM_NAME);
  shm_unlink(READFROM_LVCOMAPP_SHARED_MEM_NAME);
  sem_unlink(SENDTO_LVCOMAPP_SEM_MUTEX_NAME);
  sem_unlink(READFROM_LVCOMAPP_SEM_MUTEX_NAME);

  shm_unlink(RECVFROM_LINUXCMDAPP_SHARED_MEM_NAME);
  sem_unlink(RECVFROM_LINUXCMDAPP_SEM_MUTEX_NAME);

  return 0;
}

