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
#include <stdint.h>
#include <sys/signal.h>
#include <termios.h>
#include <ctype.h>
#include "netapi.h"

int gs_coms_skt;  // device coms listening socket
int gs_port_coms = 10001; // Listening port

int gs_quit = 0;
int gs_exit = 0;

int TCPIPtransmissionerror = 0;		// Initialize transmission error count
int TCPIPbeginnewdata = 0;
int TCPIPdatasize = 0;
int numTXChars = 0;

#define INBUFFSIZE (256)
#define OUTBUFFSIZE (INBUFFSIZE + 2)
char TCPIPMessageArray[INBUFFSIZE];

//void *map_baseGPIO, *virt_addrGPIO;
//unsigned int *myGPIO;

char inbuf[INBUFFSIZE];
char outbuf[OUTBUFFSIZE];
int accepted_skt;

char buffer4SystemCall[512];
int size_buff = 0;

int sem_count_send = 0; //for sem_getvalue
float LVflt0 = 0;
float LVflt1 = 0;
float LVflt2 = 0;
float LVflt3 = 0;
float LVflt4 = 0;
float LVflt5 = 0;
float LVflt6 = 0;
float LVflt7 = 0;
#define LVNUM_TOFROM_FLOATS 8
float f28xtoLVFloat[LVNUM_TOFROM_FLOATS];
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
struct shared_memory_sendto_LVCOMApp *shared_mem_ptr_sendto_LVCOMApp;
struct shared_memory_readfrom_LVCOMApp *shared_mem_ptr_readfrom_LVCOMApp;
sem_t *sendto_LVCOMApp_mutex_sem;
sem_t *readfrom_LVCOMApp_mutex_sem;
int sendto_LVCOMApp_fd_shm;
int readfrom_LVCOMApp_fd_shm;
char wavbuf[256];

// Print system error and exit
void error (char *msg)
{
    perror (msg);
    exit (1);
}

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
	close(gs_coms_skt);
	return;
}


/*
* init_server()
*   Starts up listening socket
*/
int init_server(void)
{
	int on = 1;
	
	// Communications sockets,
	// tcp:
	gs_coms_skt = tcpsock();
	
	if (setsockopt(gs_coms_skt, SOL_SOCKET,SO_REUSEADDR, (char*)&on, sizeof(on)) < 0) {
		printf("Error Reusing Socket\n");
	}	
	sockbind(gs_coms_skt, gs_port_coms);
	socklisten(gs_coms_skt);
	printf("Listening on port %d\n.",gs_port_coms);
	sock_set_blocking(gs_coms_skt);
}

void sd_signal_handler_IO (int status)
{
	int i;
	int numrecChars = 0;
	
	for (i=0;i<INBUFFSIZE;i++) {
		inbuf[i] = '\0';
	}
	numrecChars = read(accepted_skt, inbuf, INBUFFSIZE); 
	if (numrecChars > 0)  {
		for (i=0;i<numrecChars;i++) {
			if (!TCPIPbeginnewdata) {// Only true if have not yet begun a message
				if (253 == (unsigned char)inbuf[i]) {// Check for start char
					TCPIPdatasize = 0;		// amount of data collected in message set to 0
					TCPIPbeginnewdata = 1;		// flag to indicate we are collecting a message
				}
			} else {	// Filling data
				// Dont go too large... limit message to outbuf chars  this can be made larger if you change inbuf size
				if ((TCPIPdatasize < INBUFFSIZE) && ((unsigned char)inbuf[i] != 255)) {	
					TCPIPMessageArray[TCPIPdatasize] = inbuf[i];				
					TCPIPdatasize++;
				} else {  // too much data or 255 char received
					if ((unsigned char)inbuf[i] != 255) {// check if end character recvd
						// Not received
						TCPIPtransmissionerror++;
						printf("TXerrors = %d\n",TCPIPtransmissionerror);
					} 
					// Whether or not message terminated correctly, send data to other tasks
					TCPIPMessageArray[TCPIPdatasize] = '\0'; 	// Null terminate the string 
					printf("Data:%s\n",TCPIPMessageArray);
					
					//printf("/home/root/wavfiles/robotspeak.sh \"%s\"\n",TCPIPMessageArray);
					// size_buff = sprintf(buffer4SystemCall,"/home/root/wavfiles/robotspeak.sh \"%s\"",TCPIPMessageArray);
					// if (size_buff < 512) {
					// 	system(buffer4SystemCall);
					// }
					  
					sscanf(TCPIPMessageArray,"%f %f %f %f %f %f %f %f",&LVflt0,&LVflt1,&LVflt2,&LVflt3,&LVflt4,&LVflt5,&LVflt6,&LVflt7);
					shared_mem_ptr_readfrom_LVCOMApp->new_FromLV.data_flts[0] = LVflt0;
					shared_mem_ptr_readfrom_LVCOMApp->new_FromLV.data_flts[1] = LVflt1;
					shared_mem_ptr_readfrom_LVCOMApp->new_FromLV.data_flts[2] = LVflt2;
					shared_mem_ptr_readfrom_LVCOMApp->new_FromLV.data_flts[3] = LVflt3;
					shared_mem_ptr_readfrom_LVCOMApp->new_FromLV.data_flts[4] = LVflt4;
					shared_mem_ptr_readfrom_LVCOMApp->new_FromLV.data_flts[5] = LVflt5;
					shared_mem_ptr_readfrom_LVCOMApp->new_FromLV.data_flts[6] = LVflt6;
					shared_mem_ptr_readfrom_LVCOMApp->new_FromLV.data_flts[7] = LVflt7;			
					if (sem_getvalue(readfrom_LVCOMApp_mutex_sem,  &sem_count_send) == 0) {
						if (sem_post(readfrom_LVCOMApp_mutex_sem) == -1){
							printf("Error LVCOMApp sem_post: send_mutex");
						}
					//printf("posted\n");
					}
					TCPIPbeginnewdata = 0;	// Reset the flag
					TCPIPdatasize = 0;	// Reset the number of chars collected
				}	
			}
		}
	} else {
		printf("numrecChars = %d\n",numrecChars);
		printf("\nQuiting\n");
		gs_quit = 1;
		close(gs_coms_skt);

	}

}





/*
* main()
*   process command line input
*/
int main (int argc, char **argv)
{
	int i = 0;
	printf("Echo Program\n");
	
	fflush(stdout);
	 
    //create the semaphore for send 
    if ((sendto_LVCOMApp_mutex_sem = sem_open(SENDTO_LVCOMAPP_SEM_MUTEX_NAME, 0, 0, 0)) == SEM_FAILED)
        error("Error send LVCOMApp sem_open");
    //create the semaphore for read path 
    if ((readfrom_LVCOMApp_mutex_sem = sem_open(READFROM_LVCOMAPP_SEM_MUTEX_NAME, 0, 0, 0)) == SEM_FAILED)
        error("Error read sem_open LVCOMApp");

    // create shared memory for send
    if ((sendto_LVCOMApp_fd_shm = shm_open(SENDTO_LVCOMAPP_SHARED_MEM_NAME, O_RDWR, 0)) == -1)
        error("Error shm_open LVCOMApp");

    //map the memory to virtual address
    if ((shared_mem_ptr_sendto_LVCOMApp = mmap(NULL, sizeof(struct shared_memory_sendto_LVCOMApp), PROT_READ | PROT_WRITE, MAP_SHARED,
                                sendto_LVCOMApp_fd_shm, 0)) == MAP_FAILED)
        error("Error mmap LVCOMApp");

    // create shared memory for read
    if ((readfrom_LVCOMApp_fd_shm = shm_open(READFROM_LVCOMAPP_SHARED_MEM_NAME, O_RDWR, 0)) == -1)
        error("Error shm_open LVCOMApp");

    //map the memory to virtual address
    if ((shared_mem_ptr_readfrom_LVCOMApp = mmap(NULL, sizeof(struct shared_memory_readfrom_LVCOMApp), PROT_READ | PROT_WRITE, MAP_SHARED,
                                readfrom_LVCOMApp_fd_shm, 0)) == MAP_FAILED)
        error("Error mmap LVCOMApp");


	if (argc ==2) {
		printf("using port %s\n",argv[1]);
		gs_port_coms = atoi(argv[1]);
	}
	printf("Setting signal handler...\n");
	signal(SIGKILL, gs_killapp);
	signal(SIGINT, gs_killapp);
	printf("...OK\n");
	printf("Initializing listening connection...\n");
	init_server();

	printf("Accepting connections...\n");
	sigset_t sigio_set; 
	struct sigaction saio;           /* definition of signal action */
	static struct sockaddr_in addr;  

	// Accept new connection
	accepted_skt = connaccept(gs_coms_skt, &addr);
	// We don't wanna block either source
	sock_set_nonblocking(accepted_skt);
	int flags = fcntl(accepted_skt, F_GETFL,0);
	fcntl (accepted_skt,F_SETFL,flags|FASYNC);
	fcntl(accepted_skt, F_SETOWN, getpid());

	sigemptyset(&sigio_set);
	sigaddset(&sigio_set,SIGIO); 
	saio.sa_handler = sd_signal_handler_IO;
	saio.sa_flags = SA_SIGINFO;
	sigemptyset(&saio.sa_mask);
	saio.sa_mask = sigio_set;
	if (sigaction(SIGIO,&saio,NULL)) {
		printf("Error in sigaction()\n");
		return 1;
	}

	if (accepted_skt>0) {
		printf("Connection accepted from %d.%d.%d.%d\n",
		(addr.sin_addr.s_addr&0x000000ff),      
		(addr.sin_addr.s_addr&0x0000ff00)>>8,
		(addr.sin_addr.s_addr&0x00ff0000)>>16,
		(addr.sin_addr.s_addr&0xff000000)>>24);
	}
	printf(".\n");
	while (!gs_exit) {
		sched_yield();

		if (sem_trywait(sendto_LVCOMApp_mutex_sem) == 0) {
			for (i = 0; i < LVNUM_TOFROM_FLOATS; i++) {
				f28xtoLVFloat[i] = shared_mem_ptr_sendto_LVCOMApp->new_ToLV.data_flts[i];
			}
			if (f28xtoLVFloat[7] > 0.5) {
				sprintf(wavbuf,"/usr/bin/aplay /home/pi/AWS5_SJWEISS3/LVCOMApp/GameOver.wav");
				system(wavbuf);
			}
			numTXChars = sprintf(outbuf,"%f %f %f %f %f %f %f %f\r\n",f28xtoLVFloat[0],f28xtoLVFloat[1],f28xtoLVFloat[2],f28xtoLVFloat[3],f28xtoLVFloat[4],f28xtoLVFloat[5],f28xtoLVFloat[6],f28xtoLVFloat[7]);
			write(accepted_skt, outbuf, numTXChars);
		}
		
	}

}


