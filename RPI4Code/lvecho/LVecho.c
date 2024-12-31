#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <fcntl.h>
#include <ctype.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/mman.h>

#include "LVecho.h"
int TCPIPtransmissionerror = 0;		// Initialize transmission error count
int TCPIPbeginnewdata = 0;
int TCPIPdatasize = 0;
int writeFromMainFlag = 0;
int numTXChars = 0;

#define INBUFFSIZE (256)
#define OUTBUFFSIZE (INBUFFSIZE + 2)
char TCPIPMessageArray[INBUFFSIZE];

//void *map_baseGPIO, *virt_addrGPIO;
//unsigned int *myGPIO;

char inbuf[INBUFFSIZE];
unsigned char outbuf[OUTBUFFSIZE];
int accepted_skt;

char buffer4SystemCall[512];
int size_buff = 0;




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
	//int on = 1;	
	
	for (i=0;i<64;i++) {
		inbuf[i] = '\0';
	}
	numrecChars = read(accepted_skt, inbuf, 64); 
	if (numrecChars > 0)  {
		for (i=0;i<numrecChars;i++) {
			if (!TCPIPbeginnewdata) {// Only true if have not yet begun a message
				if (253 == (unsigned char)inbuf[i]) {// Check for start char
					TCPIPdatasize = 0;		// amount of data collected in message set to 0
					TCPIPbeginnewdata = 1;		// flag to indicate we are collecting a message
				}
			} else {	// Filling data
				// Dont go too large... limit message to 256 chars  this can be made larger if you change inbuf size
				if ((TCPIPdatasize < 256) && ((unsigned char)inbuf[i] != 255)) {	
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
					printf("New Test String:%s\n",TCPIPMessageArray);
					
					//printf("/home/root/wavfiles/robotspeak.sh \"%s\"\n",TCPIPMessageArray);
					// size_buff = sprintf(buffer4SystemCall,"/home/root/wavfiles/robotspeak.sh \"%s\"",TCPIPMessageArray);
					// if (size_buff < 512) {
					// 	system(buffer4SystemCall);
					// }
					
					numTXChars = sprintf(((char *)outbuf),"Echo Msg:%s\r\n",TCPIPMessageArray);
					
					
					
					//memcpy(&outbuf,(void *)(&inbuf[0]),64);      
					//write(accepted_skt, outbuf, numTXChars);
					writeFromMainFlag = 1;
					TCPIPbeginnewdata = 0;	// Reset the flag
					TCPIPdatasize = 0;	// Reset the number of chars collected
				}	
			}
		}
	} else {
		printf("numrecChars = %d\n",numrecChars);
		printf("\nReseting\n");
		gs_quit = 1;
		close(gs_coms_skt);

	}

	


	// call this kill when kill string is sent
	//      gs_killapp(0);

	return;
}




/*
* run_server()
*   This function runs until the server quits and it:
*
*   1. accepts incoming TCP connections
*   2. reads from ready sockets and writes incoming messages
*      to the serial port / controls devices
*   3. reads from serial port and prints
*/
int run_server()
{

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
	while (!gs_quit) {
		sched_yield();
		if (writeFromMainFlag == 1) {
			writeFromMainFlag = 0;
			write(accepted_skt, outbuf, numTXChars);
		}
	}
}




/*
* main()
*   process command line input
*/
int main (int argc, char **argv)
{
	int index = 0;
	int firsttime = 1;
	printf("Echo Program\n");
	
	fflush(stdout);
	
	

	if (argc ==2) {
		printf("using port %s\n",argv[1]);
		gs_port_coms = atoi(argv[1]);
	}
	printf("Setting signal handler...\n");
	signal(SIGKILL, gs_killapp);
	signal(SIGINT, gs_killapp);
	printf("...OK\n");
	while (!gs_exit) {
		gs_quit = 0;
		if (!firsttime) {
			printf("Waiting 2 Seconds to make sure socket data is cleaned from the network.\n");
			sleep(2);
		}
		printf("Initializing listening connection...\n");
		init_server();
		firsttime = 0;
		printf("...OK\n");

		printf("Accepting connections...\n");
		run_server();
	}

}


