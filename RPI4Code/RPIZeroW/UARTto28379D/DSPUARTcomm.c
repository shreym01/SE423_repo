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
#include "DSPUARTcomm.h"

#define DELAYTIME 100000
#define SERFILE "/dev/ttyAMA0"

// functions declared at bottom of this file
int mygetch(void);
void gs_killapp(int s);
int init_server(void);
void tcp_signal_handler_IO (int status);
int run_server(void);

int TCPIPtransmissionerror = 0;		// Initialize transmission error count
int TCPIPbeginnewdata = 0;
int TCPIPdatasize = 0;
int numTXChars = 0;

long tcpipcount = 0;

#define INBUFFSIZE (2048)
#define OUTBUFFSIZE (INBUFFSIZE + 2)
volatile char TCPIPMessageArray[INBUFFSIZE];
volatile char tempReadbackToCleanCache[INBUFFSIZE];
volatile int tempReadbackSize;

volatile unsigned int delaycount = 0;

char inbuf[INBUFFSIZE];  // TCPIP input and output buffers
unsigned char outbuf[OUTBUFFSIZE];
int accepted_skt;

int firsttime = 1;

#define SERIALBUFFSIZE 1024
char DanTXBuff[SERIALBUFFSIZE];
/*
 * setup_serial()
 *   sets the serial port up at 115200 baud
 */
int setup_serial()
{ 
  sd_setup(SERFILE);//starts non-blocking
  sd_ioflush();
}

/*
* main()
*   process command line input
*/
int main (int argc, char **argv)
{
	int index = 0;
	char buffer[200];  // used by fgets to read character string typed by user.
	char mychar;
	int inputdone = 0;
	float tempfloat = 0;
	float DVel = 0;
	float turn = 0.0;

	printf("Initializing serial port driver %s...\n",SERFILE);
	setup_serial();
	printf("...OK\n");
	
	//sd_set_nonblocking();
	sd_set_blocking();
	printf(".\n");
	sd_ioflush();


	// stay in this while loop receiving input from the user until user exits by selecting option e or v  
	while (!inputdone) {
		printf("\n\n");
		printf("Menu of Selections\n");
		printf("DO NOT PRESS CTRL-C when in this menu selection\n");
		printf("e - Exit Application\n");
		printf("v - Done.  Connect to LabView\n");
		printf("s - enter Desired Velocity Setpoint\n");
		printf("q - increment Left\n");
		printf("p - increment Right\n");
		mychar = (char) mygetch();
		
		switch (mychar) {
		case 'q':
			if (turn > 0.0) {
				turn = 0.0;
			} else {
				turn = turn - 0.2;
			}
			printf("turn =%.3f\n",turn);
			
			// send new turn value to DSP
			sprintf(DanTXBuff,"%.5f\n",turn);
		    sd_write(DanTXBuff);
			break;
		case 'p':                                
			if (turn < 0.0) {
				turn = 0.0;
			} else {
				turn = turn + 0.2;
			}
			printf("turn =%.3f\n",turn);
			
			// send new turn value to DSP
			sprintf(DanTXBuff,"%.5f\n",turn);
		    sd_write(DanTXBuff);
			break;
		case 'v':
			inputdone = 2;
			break;
		case 'e':
			inputdone = 1;
			break;
		case 's':
			// request new Velocity reference point from user and send it to DSP
			printf("Enter Desired Velocity Setpoint\n");
			fgets(buffer,190,stdin); 
			buffer[strlen(buffer)-1] = '\0';  // get ride of '\n' in returned string
			if (buffer[0] != '\0') {
				if (sscanf(buffer,"%f",&tempfloat) != 0) {  // check that it was a number entered
					DVel = tempfloat;
					printf("DVel = %.3f\n",DVel);
				}  else {
					printf("Error: Non numerical value typed\n");
				}
			} else {
				printf("Error: DVel not changed\n");
			}
			// send new Vref value to DSP
			sprintf(DanTXBuff,"%.5f\n",DVel);
		    sd_write(DanTXBuff);
			
			break;
		default:
			
			break;
		}
	} 

	
	if (inputdone == 2) {  // Start TCPIP connection with LabView application    
		if (argc ==2) {  // port number enter when starting this application
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
			run_server();  // sit in this function until TCPIP connection disconnects
		}	
	}
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
int run_server(void)
{

	sigset_t sigio_set; 
	struct sigaction saio;           /* definition of signal action */
	static struct sockaddr_in addr;  
	int i;

	// Accept new connection
	accepted_skt = connaccept(gs_coms_skt, &addr);
	// We don't wanna block either source
	sock_set_nonblocking(accepted_skt);
	int flags = fcntl(accepted_skt, F_GETFL,0);
	fcntl (accepted_skt,F_SETFL,flags|FASYNC);
	fcntl(accepted_skt, F_SETOWN, getpid());

	sigemptyset(&sigio_set);  // set up call back function called when new data received over ethernet. function "tcp_signal_handler_IO"
	sigaddset(&sigio_set,SIGIO); 
	saio.sa_handler = tcp_signal_handler_IO;
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
	while (!gs_quit) {  // sit inside this while checking for new data from the DSP until TCPIP connection disconnected
		sched_yield();  // allow other processes to run if necessary
		
		
	}
}



void tcp_signal_handler_IO (int status)
{
	int i,j;
	int numrecChars = 0;
	
	for (i=0;i<INBUFFSIZE/2;i++) {
		inbuf[i] = '\0';
	}
	numrecChars = read(accepted_skt, inbuf, INBUFFSIZE/2);  // read string of characters from TCPIP socket 
	if (numrecChars > 0)  {  // if 1 or more characters read
		for (i=0;i<numrecChars;i++) {  // loop through all the recieved characters
			if (!TCPIPbeginnewdata) {// Only true if have not yet begun a message
				if (253 == (unsigned char)inbuf[i]) {// Check for start char
					TCPIPdatasize = 0;		// amount of data collected in message set to 0
					TCPIPbeginnewdata = 1;		// flag to indicate we are now collecting a message
				}
			} else {	// Filling data
				// Dont go too large... limit message to 256 chars  this can be made larger if you change inbuf size
				if ((TCPIPdatasize < INBUFFSIZE) && ((unsigned char)inbuf[i] != 255)) {  // also check that it is not stop character	
					TCPIPMessageArray[TCPIPdatasize] = inbuf[i];  // add character to message array				
					TCPIPdatasize++;  // increment number of characters in message
				} else {  // too much data or 255 char received
					if ((unsigned char)inbuf[i] != 255) {// check if end character recvd if not print overflow times
						// Not received
						TCPIPtransmissionerror++;
						printf("TXerrors = %d\n",TCPIPtransmissionerror);
					} 
					// Whether or not message terminated correctly, send data to other tasks
					TCPIPMessageArray[TCPIPdatasize] = '\0'; 	// Null terminate the string
					TCPIPdatasize++;  // increment by one so that the Null is copied to shared memory 
					printf("New Test String:%s %d\n",TCPIPMessageArray,TCPIPdatasize);

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




int mygetch(void)
{
	struct termios oldt,
	newt;
	int ch;
	tcgetattr( STDIN_FILENO, &oldt );
	newt = oldt;
	newt.c_lflag &= ~( ICANON | ECHO );
	tcsetattr( STDIN_FILENO, TCSANOW, &newt );
	ch = getchar();
	tcsetattr( STDIN_FILENO, TCSANOW, &oldt );
	return ch;
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