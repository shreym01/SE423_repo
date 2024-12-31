#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <math.h>
#include <errno.h>
#include <signal.h>
#include <fcntl.h>
#include <ctype.h>
#include <termios.h>
#include <sys/mman.h>
#include "Practical.h"

int mygetch(void);

#define TCPSENDSIZE 1024
char TCPSendBuff[TCPSENDSIZE];
char commandline[TCPSENDSIZE/2];
#define TCPRECSIZE 1024
char TCPRecBuff[TCPRECSIZE];
int16_t TCPRecBuffLen = 0;
int16_t TCPSendBuffLen = 0;
int16_t numBytes = 0;
float turn = 0;
float RecTurn = 0;
float fboffset=0.3;

int main(int argc, char *argv[]) {

  char mychar;
  int inputdone = 0;
  
//  if (argc < 3 || argc > 4) // Test for correct number of arguments
//    DieWithUserMessage("Parameter(s)",
//        "<Server Address/Name> <Echo Word> [<Server Port/Service>]");

//  char *server = argv[1];     // First arg: server address/name
//  char *echoString = argv[2]; // Second arg: string to echo
  // Third arg (optional): server port/service
//  char *service = (argc == 4) ? argv[3] : "echo";

  char server[] = "192.168.1.59";
  char service[] = "1336";
   
  // Create a connected TCP socket
  int sock = SetupTCPClientSocket(server, service);
  if (sock < 0)
    DieWithUserMessage("SetupTCPClientSocket() failed", "unable to connect");

//  size_t echoStringLen = strlen(echoString); // Determine input length
// stay in this while loop receiving input from the user until user exits by selecting option e or v  
  while (!inputdone) {
		printf("\n\n");
		printf("Menu of Selections\n");
		printf("DO NOT PRESS CTRL-C when in this menu selection\n");
		//printf("WASD - increment Forward/Left/Back/Right\n");
		//printf("f - enter Desired Velocity Setpoint\n");
		printf("a - increment Left\n");
		printf("d - increment Right\n");
		//printf("w - increment Forward\n");
		//printf("s - increment Backward\n");
		//printf("r - reset forward/back motion\n");
		printf("q - reset turn\n");
		printf("g - get the Current Turn value");
		printf("e - Exit Application\n");
		mychar = (char) mygetch();
		
		switch (mychar) {
		case 'a': //left turn
			if (turn > 0.0) {
				turn = 0.0;
			} else {
				turn = turn - 0.2;
			}
			printf("turn =%.3f\n",turn);
			
			// send new turn value to ESP8266
			TCPSendBuffLen = sprintf(TCPSendBuff,"*%.5f\n",turn); //t for turn
			  // Send the string to the server
			numBytes = send(sock, TCPSendBuff, TCPSendBuffLen, 0);
			if (numBytes < 0) {
				DieWithSystemMessage("send() failed");
			} else if (numBytes != TCPSendBuffLen) {
				DieWithUserMessage("send()", "sent unexpected number of bytes");
			}
			
			break;
		case 'd':  //right turn                              
			if (turn < 0.0) {
				turn = 0.0;
			} else {
				turn = turn + 0.2;
			}
			printf("turn =%.3f\n",turn);
			
			// send new turn value to ESP8266
			TCPSendBuffLen = sprintf(TCPSendBuff,"*%.5f\n",turn);
			  // Send the string to the server
			numBytes = send(sock, TCPSendBuff, TCPSendBuffLen, 0);
			if (numBytes < 0) {
				DieWithSystemMessage("send() failed");
			} else if (numBytes != TCPSendBuffLen) {
				DieWithUserMessage("send()", "sent unexpected number of bytes");
			}
			break;
			
			
		case 'q': //reset turn                                
			turn = 0.0;
			printf("turn =%.3f\n",turn);
			
			// send new turn value to ESP8266
			TCPSendBuffLen = sprintf(TCPSendBuff,"*%.5f\n",turn);
			  // Send the string to the server
			numBytes = send(sock, TCPSendBuff, TCPSendBuffLen, 0);
			if (numBytes < 0) {
				DieWithSystemMessage("send() failed");
			} else if (numBytes != TCPSendBuffLen) {
				DieWithUserMessage("send()", "sent unexpected number of bytes");
			}
			break;
			
			
		// case 'w':  //forward                              
		// 	fboffset = fboffset + 0.1;
			
		// 	printf("fboffset =%.3f\n",fboffset);
			
		// 	// send new turn value to ESP8266
		// 	TCPSendBuffLen = sprintf(TCPSendBuff,"b%.5f\n",fboffset);
		// 	  // Send the string to the server
		// 	numBytes = send(sock, TCPSendBuff, TCPSendBuffLen, 0);
		// 	if (numBytes < 0) {
		// 		DieWithSystemMessage("send() failed");
		// 	} else if (numBytes != TCPSendBuffLen) {
		// 		DieWithUserMessage("send()", "sent unexpected number of bytes");
		// 	}
		// 	break;
			
		// case 's':  //backward                              
		// 	fboffset = fboffset - 0.1;
		// 	printf("fboffset =%.3f\n",fboffset);
		// 	// send new turn value to ESP8266
		// 	TCPSendBuffLen = sprintf(TCPSendBuff,"b%.5f\n",fboffset);
		// 	  // Send the string to the server
		// 	numBytes = send(sock, TCPSendBuff, TCPSendBuffLen, 0);
		// 	if (numBytes < 0) {
		// 		DieWithSystemMessage("send() failed");
		// 	} else if (numBytes != TCPSendBuffLen) {
		// 		DieWithUserMessage("send()", "sent unexpected number of bytes");
		// 	}
		// 	break;	
			
		// case 'r':  //reset direction of motion                              
		// 	fboffset = 0.27;
		// 	printf("fboffset =%.3f\n",fboffset);
		// 	// send new turn value to ESP8266
		// 	TCPSendBuffLen = sprintf(TCPSendBuff,"b%.5f\n",fboffset);
		// 	  // Send the string to the server
		// 	numBytes = send(sock, TCPSendBuff, TCPSendBuffLen, 0);
		// 	if (numBytes < 0) {
		// 		DieWithSystemMessage("send() failed");
		// 	} else if (numBytes != TCPSendBuffLen) {
		// 		DieWithUserMessage("send()", "sent unexpected number of bytes");
		// 	}
		// 	break;	
		
		case 'g':
			printf("Request Current Turn value\n");
			
			// send new turn value to ESP8266
			TCPSendBuffLen = sprintf(TCPSendBuff,"*a");
			  // Send the string to the server
			numBytes = send(sock, TCPSendBuff, TCPSendBuffLen, 0);
			if (numBytes < 0) {
				DieWithSystemMessage("send() failed");
			} else if (numBytes != TCPSendBuffLen) {
				DieWithUserMessage("send()", "sent unexpected number of bytes");
			} else {
				usleep(100000);
				TCPRecBuffLen = recv(sock, TCPRecBuff, TCPRECSIZE - 1, 0);
				if (TCPRecBuffLen < 0) {
     				DieWithSystemMessage("recv() failed");
				} else if (TCPRecBuffLen == 0) {
     				DieWithUserMessage("recv()", "connection closed prematurely");
				}
   				TCPRecBuff[TCPRecBuffLen] = '\0';    // Terminate the string!
				sscanf(TCPRecBuff,"%f",&RecTurn);
				printf("RecStr %s, RecFloat = %.5f",TCPRecBuff,RecTurn);
			}
			break;	
		case 'e':
			inputdone = 1;
			break;
		// case 'f':
		// 	// request new Velocity reference point from user and send it to DSP
		// 	printf("Enter Desired Velocity Setpoint\n");
		// 	fgets(commandline,190,stdin); 
		// 	sprintf(TCPSendBuff,"v%s\n",commandline);
		// 	TCPSendBuffLen = strlen(TCPSendBuff);
			
		// 	numBytes = send(sock, TCPSendBuff, TCPSendBuffLen, 0);
		// 	if (numBytes < 0) {
		// 		DieWithSystemMessage("send() failed");
		// 	} else if (numBytes != TCPSendBuffLen) {
		// 		DieWithUserMessage("send()", "sent unexpected number of bytes");
		// 	}
			
		// 	break;
		default:
			
			break;
		}
  }

 

  // Receive the same string back from the server
//  unsigned int totalBytesRcvd = 0; // Count of total bytes received
//  fputs("Received: ", stdout);     // Setup to print the echoed string
//  while (totalBytesRcvd < echoStringLen) {
//    char buffer[BUFSIZE]; // I/O buffer
    // Receive up to the buffer size (minus 1 to leave space for
    // a null terminator) bytes from the sender
//    numBytes = recv(sock, buffer, BUFSIZE - 1, 0);
//    if (numBytes < 0)
//      DieWithSystemMessage("recv() failed");
//    else if (numBytes == 0)
//      DieWithUserMessage("recv()", "connection closed prematurely");
//    totalBytesRcvd += numBytes; // Keep tally of total bytes
//    buffer[numBytes] = '\0';    // Terminate the string!
//    fputs(buffer, stdout);      // Print the buffer
//  }

  fputc('\n', stdout); // Print a final linefeed

  close(sock);
  exit(0);
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

