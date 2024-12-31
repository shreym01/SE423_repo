// Linux network API
// Vladimeros Vladimerou (c) 2008
// GPL license

#ifndef NETAPI_H
#include <stdio.h>          /* stderr, stdout */
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>      
#include <sys/socket.h>     /* for bind() */
#include <unistd.h>
#include <fcntl.h>
#include <netdb.h>          /* hostent struct, gethostbyname() */
#include <arpa/inet.h>      /* inet_ntoa() to format IP address */
#include <netinet/in.h>     /* in_addr structure */
#include <errno.h>

int sock_set_blocking(int skt);
int sock_set_nonblocking(int skt);
unsigned long nslookup(char *dnsname);
int tcpsock();
int udpsock();
int sockbind(int skt, int prt);
int sockconn(int skt, char *remote_ip, int remote_port);
int sockconn2(int skt, unsigned char ip1, unsigned char ip2,
unsigned char ip3, unsigned char ip4, int remote_port);
int socklisten(int skt);
int connaccept(int skt, struct sockaddr_in *addr);
int sock_set_blocking(int skt);
int sock_set_nonblocking(int skt);

#endif  // (ifndef NETAPI_H)



