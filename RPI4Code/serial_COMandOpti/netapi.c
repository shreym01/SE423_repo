// Linux network API
// Vladimeros Vladimerou (c) 2008
// GPL license

#include "netapi.h"

/*
 * struct sockaddr_in {
 *     short            sin_family;   // e.g. AF_INET
 *     unsigned short   sin_port;     // e.g. htons(3490)
 *     struct in_addr   sin_addr;     // see struct in_addr, below
 *     char             sin_zero[8];  // zero this if you want to
 * };
 *
 * struct in_addr {
 *     unsigned long s_addr;  // load with inet_aton()
 * };
 */

/*
 * sock_set_blocking()
 *  converts the given socket to a blocking socket
 *
 * Inputs:
 *   int skt: the handle
 *
 * Outputs:
 *   zero on success
 */
int sock_set_blocking(int skt)
{
  long temp_flags;
  temp_flags = fcntl(skt, F_GETFL);
  if (temp_flags == -1) 
    perror("serial_dev::set_blocking get\n");
  temp_flags &= ~O_NONBLOCK;
  int i = fcntl(skt, F_SETFL, temp_flags);
  if (i == -1) 
    perror("serial_dev::set_blocking set \n");
  else 
    return 0;
    //return sd_flags = temp_flags;
}


/*
 * sock_set_nonblocking()
 *  converts the given socket to a blocking socket
 *
 * Inputs:
 *   int skt: the handle
 *
 * Outputs:
 *   zero on success
 */
int sock_set_nonblocking(int skt)
{
  long temp_flags;
  temp_flags = fcntl(skt, F_GETFL);
  if (temp_flags == -1) 
    perror("serial_dev::set_blocking get\n");
  temp_flags |= O_NONBLOCK;
  int i = fcntl(skt, F_SETFL, temp_flags);
  if (i == -1) 
    perror("serial_dev::set_blocking set \n");
  else 
    return 0;
    //return sd_flags = temp_flags;
}


/**
 * nslookup()
 *
 * Description:
 *      converts dnsname to 4 byte address
 * inputs: 
 *      char *dnsname: null-terminated string of dnsname to look up
 * output:
 *      unsigned int: long 4-byte internet address in network byte-order
 */
unsigned long nslookup(char *dnsname)
{
  struct hostent *host;
  struct in_addr h_addr;
  printf("Looking up %s\n", dnsname);
  if ((host = gethostbyname(dnsname)) == NULL) {
    perror("nslookup(): DNS lookup failed\n");
    exit(1);
  }
  //printf("test\n");
  h_addr.s_addr = *((unsigned long *) host->h_addr_list[0]);
  //printf("nslookup(): Name is %s\n", inet_ntoa(h_addr));
  //printf("test\n");
  return h_addr.s_addr;
}



/*
 * udpsock()
 *
 * Description:
 *      sets up a udp socket
 * Return value:
 *      socket handle
 */
int udpsock()
{
  int s;
  //if ((s = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
  if ((s = socket(PF_INET, SOCK_DGRAM, 0)) < 0) {
    perror("udpock(): socket creation failed\n");
    exit(1);
  }
  return s;
}       


/*
 * tcpsock()
 *
 * Description:
 *      sets up a tcp socket
 * Return value:
 *      socket handle
 */
int tcpsock()
{
  int s;
  if ((s = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0) {
    perror("tcpsock(): socket creation failed\n");
    exit(1);
  }
  return s;
}

/*
 * sockbind()
 *      Binds socket to local port
 * Inputs:
 *      int skt: the socket handle
 *      int prt: the port number
 * Outputs:
 *      zero on success
 */ 
int sockbind(int skt, int prt)
{
  int r;
  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(struct sockaddr_in));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(prt);
  addr.sin_addr.s_addr = INADDR_ANY; // my default address
  if (r = bind(skt, (struct sockaddr *)&addr, sizeof(addr))) {
    perror("sockbind(): bind failed");
    exit(r);
  }
}


/*
 * sockconn():
 *      connects a TCP/IP socket to a remote listening port
 * Inputs:
 *      int skt: local socket
 *      char *remote_ip: ip address of host to connect to
 *      int remote_port: port of host to connect to
 * Returns:
 *      zero on success
 */
int sockconn(int skt, char *remote_ip, int remote_port)
{
  int rtn;
  struct sockaddr_in dest_addr;
  dest_addr.sin_family = AF_INET;
  dest_addr.sin_port = htons(remote_port);
  dest_addr.sin_addr.s_addr = (nslookup(remote_ip));
  memset(dest_addr.sin_zero,'\0',sizeof(dest_addr.sin_zero));
  rtn = connect(skt, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
  if (rtn) {
    perror("sockconn(): connect failed\n");
    exit(errno);
  }
  else
    return 0;
}

/*
 * sockconn2():
 *      connects a TCP/IP socket to a remote listening port
 * Inputs:
 *      int skt: local socket
 *      unsigned char ip1.ip2.ip3.ip4: destn ip address in common format  (e.g. 127.0.0.1)
 *      int remote_port: port of host to connect to
 * Returns:
 *      zero on success
 */
int sockconn2(int skt,unsigned char ip1,
unsigned char ip2,unsigned char ip3,unsigned char ip4, int remote_port)
{
  int rtn;
  struct sockaddr_in dest_addr;
  struct in_addr in_dest_addr;
  dest_addr.sin_family = AF_INET;
  dest_addr.sin_port = htons(remote_port);
  dest_addr.sin_addr.s_addr = ip1|(ip2<<8)|(ip3<<16)|(ip4<<24);
  memset(dest_addr.sin_zero,'\0',sizeof(dest_addr.sin_zero));
  rtn = connect(skt, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
  if (rtn) {
    perror("sockconn(): connect failed\n");
    exit(errno);
  }
  else
    return 0;
}


/*
 * socklisten():
 *      listens for incoming TCP connections at given socket
 *      assuming backlog of 20
 * Inputs: 
 *      int skt: socket handle
 * Returns:
 *      zero on success, otherwise exits
 */
int socklisten(int skt)
{
  int r;
  r = listen(skt, 20);
  if (r) {
    perror("socklisten():error\n");
    exit(errno);
  }
  else 
    return 0;
}

/*
 * connaccept():
 *      accepts incoming connection from listening socket
 * Inputs:
 *      int skt: listening socket
 *      struct sockaddr *addr: address of incoming call
 * Returns:
 *      new connected socket
 *      modifies address structure given
 */
int connaccept(int skt, struct sockaddr_in *addr)
{
  int s;
  int addrsize = sizeof(struct sockaddr_in);
  s = accept(skt, (struct sockaddr *)addr, &addrsize);
  if (s == -1) {
    if (errno == EAGAIN ||errno == EWOULDBLOCK)
      return -1;
    else {
      perror("connaccept(): error\n");
      exit(s);
    }
  }
  return s;
}
