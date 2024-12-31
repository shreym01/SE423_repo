#include "serial_dev.h"
  char sd_data[256];
  char sd_dev_name[256];
  int sd_dev_fd;
  struct termios sd_options;
  long sd_flags;
  int sd_blocking;
int sd_setup(char *dn)
{
	
//  struct serial_struct serinfo;
	
	
  sd_dev_fd = open(dn, O_RDWR| O_NOCTTY | O_NONBLOCK);
  strcpy(sd_dev_name, dn);
  if (sd_dev_fd < 0)
  {
    perror(dn);
    exit(-1);
  }
  sd_ioflush();
  
  // added to change to ASYNC_LOW_LATENCY
  // Not sure if this helps at all or is doing anything
//  ioctl (sd_dev_fd, TIOCGSERIAL, &serinfo);
//  serinfo.flags |= ASYNC_LOW_LATENCY;
//  ioctl (sd_dev_fd, TIOCSSERIAL, &serinfo);
  
  
  tcgetattr(sd_dev_fd, &sd_options);
  sd_options.c_cflag = B115200| CS8 | CLOCAL | CREAD;
  sd_options.c_iflag = IGNPAR;
  sd_options.c_oflag = 0;
  sd_options.c_lflag = 0; 
  sd_options.c_cc[VMIN]=1;
  sd_options.c_cc[VTIME]=0;
  //cfmakeraw(&options);
  tcsetattr(sd_dev_fd, TCSANOW ,&sd_options);
  printf("set\n");
}

int sd_set_blocking()
{
  long temp_flags;
  temp_flags = fcntl(sd_dev_fd, F_GETFL);
  if (temp_flags == -1) 
    perror("serial_dev::set_blocking get\n");
  temp_flags &= ~O_NONBLOCK;
  int i = fcntl(sd_dev_fd, F_SETFL, temp_flags);
  if (i == -1) 
    perror("serial_dev::set_blocking set \n");
  else {
    return sd_flags = temp_flags;
    sd_blocking = 1;
  }
}

int sd_set_nonblocking()
{
  long temp_flags;
  temp_flags = fcntl(sd_dev_fd, F_GETFL);
  if (temp_flags == -1) 
    perror("serial_dev::set_nonblocking get\n");
  temp_flags |= O_NONBLOCK;
  int i = fcntl(sd_dev_fd, F_SETFL, temp_flags);
  if (i == -1) {
    perror("serial_dev::set_nonblocking set \n");
    return i;
  }
  else {
    return sd_flags = temp_flags;
    sd_blocking = 0;
  }
}

int sd_iflush()
{
  return tcflush(sd_dev_fd, TCIFLUSH);
}

int sd_oflush()
{
  return tcflush(sd_dev_fd, TCOFLUSH);
}

int sd_ioflush()
{
  return tcflush(sd_dev_fd, TCIOFLUSH);
}

ssize_t sd_writen(char *buf, size_t count)
{
  size_t orig_count = count;
  int temp;
  if (sd_blocking) {
    while(count > 0) {
      temp =  write(sd_dev_fd, buf, count);
      if (temp == -1)
        perror("serial_dev::write");
      count -= temp;
      buf += temp;
    }
    return orig_count;
  }
  else {
    return write(sd_dev_fd, buf, count);
  }
}

ssize_t sd_readn(char *buf, size_t count)
{
  size_t orig_count = count;
  int temp;
  if (sd_blocking) {
    while(count > 0) {
      temp = read(sd_dev_fd, buf, count);
      if (temp == -1)
        perror("serial_dev::read");
      count -= temp;
      buf += temp;
    }
    return orig_count;
  }
  else {
      return read(sd_dev_fd, buf, count);
  }
}

ssize_t sd_write(char *stringdata)
{ 
  size_t len = strlen(stringdata);
  return sd_writen(stringdata, len);
}

ssize_t sd_read(size_t len)
{
  return sd_readn(sd_data, len);
}

int sd_get_speed()
{
  sd_get_options();
  speed_t code = cfgetospeed(&sd_options);
  switch (code) {
    case B0:     return(0);
    case B300:   return(300);
    case B1200:  return(1200);
    case B2400:  return(2400);
    case B4800:  return(4800);
    case B9600:  return(9600);
    case B19200: return(19200);
    case B38400: return(38400);
    case B57600: return(57600);
    default: return(115200);
  };

}

int sd_set_speed(int speed)
{
  unsigned int  rate;
  if (speed < 300)
    rate = B0;
  else if (speed < 1200)
    rate =  B300;
  else if (speed < 2400)
    rate =  B1200;
  else if (speed < 4800)
    rate =  B2400;
  else if (speed < 9600)
    rate =  B4800;
  else if (speed < 19200)
    rate =  B9600;
  else if (speed < 38400)
    rate =  B19200;
  else if (speed < 57600)
    rate =  B38400;
  else if (speed < 115200)
    rate =  B57600;
  else
    rate =  B115200;

  sd_get_options();
  if(cfsetispeed(&sd_options, rate) != 0)
    perror("Failed setting input speed\n");
  if(cfsetospeed(&sd_options, rate) != 0)
    perror("Failed setting output speed\n");
  return sd_set_options();
}

int sd_get_options()
{
  return tcgetattr(sd_dev_fd, &sd_options);
}

int sd_set_options()
{
  return tcsetattr(sd_dev_fd,TCSANOW, &sd_options);
}

int sd_kill()
{
  close(sd_dev_fd);
}



