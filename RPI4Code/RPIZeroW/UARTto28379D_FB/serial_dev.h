#ifndef SERIAL_DEV_H
#define SERIAL_DEV_H

#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/signal.h>
#include <sys/types.h>
#include <sys/time.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <asm/ioctls.h>
#include <linux/serial.h>
#define _POSIX_SOURCE 1         //POSIX compliant source

/*
 * Very useful serial interface - you might wanna use it
 * if you want to change the sickd or customise it
 */

  // char sd_dev_name[256];
  // int sd_dev_fd;
  // struct termios sd_options;
  // long sd_flags;
  // int sd_blocking;

  int sd_setup(char *dn);
  int sd_kill();
  int sd_set_blocking();
  int sd_set_nonblocking();
  int sd_get_options();
  int sd_set_options();
  int sd_set_speed(int speed);
  int sd_get_speed();
  ssize_t sd_write(char *stringdata);
  ssize_t sd_read(size_t len);
  ssize_t sd_writen(char *buf, size_t count);
  ssize_t sd_readn(char *buf, size_t count);
  int sd_setflags(int flags);
  int sd_getflags(int flags);
  int sd_iflush();
  int sd_oflush();
  int sd_ioflush();
  // char sd_data[256];

#endif
