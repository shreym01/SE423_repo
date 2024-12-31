//compile C code:
//gcc sem_for_py.c -lrt -lpthread -o test 
//--------------------------------------------------
//generate .so:
//cc -fPIC -shared sem_for_py.c -lrt -lpthread -o plot_sem.so

//functions for single semaphore in python 
//usage: 
// from ctypes import *
// so_file = "/home/pi/PATH_TO_FILE/test.so"
// fcn = CDLL(so_file)
// fcn.my_sem_open()
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <semaphore.h>
#include <sys/mman.h>

int get_value;
int sem_count;
sem_t *mutex_sem;
#define SEM_MUTEX_NAME "/sem-new-lidar-dist"
//open the semaphore named sem-test
int my_sem_open(){
  return ((mutex_sem = sem_open (SEM_MUTEX_NAME, 0, 0, 0)) == SEM_FAILED);
}
//unlink the semaphore
int my_sem_unlink(){
  return sem_unlink(SEM_MUTEX_NAME);
}
//post (add 1)
int my_sem_post(){
  return sem_post(mutex_sem);
}
//wait block until greater than 0
int my_sem_wait(){
  return sem_wait(mutex_sem);
}
//trywait return -1 if not availble
int my_sem_trywait(){
  return sem_trywait(mutex_sem);
}
// int my_sem_timedwait(int time) {
//   return 0; 
//   //https://stackoverflow.com/questions/37010836/how-to-correctly-use-sem-timedwait
// }
int my_sem_close(){
  return sem_close(mutex_sem);
}
//get the value of semaphore(number of post-number of wait)
int my_sem_getvalue() {
  get_value = sem_getvalue(mutex_sem,  &sem_count); 
  if (get_value == 0) {
    return abs(sem_count);
  } else {
    return -1;
  }
}

// int main(){
//   my_sem_open();
//   return my_sem_post();
// }