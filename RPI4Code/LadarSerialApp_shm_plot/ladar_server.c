#include "ladar_server.h"

// Buffer data structures
#define MAX_BUFFERS 10

#define SEM_MUTEX_NAME "/sem-new-ladar-dist"
#define SHARED_MEM_NAME "/posix-shared-mem-ladar-dist"

struct shared_memory
{
  float ladar_dist[228];
};

void error(char *msg);

#define SERFILE "/dev/ttyAMA2"
#define LADAR_MAX_READING 5700

/* 
 * gs_killapp()
 *   ends application safely, will enter this after ctrl+C
 *
 */
void gs_killapp(int s)
{
  printf("\nTerminating\n");
  gs_quit = 1;
  return;
}

#define BUFFSIZE 1024 * 8
/*
 * run_app()
 *   This function runs until the server quits and it:
 *
 *   1. reads from serial port and prints
 */
int run_app()
{

  char sbuf[BUFFSIZE];
  int numCharsRead = 0;
  int currentChar = 0;
  int startChar = 0;
  char data = '0';
  char buf[477]; //store all the data
  int dist[228]; //store the distance
  int dis_index = 0;
  int G_count = 0;  //number of thing received after 'G'
  int LF_count = 0; //number of consecutive line feed
  int get_dis_first; //if it is the first part of the two parts data
  int16_t tempscid;
  int16_t distance;
  int count = 0;
  char mychar;
  float ladar_dist_pong[228];
  float ladar_dist_ping[228];

  int to_xy = 0; //convert distance to xy
  int read_ladar_state = 0;         //states for ladar reading, initialized as wait for S command
  char G_command[] = "G04472503\n"; //command for getting distance -120 to 120 degree
  int G_len = 11;                   //length of command

  int pingpong = 0;
  int kb_state;
  // We don't wanna block
  sd_set_nonblocking();
  printf(".\n");
  sd_ioflush();

  struct shared_memory *shared_mem_ptr;
  sem_t *mutex_sem;
  int fd_shm;

  //  mutual exclusion semaphore, mutex_sem with an initial value 0.
  if ((mutex_sem = sem_open(SEM_MUTEX_NAME, O_CREAT, 0660, 0)) == SEM_FAILED)
    error("sem_open");
  // Get shared memory
  if ((fd_shm = shm_open(SHARED_MEM_NAME, O_RDWR | O_CREAT | O_EXCL, 0660)) == -1)
    error("shm_open");
  //set the size of the shared memory
  if (ftruncate(fd_shm, sizeof(struct shared_memory)) == -1)
    error("ftruncate");
  //map the memory to virtual address
  if ((shared_mem_ptr = mmap(NULL, sizeof(struct shared_memory), PROT_READ | PROT_WRITE, MAP_SHARED,
                             fd_shm, 0)) == MAP_FAILED)
    error("mmap");

  int ii;
  //initialize the shared memory
  for (ii = 0; ii < 228; ii++)
  {
    shared_mem_ptr->ladar_dist[ii] = 0;
  }

  //stop when ctrl+c or 'q'
  while (!gs_quit)
  {
    kb_state = _kbhit(); //check if hit keyboard
    if (kb_state == 1)
    {
      //read the entered character
      mychar = mygetch();
      if (mychar == 'q')
      { //exit if hit 'q'
        gs_quit = 1;
      }
    }
    ///////////////////////////////////////////////////////////////////////
    // Read from serial port and print everyt hing upto the return character
    if (currentChar >= numCharsRead) {
        numCharsRead = sd_readn(sbuf, BUFFSIZE);  //nonblocking
        startChar = 0;
    } else {
        startChar = currentChar;
    }
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
        if (read_ladar_state == 0)
        { //wait for the starting char 'G'
          if (data == 'G')
          {
            read_ladar_state = 1;
            G_count = 1;
          }
        }
        else if (read_ladar_state == 1)
        { //get the headers
          G_count += 1; //count the number of headers
          if (G_count >= 12)
          { //there are total 12 things before the data
            read_ladar_state = 2;
            get_dis_first = 1;
            dis_index = 0;
          }
        }
        else if (read_ladar_state == 2)
        { //get actual data
          if (data == 0x0a)
          { //check for line feed
            LF_count += 1;
          }
          else
          {
            LF_count = 0;
            if (get_dis_first == 1)
            {
              tempscid = (data - 0x30) << 6; //first data is the top 6 bit of the distance reading
              get_dis_first = 0;
            }
            else if (get_dis_first == 0)
            {
              distance = tempscid + (data - 0x30); //second data if the bottom 6 bit of the distance reading
              if (distance == 0 || distance == 6)
              {                               //error code
                distance = LADAR_MAX_READING; //set to the max reading
              }
              else if (distance == 16)
              { //distance is 4096mm
                distance = 4096;
              }
              if (pingpong == 0)
              {
                ladar_dist_ping[dis_index % 228] = distance;
              }
              else if (pingpong == 1)
              {
                ladar_dist_pong[dis_index % 228] = distance;
              }
              dis_index += 1;
              get_dis_first = 1;
            }
          }
          if (LF_count == 2)
          { //two consecutive line feed means the end of the current data stream
            count += 1;
            read_ladar_state = 0;
            to_xy = 1;
            pingpong = 1 - pingpong;
            printf("%.3f %.3f %.3f \n", ladar_dist_pong[0], ladar_dist_pong[1], ladar_dist_pong[2]);//print the first three distance

            if (count % 1 == 0)
            {
              for (ii = 0; ii < 228; ii++)
              {
                if (pingpong == 1)
                {
                  shared_mem_ptr->ladar_dist[ii] = ladar_dist_ping[ii];
                }
                else
                {
                  shared_mem_ptr->ladar_dist[ii] = ladar_dist_pong[ii];
                }
              }
              // Initialization complete; now we can set mutex semaphore as 1 to
              // indicate shared memory segment is available
              if (sem_post(mutex_sem) == -1)
                error("sem_post: new_dist_sem");
              printf("posted\n");

            }
            currentChar++;
            break;
          }
        }
      }
    }

    sched_yield();
  }
}

/*
 * setup_serial()
 *   sets the serial port up at 115200 baud
 */
int setup_serial()
{
  sd_setup(SERFILE); //starts non-blocking
  sd_ioflush();
}

/*
 * main()
 *   process command line input
 */
int main(int argc, char **argv)
{
  /* File pointer to hold reference to our file */
  printf("(C)2017 CHIP sample serial\n");
  /* 
     * Open file in w (write) mode. 
     * "data/file1.txt" is complete path to create file
     */

  printf("Setting signal handler...\n");
  signal(SIGKILL, gs_killapp);
  signal(SIGINT, gs_killapp);
  printf("...OK\n");

  printf("Initializing serial port driver %s...\n", SERFILE);
  setup_serial();
  printf("...OK\n");

  printf("Running...\n");
  run_app();
  shm_unlink(SHARED_MEM_NAME);
  sem_unlink(SEM_MUTEX_NAME);
  printf("removed shared mem...\n");
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