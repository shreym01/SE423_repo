/*
 *
 *       client.c: Write strings for printing in POSIX shared memory object
 *                 
 */

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

// Buffer data structures
#define MAX_BUFFERS 10

#define SEM_MUTEX_NAME "/sem-new-ladar-dist"
#define SHARED_MEM_NAME "/posix-shared-mem-ladar-dist"

struct shared_memory {
  float ladar_dist[228];
};

void error (char *msg);
float client_dist[228];
int i;
int main (int argc, char **argv)
{
    struct shared_memory *shared_mem_ptr;
    sem_t *mutex_sem;
    int fd_shm;
    
    //  mutual exclusion semaphore, mutex_sem 
    if ((mutex_sem = sem_open (SEM_MUTEX_NAME, 0, 0, 0)) == SEM_FAILED)
        error ("sem_open");
    
    // Get shared memory 
    if ((fd_shm = shm_open (SHARED_MEM_NAME, O_RDWR, 0)) == -1)
        error ("shm_open");

    if ((shared_mem_ptr = mmap (NULL, sizeof (struct shared_memory), PROT_READ | PROT_WRITE, MAP_SHARED,
            fd_shm, 0)) == MAP_FAILED)
       error ("mmap");

    
    char *cp;
    printf ("Waiting for data ");
    int count = 0;
    while (1) {    
        /* There might be multiple producers. We must ensure that 
            only one producer uses buffer_index at a time.  */
        // P (mutex_sem);
        if (sem_wait (mutex_sem) == -1)
            error ("sem_wait: mutex_sem");

	    // Critical section
        time_t now = time (NULL);
        cp = ctime (&now);
        int len = strlen (cp);
        if (*(cp + len -1) == '\n')
            *(cp + len -1) = '\0';
        for (i = 0; i < 228; i++){
            client_dist[i] = shared_mem_ptr->ladar_dist[i];
        }
        printf ("%d: %s \n", getpid (), cp);
        for (i = 0; i < 57; i++){
            printf("%.3f %.3f %.3f %.3f \n", client_dist[4*i],client_dist[4*i+1],client_dist[4*i+2],client_dist[4*i+3]);
        }
        printf ("Please type a message: ");
        count++;
    }
 
    if (munmap (shared_mem_ptr, sizeof (struct shared_memory)) == -1)
        error ("munmap");
    exit (0);
}

// Print system error and exit
void error (char *msg)
{
    perror (msg);
    exit (1);
}