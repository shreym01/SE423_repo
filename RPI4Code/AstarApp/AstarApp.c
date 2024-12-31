/* 
Whenever it sees the semaphore for send is 1, get the data from shared memory and process A*
Send the path to send shm, and post the semaphore for read.
*/

#include <sys/ioctl.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <semaphore.h>
#include <sys/mman.h>
#include <stdio.h>	//for printf and scanf
#include <math.h>	//for abs
#include <termios.h>
#include <sys/signal.h>
#include <sys/time.h>
#include <asm/ioctls.h>
#include <linux/serial.h>

/*
functions implemented in pQueue.cpp 
priority queue functions:	--> necessary for open list and closed list
size()
peek()
push()
pop()
*/

/*
priority queue elements aka nodes properties
position (x and y position for this case)  called xPox, yPos
total distance (f), distance traveled from start (g), distance to goal(h) called totalDist, distTravelFromStart, distToGoal
--> defined in pQueue.h 
*/
#include "pQueue.h"

// Types and Variables for AstarApp COM  *************************
#define SENDTO_ASTARAPP_SEM_MUTEX_NAME "/sem-AstarApp-sendto-pose-obstacle"
#define SENDTO_ASTARAPP_SHARED_MEM_NAME "/sharedmem-AstarApp-sendto-pose-obstacle"
#define READFROM_ASTARAPP_SEM_MUTEX_NAME "/sem-AstarApp-readfrom-new-path"
#define READFROM_ASTARAPP_SHARED_MEM_NAME "/sharedmem-AstarApp-readfrom-new-path"
#define MIN(A,B)        (((A) < (B)) ? (A) : (B));

//structure for pose and obstacle from F28
/*
total length is 2*4 + 2*2 + 22*1 = 34 bytes
*/
typedef struct pose_obstacle{
    float x; //4
    float y;
	short destrow; //astar end point row
	short destcol; //astar end point col
	char mapCondensed[22];
} pose_obstacle;

//union of char and pose_obstacle for reading data
typedef union {
    char obs_data_char[34];
    pose_obstacle cur_obs;
} int_pose_union;


struct shared_memory_sendto_AstarApp
{
  int_pose_union new_pose;
};

struct shared_memory_readfrom_AstarApp
{
  char read_path[81];   // Path up to 80 long.  81st char is a status char.  char81 = [bit7=NA
  																						//bit6=NA
																					 	//bit5=NA
																						//bit4=NA
																						//bit3=start or stop are obstacles
																						//bit2=start or stop outside map
																						//bit1=1 start == endpoint
																						//bit0=1 Need to reset Map]
};

// Print system error and exit
void error (char *msg)
{
    perror (msg);
    exit (1);
}
//path data to send to F28, 10 points in the format of row1,col1,row2,col2...initialize to 199(large number that can never happen)
int sem_count_read = 0;
int_pose_union pose_obs_data;
char mychar;
static int gs_quit = 0;
char planned_path[80]={[0 ... 79] = 199}; //the path generated from A*
int first_time = 0;//first time enter the program, used to get the initial path


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

//global variables necessary for the algorithm
node_t neighbors[8];  //used to get a list of neighboring nodes only need 4 but made it 8 if you want to add corners later
double currDist = 0;	//distance traveled from starting point
int pathLen = 0;			//used to keep track of number of points in reconstructed path
int pathRow[400];			//reconstructed path in reverse order
int pathCol[400];
int endRow = 0; //row of destination
int endCol = 0; //col of destination
// int oldendRow = 0; //store the last endrow
// int oldendCol = 0; //store the last endcol
int retvalue = 0;
char returnstatus = 0;

char map[176] =      //16x11
{   '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
    'x', 'x', 'x', 'x', '0', '0', '0', 'x', 'x', 'x', 'x',
    '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'   };
char map_sol[176] =      //16x11
{   '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
    'x', 'x', 'x', 'x', '0', '0', '0', 'x', 'x', 'x', 'x',
    '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'   };
int mapRowSize = 16;
int mapColSize = 11;
dictElem_t nodeTrack[176];		//to track location of nodes in the heap, and to track parents of nodes technically only needs to be mapRowSize*mapColsize long
heap_t openSet, closedSet;

//preconditions: rowCurr, colCurr, rowGoal, and colGoal are feasible locations in the matrix
//returns the distance between two points as the sum of the row and column differences "Euclidean" distance on a square grid
double heuristic(int rowCurr, int colCurr, int rowGoal, int colGoal)
{
	int rowDiff = rowCurr - rowGoal;
	int colDiff = colCurr - colGoal;
	return sqrt(rowDiff * rowDiff + colDiff * colDiff);
}

//assumes that canTravel is called with neighboring points of a valid starting point
//precondition: row, col must be valid indices for a matrix
//returns 0 for false and 1 for true
//(row, col) is considered an unacceptable traveling point if it is represented as an 'x' in map, or if it is out of bounds of the map
int canTravel(int row, int col)
{
	//check for out of bounds
	int mapIdx = row*mapColSize + col;		//map data stored as 1d array
	if(mapIdx >= (mapRowSize*mapColSize) || mapIdx < 0)		//index out of bounds
		return 0;		//0 for cannot travel
	else if(col >= mapColSize || col < 0)			//too many columns, will result in a location in the next row
		return 0;
	else if(map[mapIdx] == 'x')	//wall reached, cannot travel
		return 0;
	else
		return 1;
}

//parameters rowCurr and colCurr must be valid starting points
//returns the number of neighbors which are viable traveling points
//fills the node_t array called neighbors, with neighboring nodes to which the path can travel
//note: not using diagonal neighbors here, so there are only for possibilities for neighboring nodes
int getNeighbors(int rowCurr, int colCurr)
{
	node_t nodeToAdd;
	int numNeighbors = 0;
	if(canTravel(rowCurr-1, colCurr) == 1)	//can travel up
	{
		nodeToAdd.row = rowCurr-1;
		nodeToAdd.col = colCurr;
		neighbors[numNeighbors] = nodeToAdd;
		numNeighbors++;
	}
	if(canTravel(rowCurr, colCurr-1) == 1)	//can travel left
	{
		nodeToAdd.row = rowCurr;
		nodeToAdd.col = colCurr-1;
		neighbors[numNeighbors] = nodeToAdd;
		numNeighbors++;
	}
	if(canTravel(rowCurr,colCurr+1) == 1)	//can travel right
	{
		nodeToAdd.row = rowCurr;
		nodeToAdd.col = colCurr+1;
		neighbors[numNeighbors] = nodeToAdd;
		numNeighbors++;
	}
	if(canTravel(rowCurr+1, colCurr) == 1)	//can travel down
	{
		nodeToAdd.row = rowCurr+1;
		nodeToAdd.col = colCurr;
		neighbors[numNeighbors] = nodeToAdd;
		numNeighbors++;
	}
	return numNeighbors;
}
 
//helper function called inside astar to return the path from the goal point back to the starting point
//the list of points is put into the array pathRow and pathCol in reverse order, pathLen is the number of inserted points
void reconstructPath(int rowEnd, int colEnd, dictElem_t nodeTrack[])
{
	pathLen = 0;		//global variable, reset so start inserting at beginning of path array
	int currRow = rowEnd;
	int currCol = colEnd;
	while(currRow != 400 && currCol != 400)
	{
		//while node is not null "400", put it into path starting at end point
		pathRow[pathLen] = currRow;  //global array for Row
		pathCol[pathLen] = currCol;  //global array for Column
		pathLen++;
	//	printf("currPath: (%d, %d), %d\n", pathRow[pathLen-1], pathCol[pathLen-1], pathLen);
		int nodeTrackIdx = currRow*mapColSize+currCol;
		currRow = nodeTrack[nodeTrackIdx].parentRow;
		currCol = nodeTrack[nodeTrackIdx].parentCol;
	//	printf("next location: (%d, %d), %d\n", currRow, currCol, pathLen);
	}
//	printf("done with reconstruction\n");
}

//path planning algorithm
//parameters rowStart, colStart, rowEnd, and colEnd must be valid locations
//they must be both within the indices of the matrix size and must not be points where barriers ('x') exist
int astar(int rowStart, int colStart, int rowEnd, int colEnd)
{
	//pseudo code instruction: initialize open and closed sets
	//initialize a dictionary to keep track of parents of nodes for easy reconstruction and for keeping track of 
	//  heap indexes for easy retrieval from heaps
	//set the values of the dictionary, open and closed sets to be zero 
	// so that no old values sitting in memory will produce weird results
	int resetNodeCnt;
	for(resetNodeCnt=0; resetNodeCnt<176; resetNodeCnt++)
	{
		nodeTrack[resetNodeCnt].heapId = ' ';
		nodeTrack[resetNodeCnt].heapIdx = 0;
		nodeTrack[resetNodeCnt].parentRow = 0;
		nodeTrack[resetNodeCnt].parentCol = 0;
	}

	startHeap(&openSet);
	startHeap(&closedSet);
	currDist = 0;
	node_t start;
	
	/* initialize a start node to be the starting position
		since this is the start node, it should have zero distance traveled from the start, the normal predicted distance to the goal,
		and the total distance is the sum of the distance traveled from the start and the distance to the goal.*/
	/* update the dictionary, use row and column location as a key in the dictionary, 
		use 400 for null parent values and don't forget to indicate which heap, open or closed set, the node is being placed*/
	/* put starting node on open set*/
	
	start.row = rowStart;
	start.col = colStart;
	start.distTravelFromStart = 0;
	start.distToGoal = heuristic(rowStart,colStart,rowEnd,colEnd);
	start.totalDist = start.distTravelFromStart+start.distToGoal;
	int startIdx = (start.row*mapColSize)+start.col;
	(nodeTrack[startIdx]).heapId = 'o';		//o for open set
	nodeTrack[startIdx].parentRow = 400;	//use 400 as NULL, if 400, know reached beginning in reconstruction
	nodeTrack[startIdx].parentCol = 400;	//no parent value = 400 since out of bounds
	if(rowStart == rowEnd && colStart == colEnd) {		//if start point is the end point, don't do anything more!!!
		printf("!!!!!!!! Exit on rowStart == rowEnd, Returning 2 !!!!!!\n");
		return 2;
	}
	push_(start, &openSet, nodeTrack, mapColSize); // put start node on the openSet

	char goalFound = 'f'; //false
	
	/*while open set not empty and goal not yet found*/
	while(openSet.numElems > 0 && goalFound != 't')	
	{

		/*find the node with the least total distance (f_score) on the open list, call it q (called minDistNode in code)*/
		node_t minDistNode = peek_(&openSet);	//find node with least total cost, make that the current node
		//printf("Current Node, Row=%d,Col=%d,g=%d,h=%d,f=%d\n",minDistNode.row,minDistNode.col,minDistNode.distTravelFromStart,minDistNode.distToGoal,minDistNode.totalDist);
		/*set the current distance to the current node's distance traveled from the start.*/
		//1.  currDist is set to q's distance traveled from the Start.  Explain why this could be different from the Manhattan distance to the Start position 
		//    This question is just asking for comments.
		currDist = minDistNode.distTravelFromStart;
		
		(nodeTrack[(minDistNode.row*mapColSize)+minDistNode.col]).heapId = 'r';		//r for removed from any set
		
		//2.  pop q (which is currently the minimum) off which queue? 
		// Choose one of these two lines of code
		// IF the Openset
		//pop_(&openSet, nodeTrack, mapColSize);
		// IF closedSet 
		//pop(&closedSet, nodeTrack, mapColSize);
		
		/*generate q's 4 neighbors*/
		// 3.  Pass q's row and col to getNeighbors
		int numNeighbors = getNeighbors(r?, c?);	//get list of neighbors
	
		/*for each neighbor*/
		int cnt = 0;
		for(cnt = 0; cnt<numNeighbors; cnt++)	//for each found neighbor
		{
			// 4. Just add comments here.  Find where the structure node_t is defined and inside commments here copy its definition for
			// viewing reference.  
			// All the answer for 4. will be commented.
			node_t next = neighbors[cnt];
			
			/*if neighbor is the goal, stop the search*/
				/*update the dictionary so this last node's parents are set to the current node*/
			if((next.row == rowEnd) && (next.col == colEnd))		//if neighbor is the goal, found the end, so stop the search
			{
				// 5.  set current neighbor's parents.  Set parentRow to q's row.  Set parentCol to q's col since q is the parent of this neighbor
				(nodeTrack[next.row*mapColSize+next.col]).parentRow = ?;	 //set goal node's parent position to current position
				(nodeTrack[next.row*mapColSize+next.col]).parentCol = ?;
				goalFound = 't';
				break;
			}
			
			/*neighbor.distTravelFromStart (g) = q.distTravelFromStart + distance between neighbor and q which is always 1 when search just top left bottom right*/
			// 6.  Set this neighbor's distance traveled from the start.  Remember you have the variable "currDist" that is the distance of q to Start
			//need to consider diagonal distance now
			next.distTravelFromStart = ?;
			// printf("next.distTravelFromStart %f\n", next.distTravelFromStart);
			
			/*neighbor.distToGoal (h) = distance from goal to neighbor, heuristic function	(estimated distance to goal)*/
			// 7.  Pass the correct parameters to "heuristic" to calculate the distance this neighbor is from the goal.
			//  Remember that we have the variables rowEnd and colEnd which are the grid coordinates of the goal 
			next.distToGoal = heuristic(?, ?, ?, ?);
			
			/*neighbor.totalDist (f) = neighbor.distTravelFromStart + neighbor.distToGoal
				(total estimated distance as sum of distance traveled from start and distance to goal)*/
			// 8.  Find f, (totalDist) for this neighbor
			next.totalDist = ?;
			
			
			// 9.  Just comments for this question.
			// Explain in your own words (not copying the comments below) what the next 19 lines of C code are doing
			
			/*if a node with the same position as neighbor is in the OPEN list
				which has a lower total distance than neighbor, skip putting this neighbor onto the open set*/
			//check if node is on the open set already
			int nodeTrackIdx = (next.row*mapColSize)+next.col;
			char skipPush = 'f';
			if(nodeTrack[nodeTrackIdx].heapId == 'o')	//on the open set
			{
				int heapIndex = nodeTrack[nodeTrackIdx].heapIdx;
				node_t fromOpen = openSet.elems[heapIndex];
				if(fromOpen.totalDist <= next.totalDist)
					skipPush = 't';		//skip putting this node onto the openset, a better one already on there
			}
			
			/*if a node with the same position as neighbor is in the CLOSED list
				which has a lower f than neighbor, skip putting this neighbor onto the open set*/
			else if(nodeTrack[nodeTrackIdx].heapId == 'c')		//on closed set
			{
				int heapIndex = nodeTrack[nodeTrackIdx].heapIdx;
				node_t fromClosed = closedSet.elems[heapIndex];
				if(fromClosed.totalDist <= next.totalDist)
					skipPush = 't';		//skip putting this node onto the openset, already part of possible solution
			}
			
			/*if not skipping putting this node on the set, then push node onto the open set
				and update the dictionary to indicate the node is on the open set and set the parents of this node to the current node*/
				//(can't be an ordinary else to the things above because then it won't push onto the open set if already on open or closed set)
			if(skipPush != 't')
			{
				nodeTrack[nodeTrackIdx].heapId = 'o';		//mark in nodetrack that this is going onto the open set
				(nodeTrack[nodeTrackIdx]).parentRow = minDistNode.row;	 //set neighbor's parent position to current position
				(nodeTrack[nodeTrackIdx]).parentCol = minDistNode.col;
				//10.  push this neighbor on which queue? 
				// Choose one of these two lines of code
				// IF openSet
				//push_(next, &openSet, nodeTrack, mapColSize);
				//printf("Push to OpenList, Row=%d,Col=%d,g=%d,h=%d,f=%d\n",next.row,next.col,next.distTravelFromStart,next.distToGoal,next.totalDist);
				// IF closedSet
				//push(next, &closedSet, nodeTrack, mapColSize);
				
			}
		}/* end for loop*/
		
		int nodeTrackIdx = minDistNode.row*mapColSize+minDistNode.col;
		nodeTrack[nodeTrackIdx].heapId = 'c';
		//11.  push q (current node) on which queue? 
		// Choose one of these two lines of code
		// IF openSet
		//push(minDistNode, &openSet, nodeTrack, mapColSize);
		// IF closedSet
		//push_(minDistNode, &closedSet, nodeTrack, mapColSize);
		//printf("Push to ClosedList, Row=%d,Col=%d\n",minDistNode.row,minDistNode.col);
	}  /*end while loop*/

	/*if a path was found from the start to the goal, then reconstruct the path*/
	if(goalFound == 't') {
		// 12.  Pass the correct varaibles to "reconstructPath" in order for it to fill in the global arrays pathRow, pathCol
		//     and integer pathLen.  Note that the path is in reverse order in pathRow and pathCol.
		reconstructPath(?, ?, ?);
		return 1;
	}
	return 0;
}

int main() {
    struct shared_memory_sendto_AstarApp *send_shared_mem_ptr;
    struct shared_memory_readfrom_AstarApp *read_shared_mem_ptr;
    sem_t *send_mutex_sem;
    sem_t *read_mutex_sem;
    int send_fd_shm;
    int read_fd_shm;
    int i,j = 0;//for loop index
    int entered_time;//dummy variable for path
	int temp_obs_row = 0;
	int temp_obs_col = 0;
	int robot_row = 0;
	int robot_col = 0;
	int re_astar = 0;
    //create the semaphore for send 
    if ((send_mutex_sem = sem_open(SENDTO_ASTARAPP_SEM_MUTEX_NAME, 0, 0, 0)) == SEM_FAILED)
        error("send sem_open");
    //create the semaphore for read path 
    if ((read_mutex_sem = sem_open(READFROM_ASTARAPP_SEM_MUTEX_NAME, 0, 0, 0)) == SEM_FAILED)
        error("read sem_open");

    // create shared memory for send
    if ((send_fd_shm = shm_open(SENDTO_ASTARAPP_SHARED_MEM_NAME, O_RDWR, 0)) == -1)
        error("shm_open");

    //map the memory to virtual address
    if ((send_shared_mem_ptr = mmap(NULL, sizeof(struct shared_memory_sendto_AstarApp), PROT_READ | PROT_WRITE, MAP_SHARED,
                                send_fd_shm, 0)) == MAP_FAILED)
        error("mmap");

    // create shared memory for read
    if ((read_fd_shm = shm_open(READFROM_ASTARAPP_SHARED_MEM_NAME, O_RDWR, 0)) == -1)
        error("shm_open");

    //map the memory to virtual address
    if ((read_shared_mem_ptr = mmap(NULL, sizeof(struct shared_memory_readfrom_AstarApp), PROT_READ | PROT_WRITE, MAP_SHARED,
                                read_fd_shm, 0)) == MAP_FAILED)
        error("mmap");
    while (!gs_quit) {
        int kb_state = _kbhit(); //check if hit keyboard
        if (kb_state == 1)
            {
            //read the entered character
            mychar = mygetch();
            if (mychar == 'q')
            { //exit if hit 'q'
                gs_quit = 1;
            }
        }

        //check if the semaphore has been posted, got new data
        if (sem_trywait(send_mutex_sem) == 0) {
            for (i = 0; i < 34; i++){
                pose_obs_data.obs_data_char[i] = send_shared_mem_ptr->new_pose.obs_data_char[i]; //get the pose from shm
            }
			
			for(i = 0; i<176; i++) {  // Intialize the Map to all '0'  176 = mapRowSize*mapColSize
				map[i] = '0';
			}
			int current_char = 0;
			for(i = 0; i<176; i++) {
				if ((pose_obs_data.cur_obs.mapCondensed[current_char]&0x1)==0x1){
					map[i] = 'x';
				}
				i++;
				if ((pose_obs_data.cur_obs.mapCondensed[current_char]&0x2)==0x2){
					map[i] = 'x';
				}
				i++;
				if ((pose_obs_data.cur_obs.mapCondensed[current_char]&0x4)==0x4){
					map[i] = 'x';
				}
				i++;
				if ((pose_obs_data.cur_obs.mapCondensed[current_char]&0x8)==0x8){
					map[i] = 'x';
				}
				i++;
				if ((pose_obs_data.cur_obs.mapCondensed[current_char]&0x10)==0x10){
					map[i] = 'x';
				}
				i++;
				if ((pose_obs_data.cur_obs.mapCondensed[current_char]&0x20)==0x20){
					map[i] = 'x';
				}
				i++;
				if ((pose_obs_data.cur_obs.mapCondensed[current_char]&0x40)==0x40){
					map[i] = 'x';
				}
				i++;
				if ((pose_obs_data.cur_obs.mapCondensed[current_char]&0x80)==0x80){
					map[i] = 'x';
				}
				current_char++;
			}

			//update the destination of the robot
			endRow = pose_obs_data.cur_obs.destrow;
			endCol = pose_obs_data.cur_obs.destcol;
			robot_col = round(pose_obs_data.cur_obs.x) +5; //get the robot pose with round as the starting point 
			robot_row = 11- round(pose_obs_data.cur_obs.y); 
			returnstatus = 0;
			if (endRow == robot_row && endCol == robot_col) {
				re_astar = 0;
				returnstatus |= 0x2;  //set status that end == start
				printf("A* Start Point Equals A* Stop Point: No A* Run\n");
			} else {
 				re_astar = 1;
			}
			printf("pose %.3f, %.3f", pose_obs_data.cur_obs.x, pose_obs_data.cur_obs.y);
			printf("\n");
            //##################### put path planning algorithm here ############################
		
			if (re_astar == 1) {
				re_astar = 0; //set back to 0

				//print map, this is the map with obstacles as x
				for(i=0; i<mapRowSize; i++){
				 	for(j = 0; j<mapColSize; j++) {
				 		printf("%c ", map[i*mapColSize+j]);
					}
				 	printf("\n");
				}
				printf("\n");
   
				printf("doing A*");
				if(robot_row>=0 && robot_row<mapRowSize && robot_col>=0 && robot_col<mapColSize && endRow>=0 && endRow<mapRowSize && endCol>=0 && endCol<mapColSize) //if in bounds
				{
					printf("here");
					if(map[robot_row*mapColSize+robot_col]!='x' && map[endRow*mapColSize+endCol]!='x')        //make sure valid start and end points
					{
						retvalue = astar(robot_row,robot_col,endRow,endCol);
					 	printf("A* starts at %d %d ends at %d %d \n", robot_row, robot_col, endRow, endCol); //print starting point of A*
						if (retvalue == 1) { //if found a path
							/* Fill in the points array for the robot to visit in reverse order of A* result
							* x = col - 5
							* y = 11 - row
							*/
							int numpts;//number of points that needed to fill in
							numpts = MIN(pathLen,40); //find the minimum of the path length and 40(the max path length possible)							
							//since the points array is reused every A*, need to reset every time
							for (i = 0; i < 40; i++) {
								// points[i].x = 0;
								// points[i].y = 0;
								planned_path[2*i] = 199; //clear the sending to f28 array
								planned_path[2*i+1] = 199;
							}
							for (i = 0; i < numpts; i++){
								planned_path[i*2] = pathRow[i];
								planned_path[i*2+1] = pathCol[i];
							}
						} else {
							returnstatus |= 0x1;  //Tell F28379D to reset the map
							printf("No Path Found, Which is impossible so Resetting Map to Default\n");
							planned_path[0] = 99;
						}
					} else {
						returnstatus |= 0x9;  // start or stop are obstacles and reset map
						printf("A* Start or Stop are obstacles\n");
						planned_path[0] = 99;	
					}
				} else {
					returnstatus |= 0x4;  // start or stop outside of map
					printf("A* Start or Stop are outside of map coordinates\n");
					planned_path[0] = 99;
				}
				/*
				print the solution
				*/
				for (i = 0; i < 176; i++){
					map_sol[i]= map[i];
				}
				//put the solution on map only when there's no error
				if (planned_path[0] != 99) {
					for(i = 0; i< pathLen; i++) {	//put solution on map
						int mapIdx = pathRow[i]*mapColSize + pathCol[i];
						// printf("pathRow is %d, pathCol is %d\n", pathRow[i], pathCol[i]);
						planned_path[i*2] = pathRow[i];
						planned_path[i*2+1] = pathCol[i];
						map_sol[mapIdx] = '-';
					}
				}
				//print map with solution
				for(i=0; i<mapRowSize; i++) {
					for(j = 0; j<mapColSize; j++) {
						printf("%c ", map_sol[i*mapColSize+j]);
					}
					printf("\n");
				}
				printf("\n");
				
				//###################################################################################
				for (i = 0; i < 80; i++) {
					read_shared_mem_ptr->read_path[i] = 199; //clear the path(set to 199 as no path code)
				}
				for (i = 0; i < pathLen*2; i++) {
					read_shared_mem_ptr->read_path[i] = planned_path[i]; // fill in the path
					// printf("send is %d\n",read_shared_mem_ptr->read_path[i]);
				}
				read_shared_mem_ptr->read_path[80] = returnstatus;
				//check if the previous data has been processed
				if (sem_getvalue(read_mutex_sem,  &sem_count_read) == 0) {
					if (sem_post(read_mutex_sem) == -1){
						error("sem_post: read mutex");
					}
				}
			} else {
				for (i = 0; i < 80; i++) {
					read_shared_mem_ptr->read_path[i] = 199; //clear the path(set to 199 as no path code)
				}
				read_shared_mem_ptr->read_path[80] = returnstatus;
				//check if the previous data has been processed
				if (sem_getvalue(read_mutex_sem,  &sem_count_read) == 0) {
					if (sem_post(read_mutex_sem) == -1){
						error("sem_post: read mutex");
					}
				}
			}

		}
        
    }
}
