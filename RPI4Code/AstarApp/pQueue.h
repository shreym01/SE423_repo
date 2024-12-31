/*
Tori Fujinami
min priority queue implemented as a heap for astar open and closed set usage
sorts based on totalDist
use an array for node storage (preferably resizeable one?-->max size for astar for robot cars = row*col)
header file 
*/

#ifndef PQUEUE_H_
#define PQUEUE_H_

typedef struct node
{
    int row;
    int col;
    double distTravelFromStart;
    double distToGoal;
    double totalDist;
    int pushOrder;      //order that element was pushed onto the heap
} node_t;

typedef struct dictElem
{
    int parentRow;
    int parentCol;
    int heapIdx;
    char heapId;
} dictElem_t;

typedef struct heap
{
    int numElems;   //initially no elements on the array
    node_t elems[400];  //array of size 400, max number of elements for ge 423 robot contest that can be on a heap
} heap_t;

void startHeap(heap_t* pq); //use for initialization so that declared heap has no elements
int size(heap_t* pq);
node_t peek_(heap_t* pq);     //so that can look at top of heap without removing
void push_(node_t newElem, heap_t* pq, dictElem_t nodeTrack[], int mapSize);     //pushes element onto the heap
void pop_(heap_t* pq, dictElem_t nodeTrack[], int mapSize);

//pQueue helper functions:
int root();     //returns the index of the root of the heap
int leftChild(int idx);     //returns the index of the left child of a node in the heap
int rightChild(int idx);    //returns the index of the right child of a node in the heap
int parent(int idx);        //returns the index of the parent child of a node in the heap
int hasChild(int idx, heap_t* pq);       //returns 1 if the node at this index has at least one child, 0 if there are no children
int maxPriorityChild(int idx, heap_t *pq);   //returns the index of the right priority child

#endif
