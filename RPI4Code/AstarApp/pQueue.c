/*
Tori Fujinami
min priority queue implemented as a heap
sorts based on total distance for solving path planning problems
*/
#include "pQueue.h" 
//variables: numElems, elems ->names of members of struct heap_t

//pQueue helper functions:

int pQueueoutofMemory = 0;
//returns the index of the root of the heap, no parameters
int root()
{
    return 0;       //root begins at 0, 0 based heap
}

//returns the index of the left child of a node in the heap
//parameter idx is the current node index
int leftChild(int idx)
{
    return 2*idx+1;     //left child in the heap is at array index 2* the current index + 1
}

//returns the index of the right child of a node in the heap
//parameter idx is the current node index
int rightChild(int idx)
{
    return 2*idx+2; //right child is one past left child
}

//returns the index of the parent child of a node in the heap
//parameter idx is the current node
int parent(int idx)
{
    return (idx-1)/2;   //returns the truncated integer (floor function) of 1/2 the index-1
}

//returns 0 if this node has no children, 1 if there is at least one child
//parameter idx is the current node index, pq is the priority queue to be checked
int hasChild(int idx, heap_t *pq)
{
    if((2*idx+1) >= pq->numElems)
        return 0;
    //since heap is a complete tree, if no left child, no right one either,
        //so only need to check if a left child exists
    else
        return 1;
}

//returns the index of the higher priority child
//parameter idx is the current node index, pq is the priority queue to be checked
int maxPriorityChild(int idx, heap_t* pq)
{
    //the higher priority child is the one with the smallest total distance

    //assumes that the current node already has children! (so before calling this function, make sure this is true)
    int idxL = leftChild(idx);
    int idxR = rightChild(idx);

    if((pq->elems[idxL]).totalDist < (pq->elems[idxR]).totalDist)
        return idxL;    //left child has higher priority than right child because it has smaller total distance
    else
        return idxR;    //right child has higher priority than left child
}

//priority queue user functions:
//set numElems to 0 so that no heap technically cleared, use when declare heap variable
void startHeap(heap_t* pq)
{
    pq->numElems = 0;
}

//returns the number of elements on the heap
int size(heap_t* pq)
{
    return pq->numElems;
}

node_t peek_(heap_t* pq)      //so that can look at top of heap without removing
{
    return pq->elems[0];     //return the top element of the heap, first element of the array
                            //does not remove the element
}

void push_(node_t newElem, heap_t* pq, dictElem_t nodeTrack[], int mapSize)      //pushes element onto the heap
{
    if (pq->numElems < 399) {
        newElem.pushOrder = pq->numElems;       //this is the numElem-th element pushed onto the array
        pq->elems[pq->numElems] = newElem;      //put element at back of the array
        int nodeTrackIdx = newElem.row*mapSize+newElem.col;
        nodeTrack[nodeTrackIdx].heapIdx = pq->numElems; //add index for swapping
        pq->numElems++;     //adjust heap size to account for added element

        int idx = pq->numElems-1;
        while (1) {
            if(idx == root()) {
                break;
            }
            int parentIdx = parent(idx);
            if((pq->elems[idx]).totalDist < (pq->elems[parentIdx]).totalDist)
            {
                //current idx has smaller total distance aka higher priority, needs to be moved up in the heap
                //start with swapping parent and child
                node_t temp = pq->elems[idx];
                pq->elems[idx] = pq->elems[parentIdx];
                pq->elems[parentIdx] = temp;
                //update heapIdx in the dictionary for current node and parent
                int nodeTrackIdx = ((pq->elems[idx]).row*mapSize)+(pq->elems[idx]).col;
                nodeTrack[nodeTrackIdx].heapIdx = idx;
                nodeTrackIdx = ((pq->elems[parentIdx]).row*mapSize)+(pq->elems[parentIdx]).col;
                nodeTrack[nodeTrackIdx].heapIdx = parentIdx;
                idx = parentIdx;
            } else {
                break;
            }
        }

    } else {
        pQueueoutofMemory = 1;
    }
}

void pop_(heap_t* pq, dictElem_t nodeTrack[], int mapSize)       //removes the minimum element does not return it!!
{
    if(pq->numElems>0)      //so that don't try to pop off non existent elements
    {
        node_t temp = pq->elems[0]; //front element
        pq->elems[0] = pq->elems[pq->numElems-1];   //swap front element with back element
        pq->elems[pq->numElems-1] = temp;   //highest priority element at back for easy removal
        int nodeTrackIdx = (pq->elems[0]).row*mapSize + (pq->elems[0]).col;
        nodeTrack[nodeTrackIdx].heapIdx = 0;
        pq->numElems--; //decrement number of elements in the array, so last element marker
                        //points to node just before the removed one, aka the actual removal
        int idx = 0;
        while(1) {
            if(hasChild(idx, pq) == 1) //has a child
            {
                int childIdx;
                if(rightChild(idx)>=pq->numElems)   //if no right child, only a left child
                    childIdx = leftChild(idx);
                else
                    childIdx = maxPriorityChild(idx, pq);  //set to child with smallest distance

                if((pq->elems[childIdx]).totalDist < (pq->elems[idx]).totalDist)
                {
                    //if child has a smaller total distance aka higher priority than the parent,
                        //swap child and parent and continue sorting until child in correct spot
                    node_t temp = pq->elems[idx];
                    pq->elems[idx] = pq->elems[childIdx];
                    pq->elems[childIdx] = temp;
                    //update heapIdx in the dictionary for current node and child
                    int nodeTrackIdx = ((pq->elems[idx]).row*mapSize)+(pq->elems[idx]).col;
                    nodeTrack[nodeTrackIdx].heapIdx = idx;
                    nodeTrackIdx = ((pq->elems[childIdx]).row*mapSize)+(pq->elems[childIdx]).col;
                    nodeTrack[nodeTrackIdx].heapIdx = childIdx;
                    idx = childIdx;
                } else {
                    break;
                }
            } else {
                break;
            }
        }
    }
}
