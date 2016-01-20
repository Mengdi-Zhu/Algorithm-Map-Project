/*NAME AND DATE GOES HERE.*/
/*Brandeis Map*/

/*Standard system stuff - these are the ONLY ones that may be used.*/
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

/*Path to the map folder; may need to change this.*/
#include "MapPATH.h"

/*MAY NOT be modified, but should be viewed.*/
#include "Map.h" /*Map data parameters, structures, and functions.*/

/*MAY NOT be modified, and there is no need view them:*/
#include "MapData.h"   /*Functions to input the map data.*/
#include "MapInput.h"  /*Functions to get user input.*/
#include "MapOutput.h" /*Functions to produce output.*/

/*Use this to get the time to travel along an edge.*/
#define EdgeCost(X) ( (TimeFlag) ? Time(X) : Elength[X] )

/*This is used to define heap's structure and we define left child, right child and parent.*/
#define LCHILD(x) (2 * (x)) /*left child*/
#define RCHILD(x) (2 * (x) + 1) /*right child*/
#define PARENT(x) (((x)) / 2) /*parent*/


/*Use this to print a leg of a route or tour.*/
void PrintLeg(int edge);



/***************************************************************************************/
/*GRAPH ADJACENCY LIST DATA STRUCTURE                                                  */
/***************************************************************************************/

/*This is the structure to denote an adjacency list node which contains the information of edge.*/
struct NodeOfEdge
{
    /*This NodeNumber represents the number of each edge.*/
    int NodeNumber;
    /*This next object denotes the next edge node from this current one.*/
    struct NodeOfEdge* next;
};

/*This is the structure to denote an adjacency list which contains the head of each sublist.*/
struct NodeList
{
    /*This denotes the pointer to each head node of list.*/
    struct NodeOfEdge *head;
    /*This denotes the pointer to previous vertex to the original vertex on the shortest path.*/
	struct NodeOfEdge* pre;
	/*This denotes a flag (or a mark) to check if the vertex has been visited.*/
	int Visited;
	/*This denotes the distance from one vertex to the original vertex.*/
	int Distance;
	int Parent;
};

/*This is the structure to represent a graph and the graph is an array of adjacency lists.*/
/*Thus, we construct this graph structure.*/
struct Graph
{
    /*Size of array will be "VSize" which means number of vertex in graph.*/
    int VSize;
    /*The graph is an array of lists.*/
    struct NodeList* NodeArray;
};

/*This denotes the procedure to create one new adjacency list node.*/
struct NodeOfEdge* NewEdgeNode()
{
    /*This is to allocate space for this new node.*/
    struct NodeOfEdge* NewNode = (struct NodeOfEdge*) malloc (sizeof(struct NodeOfEdge));
    /*Initialize the node number to be zero.*/
    NewNode->NodeNumber = 0;
    /*Initialize the node's next to be null.*/
    NewNode->next = NULL;
    /*Return the NewNode.*/
    return NewNode;
}

/*This is the procedure to create a graph of VSize vertex.*/
struct Graph* creNewGraph(int VSize)
{
	int i;
	/*This is to allocate new space for graph structure.*/
    struct Graph* graph = (struct Graph*) malloc (sizeof(struct Graph));
    /*This is to set up graph's size for one graph.*/
    graph->VSize = VSize;
    /*We will create an array of adjacency lists with size of the array is VSize.*/
    graph->NodeArray = (struct NodeList*) malloc (VSize * sizeof(struct NodeList));
    /*We will initialize each adjacency list to be empty by making the head NULL.*/
    for (i = 0; i < VSize; ++i)
        graph->NodeArray[i].head = NULL;
    /*Return the graph.*/
    return graph;
}

/*This is to add a new edge to a directed graph.*/
void NewEdge(struct Graph* graph, int original, int finish,int NodeNumber)
{
    /*This is to add an edge from original vertex to final vertex.*/
	/*The new node is added to the adjacency list of original vertex.*/

	/*This is to create a new node.*/
    struct NodeOfEdge* NewNode = NewEdgeNode();
    /*Set up the node number.*/
	NewNode->NodeNumber = NodeNumber;
	/*Set up for new head.*/
    NewNode->next = graph->NodeArray[original].head;
    /* Initialize the head of the array to be a new node.*/
    graph->NodeArray[original].head = NewNode;
}

/*This function is used to add vertex into graph.*/
struct Graph* add() {
    /*Create a new graph with max number of vertex.*/
	struct Graph *graph = creNewGraph(MaxVertex);
	/* Set a temp variable. */
	int i;
    /* Set the index to use. */
	int number;
	for(i = 0; i < (sizeof(Eindex)) / (sizeof(Eindex[0])); i++) {
		number = i;
		/*source and destination point*/
		NewEdge(graph, Estart[number], Eend[number], number);
	}
	/*Return the graph we build up. */
	return graph;
}

/***************************************************************************************/
/*HEAP DATA STRUCTURE                                                                  */
/***************************************************************************************/

/* This is the structure to denote heap node.*/
typedef struct HeapNode
{
    /* This represents index number of vertex (HeapNode).*/
    int index;
    /* This will be implement in the third part.*/
    int distance;
}heapnode;

/* This is the structure to represent heap and contains information of data.*/
typedef struct Heap
{
	int number;
	/* This is to represent the node of a new array and we will create this heap.*/
	struct HeapNode* NodeOfArray;
}heap;

/* This function is used to create initialize this new array.*/
heap createArray(int size)
{
    /* define the heap structure.*/
	heap hp;
	/*This is to set up parameter of number.*/
	hp.number = 0;
	/*Allocate space for new array.*/
	hp.NodeOfArray = malloc (sizeof(struct HeapNode) * size);
	/* Return the heap we build. */
	return hp;
}

/* This function is to represent the insertion operation.*/
void insert(heap* hp, int index, int distance) {
    /* Create one new heap node.*/
	heapnode node ;
    /* number plus one.*/
    int i = (hp->number)++ ;
	/* Allocate space.*/
    if(hp->number)
		{hp->NodeOfArray = realloc(hp->NodeOfArray, (hp->number + 1) * sizeof(struct HeapNode)) ;}
		/* if hp->number is not 0.*/
	else
        {hp->NodeOfArray = malloc(sizeof(struct HeapNode)) ;}
        /* Otherwise.*/

    /* Set up the initial value for new node.*/
    /* Set up distance.*/
    node.distance = distance ;
    /* Set up index, initialize it.*/
	node.index = index ;

	/* Locate the node at the correct position in the min heap.*/
    while((i) && (node.distance < hp->NodeOfArray[PARENT(i)].distance)) {
        hp->NodeOfArray[i] = hp->NodeOfArray[PARENT(i)] ;
        /* Use definition of parent.*/
        i = PARENT(i) ;
    }
    hp->NodeOfArray[i] = node ;
}

/* This function is used to swap value of two objects.*/
void swap(struct HeapNode *n1, struct HeapNode *n2) {
    /* Create a new temp variable.*/
    struct HeapNode temp = *n1 ;
    *n1 = *n2 ;
    *n2 = temp ;
}

/*This function is a special function called modify() function recursively to ensure that heap property is fine.*/
void modify(heap *hp, int i) {
    /* Clarify the smallest one.*/
    int minimum = (LCHILD(i) < hp->number
                   && hp->NodeOfArray[LCHILD(i)].distance < hp->NodeOfArray[i].distance) ? LCHILD(i) : i ;

    if(RCHILD(i) < hp->number && hp->NodeOfArray[RCHILD(i)].distance < hp->NodeOfArray[minimum].distance) {
        /* Recursively call itself.*/
        minimum = RCHILD(i) ;
    }
    /*If the minimum value not equals to i.*/
    if(minimum != i) {
        /* Swap the value to be the smallest one. */
        swap(&(hp->NodeOfArray[i]), &(hp->NodeOfArray[minimum])) ;
        /* Recursively call itself.*/
        modify(hp, minimum) ;
    }
}

/*/ This function is used to delete node of the min heap.*/
int deleteNode(heap *hp) {
    /* This is the index of the vertex.*/
	int i;
	/* Clarify a new node.*/
    heapnode* node ;
    /* Start with the new one.*/
	node = &hp->NodeOfArray[0] ;
	i = node->index;
	hp->NodeOfArray[0] = hp->NodeOfArray[--(hp->number)] ;
	/* Call modify function to make sure that the heap property is not violated.*/
	modify(hp, 0) ;
	/* Return the index. */
	return i ;
}

/***************************************************************************************/
/*Dijkstra Algorithm                                                                   */
/*DijkstraFlag=1 to supress output when Dijkstra is called from tour code.)            */
/***************************************************************************************/

void Dijkstra(int DijkstraFlag) {
	/* We shall initialize some parameters and initialize them.*/
	/* Create a new graph using add function.*/
	struct Graph * graph = add();
	/*Create the new heap using createArray function.*/
	struct Heap hp = createArray(MaxVertex);

	int i;

    /* the index of the vertex.*/
	int vertexNumber;
	/* the index of the edge.*/
	int edgeNumber;
	/* Create a new edge.*/
	struct NodeOfEdge *edge;
	/* Create a new node.*/
	struct NodeList *node;
	/* Create a new node that is adjacent.*/
	struct NodeList *adjacent;
	/* Calculate the new cost.*/
	int cost;

	for(i = 0; i < graph->VSize; i++){
        /* Initialize the distance to be infinite.*/
		graph->NodeArray[i].Distance = 1e9;
        /* Set up the flag to check if it has been visited.*/
		graph->NodeArray[i].Visited = 0;
	}
	/* Set up the first node's distance to be zero.*/
	graph->NodeArray[Begin].Distance = 0;

	/* We will add vertex into the heap using insert function.*/
	insert(&hp, Begin, graph->NodeArray[Begin].Distance);

	/* Pop the minimum vertex and if it's not visited, pop it and choose a new one.*/
	while(hp.number > 0){
		/* Delete the first one.*/
		vertexNumber = deleteNode(&hp);
		node = &graph->NodeArray[vertexNumber];
		/* Check if it has been visited or marked.*/
		if(node->Visited != 1){
            /* Mark it.*/
		 	node->Visited = 1;

			edge = node->head;
			/* If the edge is not null.*/
			while(edge != NULL){
                /* Set up edgeNumber and define it.*/
			 	edgeNumber = edge->NodeNumber;
                /* Create and set up value for the adjacent node.*/
				adjacent = &(graph->NodeArray[Eend[edgeNumber]]);
				/* Calculate the new value of the node's distance.*/
				cost = node->Distance + EdgeCost(edgeNumber);
				/* Check if the adjacent's distance is more than the cost.*/
				if(adjacent->Distance > cost){
                    /* Use NodeList's attribute: Parent. */
				 	adjacent->Parent = edgeNumber;
					adjacent->Distance = cost;
                    /* Insert node to the heap. */
					insert(&hp, Eend[edgeNumber], cost);
				}
				edge = edge->next;
			}
		}
	}

	int ptr = Finish, stack[MaxEdge], stackp = 0;
	while(ptr != Begin){
		stack[stackp++] = graph->NodeArray[ptr].Parent;
		ptr = Estart[graph->NodeArray[ptr].Parent];
	}
	while(stackp) PrintLeg(stack[--stackp]);
}


/***************************************************************************************/
/*CAMPUS TOUR                                                                          */
/***************************************************************************************/
#include "Tour.h"


/***************************************************************************************/
/*MAIN PROGRAM (don't modify)                                                          */
/***************************************************************************************/
int main() {
GetVertices();
GetEdges();
while (GetRequest()) {RouteOpen(); TourFlag ? Tour() : Dijkstra(0); RouteClose();}
return(0);
}
