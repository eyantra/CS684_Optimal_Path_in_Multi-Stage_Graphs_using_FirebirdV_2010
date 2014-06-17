
#define NULL 0

/**
* This represents the Node of the Tree Structure built based on the Graph
*/
struct Node
{
	struct Node* parent;		// Pointer to the parent Node
	struct Node* children[4];	// Pointer to the 4 children pointers

	int angles[4];				// Angle of each of 4 childrens with  horizontal axis
	int parAngle;				// Angle with the parent

	int visited[6];				//node is visited or not
	int validChildIndex;		// Index of the Last Valid child.
};


