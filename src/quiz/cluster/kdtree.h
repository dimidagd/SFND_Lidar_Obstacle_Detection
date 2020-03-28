/* \author Aaron Brown */
// Quiz on implementing kd tree

//#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}
	void insert_helper(Node** node, uint depth, std::vector<float> point, int id)
	{
		if (*node == NULL)
			*node = new Node(point, id);
		else
		{
			uint cd  = depth%3;

			if ( point[cd] < (*node)->point[cd])
				insert_helper(&((*node)->left), depth+1, point, id);
			else
				insert_helper(&((*node)->right), depth+1, point, id); 

		}
	}
	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insert_helper(&root, 0, point, id);

	}

	void search_helper(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>& ids)
	{
		if (node != NULL)
		{
			if(is_in_box(node, target, distanceTol))
			{
					float d = distance(node, target);
					if (d <= distanceTol)
						ids.push_back(node->id);
						
			}
			//boundary box

			if ((target[depth%3]-distanceTol) < node->point[depth%3])
				search_helper(target, node->left, depth+1, distanceTol, ids);
			if ((target[depth%3]+distanceTol) > node->point[depth%3])
				search_helper(target, node->right, depth+1, distanceTol, ids);

		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		search_helper(target, root, 0, distanceTol, ids);
		return ids;
	}

	static bool is_in_box(Node* node, std::vector<float> target,float distanceTol)
	{
		return ((fabs(node->point[0]-target[0]) < distanceTol) && (fabs(node->point[1]-target[1]) < distanceTol) && (fabs(node->point[2]-target[2]) < distanceTol));
	}
	static float distance(Node* node, std::vector<float> target)
	{
		float dx = node->point[0] - target[0];
		float dy = node->point[1] - target[1];
		float dz = node->point[2] - target[2];
		return sqrt(dx*dx + dy*dy + dz*dz);
	}
};




