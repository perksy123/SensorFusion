/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


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

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(&root, point, id, 0);
	}

	void insertHelper(Node **node, const std::vector<float> &point, int id, int dataIdx)
	{
		if (*node == nullptr)
		{
			*node = new Node(point, id);
		}
		else
		{
			if (point[dataIdx] < (*node)->point[dataIdx])
			{
				insertHelper(&(*node)->left, point, id, (++dataIdx) % 2);
			}
			else
			{
				insertHelper(&(*node)->right, point, id, (++dataIdx) % 2);
			}

		}
	}


	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		std::cout << "Target Pt: x = "  << target[0] << " y = " << target[1] << std::endl;
		searchHelper(root, ids, target, distanceTol, 0);
		return ids;
	}
	
	void searchHelper(Node *node, std::vector<int> &ids, const std::vector<float> &target, float distanceTol, int dataIdx)
	{
		if (node == nullptr)
		{
			return;
		}

		std::cout << "Considering Node: " << node->id << " Node.x = " << node->point[0] << " Node.y = " << node->point[1] << std::endl;
		float xsep = node->point[0] - target[0];
		float ysep = node->point[1] - target[1];
		if ( (std::abs(xsep) <= distanceTol) &&
			 (std::abs(ysep) <= distanceTol) )
		{
			// Calculate the actual distance separation
			float sep = std::sqrt(xsep*xsep + ysep*ysep);
			if (sep < distanceTol)
				ids.push_back(node->id);
		}

		// Check if we need to consider the less than division
		if (target[dataIdx] - distanceTol < node->point[dataIdx])
		{
			searchHelper(node->left, ids, target, distanceTol, (++dataIdx) % 2);
		}

		// Check if we need to consider the greater than division
		if (target[dataIdx] + distanceTol >= node->point[dataIdx])
		{
			searchHelper(node->right, ids, target, distanceTol, (++dataIdx) % 2);
		}
	}

};




