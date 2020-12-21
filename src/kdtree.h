#ifndef _KDTREE_H_
#define _KDTREE_H_


/* \author Aaron Brown */
// Quiz on implementing kd tree

//#include "../../render/render.h"



// Class to represent node of kd tree
template<typename PointT>
class Node
{
	public:

	Node(const PointT &arr, int setId)
	:	point(arr), id(setId), left(nullptr), right(nullptr)
	{}

	PointT point;		// Poss more efficient to store a reference, but then the Node becomes dependent of the lifetime of the point cloud. Maybe not a good idea.
	int id;
	Node* left;
	Node* right;

};

template<typename PointT>
class KdTree
{
	public:

		KdTree()
		: root(nullptr)
		{}

		void insert(const PointT &point, int id)
		{
			// the function should create a new node and place correctly with in the root 
			insertHelper(&root, point, id, 0);
		}

		void insertHelper(Node<PointT> **node, const PointT &point, int id, int dataIdx)
		{
			if (*node == nullptr)
			{
				*node = new Node<PointT>(point, id);
			}
			else
			{
				if (point.data[dataIdx] < (*node)->point.data[dataIdx])
				{
					insertHelper(&(*node)->left, point, id, (dataIdx + 1) % 3);
				}
				else
				{
					insertHelper(&(*node)->right, point, id, (dataIdx + 1) % 3);
				}

			}
		}


		// return a list of point ids in the tree that are within distance of target
		std::vector<int> search(const PointT &target, float distanceTol)
		{
			std::vector<int> ids;
			searchHelper(root, ids, target, distanceTol, 0);
			return ids;
		}
		
		void searchHelper(Node<PointT> *node, std::vector<int> &ids, const PointT &target, float distanceTol, int dataIdx)
		{
			if (node == nullptr)
			{
				return;
			}

			float xsep = node->point.data[0] - target.data[0];
			float ysep = node->point.data[1] - target.data[1];
			float zsep = node->point.data[2] - target.data[2];
			if ( (std::abs(xsep) <= distanceTol) &&
				 (std::abs(ysep) <= distanceTol) &&
				 (std::abs(zsep) <= distanceTol) )
			{
				// Calculate the actual distance separation
				float sep = std::sqrt(xsep*xsep + ysep*ysep + zsep*zsep);
				if (sep < distanceTol)
					ids.push_back(node->id);
			}

			// Check if we need to consider the less than division
			if (target.data[dataIdx] - distanceTol < node->point.data[dataIdx])
			{
				searchHelper(node->left, ids, target, distanceTol, (dataIdx + 1) % 3);
			}

			// Check if we need to consider the greater than division
			if (target.data[dataIdx] + distanceTol >= node->point.data[dataIdx])
			{
				searchHelper(node->right, ids, target, distanceTol, (dataIdx + 1) % 3);
			}
		}

	private:

		Node<PointT> *root;

};

#endif


