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
		insertHelper(point, &root, 0, id);
		
	}

	void insertHelper(std::vector<float> point, Node **node, int depth, int id) {
		if (*node == NULL) {
			*node = new Node(point, id);
		} else {
			if (depth%2 == 0) {
				if (point[0] < (*node)->point[0]) {
					insertHelper(point, &(*node)->left, depth+1, id);
				} else {
					insertHelper(point, &(*node)->right, depth+1, id);
				}
			} else {
				if (point[1] < (*node)->point[1]) {
					insertHelper(point, &(*node)->left, depth+1, id);
				} else {
					insertHelper(point, &(*node)->right, depth+1, id);
				}
			}
		}
	}


	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{	
		std::vector<int> ids;
		searchHelper(target, distanceTol, ids, root, 0);
		return ids;
	}
	
	void searchHelper(std::vector<float> target, float distanceTol, 
					std::vector<int> &ids, Node *node, int depth) {
		
		if (node == NULL) {
			return;
		}
		if (node->point[0] < target[0] + distanceTol && node->point[0] > target[0] - distanceTol
			&& node->point[1] < target[1] + distanceTol && node->point[1] > target[1] - distanceTol) {
				ids.push_back(node->id);
			}
		int switcher = depth%2;
		if (target[switcher] < node->point[switcher]) {
			searchHelper(target, distanceTol, ids, node->left, depth+1);
		} else {
			searchHelper(target, distanceTol, ids, node->right, depth+1);
		}
	}

};




