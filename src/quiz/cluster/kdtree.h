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

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

	void insert(std::vector<float> point, int id)
	{
		int depth = point.size();
		int curr_depth = 0;
		Node * node = new Node(point, id);

		if (this->root == NULL)
		{
			this->root = node;
			return;
		}

		if (depth < 1)
			return;

		Node * temp = this->root;

		while (temp != NULL)
		{
			if (point[curr_depth] >= temp->point[curr_depth])
			{
				//put left
				if (temp->left == NULL)
				{
					temp->left = node;
					temp = NULL;
				}
				else
				{
					temp = temp->left;
				}
			}
			else
			{
				// put to the right
				if (temp->right == NULL)
				{
					temp->right = node;
					temp = NULL;
				}
				else
				{
					temp = temp->right;
				}
			}
			curr_depth = (curr_depth + 1) % depth;
		}

	}

	bool is_point_inbox(std::vector<float>& point, std::vector<float>& target, float distanceTol)
	{
		bool p_in_box = true;

		// sanity check
		if ((point.size() != target.size()) && (point.size() > 0))
			return false;

		// if the point is out of one boundary of the target it is out
		for (int i = 0; i < point.size(); i++)
		{
			if (point[i] < (target[i] - distanceTol) || (point[i] > target[i] + distanceTol))
			{
				p_in_box = false;
				break;
			}
		}
		
		return p_in_box;
	}
	
	bool is_point_inraduis(std::vector<float>& point, std::vector<float>& target, float distanceTol)
	{
		float distance = 0;

		if ((point.size() != target.size()) && (point.size() > 0))
			return false;


		for (int i = 0; i < point.size(); i++)
		{
			distance += (point[i] - target[i]) * ((point[i] - target[i]));
		}
		
		// instead of comparing sqrt we compare it squared for both
		if (distance <= (distanceTol * distanceTol))
			return true;
		
		return false;
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;

		int curr_depth = 0;
		int depth = target.size();
		std::vector<Node *> stack;

		// sanity check
		if (depth < 1)
			return ids;

		// we could have used recursion to avoid stacks, but if it is for embedded use
		// recursion is not recommended.

		stack.push_back(this->root);

		while (!stack.empty())
		{
			Node * node = stack.back();
			stack.pop_back();

			// first we use this cheap calculation to decide whether to the point is nearby
			if (is_point_inbox(node->point, target, distanceTol))
			{
				// if it is neaby, we perform this more expensive calculation to 
				if (is_point_inraduis(node->point, target, distanceTol))
					ids.push_back(node->id);
			}

			// navigate left, right or both
			if ((target[curr_depth] - distanceTol) < node->point[curr_depth])
			{
				if (node->left != NULL)
					stack.push_back(node->left);
			}

			if ((target[curr_depth] + distanceTol) > node->point[curr_depth])
			{
				if (node->right != NULL)
					stack.push_back(node->right);
			}

			curr_depth = (curr_depth + 1) % depth;
		}

		return ids;
	}

};




