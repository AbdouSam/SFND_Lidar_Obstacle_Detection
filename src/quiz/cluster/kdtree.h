/* \author Aaron Brown */
// Quiz on implementing kd tree

namespace custom_impl
{

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
		int K;

		KdTree(int _k)
		: root(NULL), K(_k)
		{}

		~KdTree()
		{
			delete root;
		}

		void insert(std::vector<float> point, int id)
		{
			Node * node = new Node(point, id);

			if (this->root == NULL)
			{
				this->root = node;
				return;
			}

			int depth = 0;

			Node * temp = this->root;

			while (temp != NULL)
			{
				if (point[depth] < temp->point[depth])
				{
					// put left
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
				depth = (depth + 1) % K;
			}

		}

		bool is_point_inbox(std::vector<float>& point, std::vector<float>& target, float distanceTol)
		{
			// sanity check
			bool p_in_box = true;

			if ((point.size() != target.size()) && (point.size() > 0))
				return false;

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

			int depth = 0;
			std::vector<Node *> stack;

			stack.push_back(this->root);

			while (!stack.empty())
			{
				Node * node = stack.back();
				stack.pop_back();

				if (is_point_inbox(node->point, target, distanceTol))
				{
					if (is_point_inraduis(node->point, target, distanceTol))
						ids.push_back(node->id);
				}

				// navigate left or right
				if ((target[depth] - distanceTol) < node->point[depth])
				{
					if (node->left != NULL)
						stack.push_back(node->left);
				}

				if ((target[depth] + distanceTol) > node->point[depth])
				{
					if (node->right != NULL)
						stack.push_back(node->right);
				}

				depth = (depth + 1) % K;
			}

			return ids;
		}
		
	};

}
