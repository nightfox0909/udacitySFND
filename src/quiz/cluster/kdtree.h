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
      insert_helper(root,point,id,0);
		// the function should create a new node and place correctly with in the root 

	}
  
    void insert_helper(Node *&node, std::vector<float> point, int id, int depth)
   {
      int index = 1;
      if ( depth % 2 == 0)
      {index=0;}
      
      if(node == NULL)
      {
        node = new Node(point,id);
      }
      else if ( point[index] < node->point[index])
      	{
        	insert_helper(node->left, point,id,depth+1);
      	} 
      else
      {
        insert_helper(node->right, point,id,depth+1);
      }
   }

  void search_helper(std::vector<float> target,Node* node,float distanceTol, std::vector<int>& ids,int depth)
	{
		int index=1;
		if ( node==NULL)
		{
			return;
		}
			float x1=node->point[0];
			float x2=target[0];
			float y1=node->point[1];
			float y2=target[1];
		if(((x1+distanceTol>=x2)) && ((x1-distanceTol<=x2)) && ((y1+distanceTol>=y2)) && ((y1-distanceTol<=y2)))
		{

			float distance = sqrt (((x1-x2)*(x1-x2)) + ((y1-y2)*(y1-y2)));
			if(distance<=distanceTol)
			{
				ids.push_back(node->id);
			}
		}

		if(target[depth%2]-distanceTol<node->point[depth%2])
		{search_helper(target,node->left,distanceTol,ids,depth+1);}

		if(target[depth%2]+distanceTol>node->point[depth%2])
		{search_helper(target,node->right,distanceTol,ids,depth+1);}
	}
	// return a list of point ids in the tree that are within distance of target
std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		search_helper(target,root,distanceTol,ids,0);
		return ids;
	}
	

};


