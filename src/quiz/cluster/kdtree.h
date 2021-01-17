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

  // New points to be considered as roots filling suitable NULL nodes
  // insertHelper is using double pointer Node** node
  void insertHelper(Node** node, uint depth, std::vector<float> point, int id){

    // traverse through tree till NULL is hit, reassign it as a new root
    if(*node == NULL)
      // Make the root pointer to point at input point as a root
      // Node(point, id) is a function (struct KdTree) for creating
      // a new node (using a new point and id)
      *node = new Node(point, id);

    //  NULL isn't not hit yet, keep traverse
    else {
      // Calculate current dim (cd=0>>x,cd=1>>y,cd=2>>z)
      uint cd = depth % 3;

      // compare new point values to current node pointer
      if(point[cd] < ((*node)->point[cd]))

        // Decide to left branch by dereference to the node pointer's left child
        // Increase depth to keep traversing
        insertHelper(&((*node)->left), depth+1, point, id);

      else
        insertHelper(&((*node)->right), depth+1, point, id);
    }
  }

  void insert(std::vector<float> point, int id)
  {
    // TODO: Fill in this function to insert a new point into the tree
    // the function should create a new node and place correctly with in the root 

    /* insertHelper to make insert a recurssive function with start intial
       depth of 0 */

    // for every new point, start from depth zero and &root memory address

    insertHelper(&root, 0, point, id);
  }


  // return a list of point ids in the tree that are within distance of target
  std::vector<int> search(std::vector<float> target, float distanceTol)
  {
    std::vector<int> ids;
    searchHelper(target, root, 0, distanceTol, ids);
    return ids;
  }

  void searchHelper(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>& ids){
   
    if(node != NULL){

      // Check if point is within bounding tolerance box (l-r-d-up-out-in)
      if((node->point[0]>=(target[0]-distanceTol) && node->point[0]<=(target[0]+distanceTol)) \
         && (node->point[1]>=(target[1]-distanceTol) && node->point[1]<=(target[1]+distanceTol))\
         && (node->point[2]>=(target[2]-distanceTol) && node->point[2]<=(target[2]+distanceTol))) {
        
        float x = node->point[0] - target[0];
        float y = node->point[1] - target[1];
        float z = node->point[2] - target[2];
        float distance = sqrt ((x*x) + (y*y) + (z*z));
        if (distance <= distanceTol)
            ids.push_back(node->id);
      }
      // Traverse through tree (left/right wrt box boundries and depth ++)
      if((target[depth%3]-distanceTol) < node->point[depth%3])
          searchHelper(target, node->left, depth+1, distanceTol, ids);

      // You had a sign problem, previously used - (instead of +)
      if((target[depth%3]+distanceTol) > node->point[depth%3])
          searchHelper(target, node->right, depth+1, distanceTol, ids);
    }
  }
};