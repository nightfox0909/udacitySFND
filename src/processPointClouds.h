// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include "render/box.h"
#include <unordered_set>
//#include "3DSegmentation/RANSACSegmentation.h"
//#include "3DClustering/clustering.h"

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

struct KdTree3D
{
	Node* root;

	KdTree3D()
	: root(NULL)
	{}

  template<typename PointT>
	void setInputCloud(typename pcl::PointCloud<PointT>::Ptr cloud)
	{
		for (int i=0;i<cloud->points.size();++i)
	  {
			std::vector<float> p;
			p.push_back(cloud->points[i].x);
			p.push_back(cloud->points[i].y);
			p.push_back(cloud->points[i].z);
			insert(p,i);
		}
	}

	void insert(std::vector<float> point, int id)
	{
      insert_helper(root,point,id,0);
	}
  
  void insert_helper(Node *&node, std::vector<float> point, int id, int depth)
   {
      int index = 1;
      if ( depth % 2 == 0)
      { index=0; }
      if (depth % 3 == 0 )
      { index=2; }
      
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
	  int index = 1;
      if ( depth % 2 == 0)
      { index=0; }
      if (depth % 3 == 0 )
      { index=2; }
    
		if ( node==NULL)
		{
			return;
		}
		float x1=node->point[0];
		float x2=target[0];
		float y1=node->point[1];
		float y2=target[1];
    	float z1=node->point[2];
    	float z2=target[2];
    
		if((x1+distanceTol>=x2) && (x1-distanceTol<=x2) && (y1+distanceTol>=y2) && (y1-distanceTol<=y2) && (z1+distanceTol>=z2) && (z1-distanceTol<=z2))
		{
			float distance = sqrt ( ((x1-x2)*(x1-x2)) + ((y1-y2)*(y1-y2)) + ((z1-z2)*(z1-z2)) );
			if(distance<=distanceTol)
			{ids.push_back(node->id);}
		}

		if(target[index]-distanceTol<node->point[index])
		{search_helper(target,node->left,distanceTol,ids,depth+1);}

		if(target[index]+distanceTol>node->point[index])
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


template<typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void clusterPointCloudsKdTree(std::vector<pcl::PointIndices>& clusterIndices,typename pcl::PointCloud<PointT>::Ptr cloud, float distanceTol);

    void proximity (std::vector<std::pair<std::vector<float>,bool>>& traverseMap,std::vector<int>& cluster, KdTree3D* tree, float distanceTol,int index);

    void euclideanCluster3D(std::vector<std::vector<int>>& clusters, const std::vector<std::vector<float>>& points, KdTree3D* tree, float distanceTol);

    void RansacPlaneSeg (pcl::PointIndices::Ptr inliersOut, typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
  
};
#endif /* PROCESSPOINTCLOUDS_H_ */