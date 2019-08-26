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

template<typename PointT>
float getValueByDepth (int depth, PointT point)
{
	if (depth % 3 == 0 )
      	{ return point.x; }
	if ( (depth-1) % 3 == 0)
      	{ return point.y; }
	return point.z;
}

template<typename PointT>
struct Node
{
	PointT point;
	int id;
	Node* left;
	Node* right;

	Node(PointT arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}

};

template<typename PointT>
struct KdTree3D
{
	Node<PointT>* root;

	KdTree3D()
	: root(NULL)
	{}

	void setInputCloud(typename pcl::PointCloud<PointT>::Ptr cloud)
	{
	  for (int i=0;i<cloud->points.size();++i)
	  	{insert(cloud->points[i],i);}
	}

	void insert(PointT point, int id)
	{
      insert_helper(root,point,id,0);
	}
  
  void insert_helper(Node<PointT> *&node, PointT point, int id, int depth)
   {
      
      if(node == NULL)
      {
        node = new Node<PointT>(point,id);
      }
      else if ( getValueByDepth(depth,point) < getValueByDepth(depth,node->point))
      	{
        	insert_helper(node->left, point,id,depth+1);
      	} 
      else
      {
        insert_helper(node->right, point,id,depth+1);
      }
   }

  void search_helper(PointT target,Node<PointT>* node,float distanceTol, std::vector<int>& ids,int depth)
	{

		if ( node==NULL)
		{
			return;
		}
		float x1=node->point.x;
		float x2=target.x;
		float y1=node->point.y;
		float y2=target.y;
    	float z1=node->point.z;
    	float z2=target.z;
    
		if((x1+distanceTol>=x2) && (x1-distanceTol<=x2) && (y1+distanceTol>=y2) && (y1-distanceTol<=y2) && (z1+distanceTol>=z2) && (z1-distanceTol<=z2))
		{
			float distance = sqrt ( ((x1-x2)*(x1-x2)) + ((y1-y2)*(y1-y2)) + ((z1-z2)*(z1-z2)) );
			if(distance<=distanceTol)
			{ids.push_back(node->id);}
		}

		if(getValueByDepth(depth,target)-distanceTol<getValueByDepth(depth,node->point))
		{search_helper(target,node->left,distanceTol,ids,depth+1);}

		if(getValueByDepth(depth,target)+distanceTol>getValueByDepth(depth,node->point))
		{search_helper(target,node->right,distanceTol,ids,depth+1);}
	}
	
  	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(PointT target, float distanceTol)
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

    //void clusterPointCloudsKdTree(std::vector<pcl::PointIndices>& clusterIndices,typename pcl::PointCloud<PointT>::Ptr cloud, float distanceTol);

    void proximity (std::vector<std::pair<PointT,bool>>& traverseMap,pcl::PointIndices& cluster, KdTree3D<PointT>* tree, float distanceTol,int index,int minClusterSize,int maxClusterSize);

    void euclideanCluster3D(std::vector<pcl::PointIndices>& clusters, typename pcl::PointCloud<PointT>::Ptr cloud, float distanceTol,int minClusterSize,int maxClusterSize);

    void RansacPlaneSeg (pcl::PointIndices::Ptr inliersOut, typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
  
};
#endif /* PROCESSPOINTCLOUDS_H_ */