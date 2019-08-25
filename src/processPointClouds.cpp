// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

template<typename PointT>
void ProcessPointClouds<PointT>::proximity (std::vector<std::pair<std::vector<float>,bool>>& traverseMap,std::vector<int>& cluster, KdTree3D* tree, float distanceTol,int index)
{
  traverseMap[index].second=true;
  cluster.push_back(index);
  std::vector<int> indices=tree->search(traverseMap[index].first,distanceTol);
  
  for (int i : indices)
	{
    if(!traverseMap[i].second)
    {
      proximity(traverseMap,cluster,tree,distanceTol,i);
    }
  }
}

template<typename PointT>
void ProcessPointClouds<PointT>::euclideanCluster3D(std::vector<std::vector<int>>& clusters, const std::vector<std::vector<float>>& points, KdTree3D* tree, float distanceTol)
{
   std::vector<std::pair<std::vector<float>,bool>> traverseMap; 

	//std::vector<std::vector<int>> clusters;
    for ( int it=0 ; it< points.size(); ++it)
    {
      std::pair<std::vector<float>,bool> pointMap (points[it],false);
      traverseMap.push_back(pointMap);
    }
   
  	for ( int i=0 ; i<points.size(); ++i)
    {
      if (!traverseMap[i].second)
      {
      std::vector<int> newCluster;
      proximity(traverseMap,newCluster,tree,distanceTol,i);
      clusters.push_back(newCluster);
      }
    }
	//return clusters;
}

template<typename PointT>
void ProcessPointClouds<PointT>::clusterPointCloudsKdTree(std::vector<pcl::PointIndices>& clusterIndices,typename pcl::PointCloud<PointT>::Ptr cloud, float distanceTol)
{
  auto startTime = std::chrono::steady_clock::now();
  
  std::vector<std::vector<float>> points;
  KdTree3D* tree = new KdTree3D;
  
  for (int i=0; i<cloud->points.size(); i++) 
      {
         
        std::vector<float> point;
        point.push_back(cloud->points[i].x);
        point.push_back(cloud->points[i].y);
        point.push_back(cloud->points[i].z);
        points.push_back(point);
        tree->insert(point,i);
      }
 
  	//
  	std::vector<std::vector<int>> clusters;
    euclideanCluster3D(clusters,points, tree, 3.0);
  
    for ( auto it=clusters.begin(); it!=clusters.end(); it++)
    {
      pcl::PointIndices tempindexes;
      for( auto it2=it->begin(); it2!=it->end();it2++)
      {
        tempindexes.indices.push_back(*it2);
      }
      clusterIndices.push_back(tempindexes);
    }
  	//
  	auto endTime = std::chrono::steady_clock::now();
  	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  	std::cout << "clustering found " << clusters.size() << " and took " << elapsedTime.count() << " milliseconds" << std::endl;
  
}

template<typename PointT>
void ProcessPointClouds<PointT>::RansacPlaneSeg(pcl::PointIndices::Ptr inliersOut, typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	srand(time(NULL));
	// For max iterations 
	for( int iteration=0;iteration<maxIterations;iteration++)
	{
		std::unordered_set<int> tempInliers;
		int i1,i2,i3,idx=0;
		float a,b,c,d,den;
		//int range = ;

		i1 = rand()% cloud->points.size();
		i2 = rand()% cloud->points.size();
		while(i2==i1)
		{
			i2=rand() % cloud->points.size();
		}
		
		i3=rand() % cloud->points.size();
		while(i3==i1 or i3==i2)
		{
			i3=rand() % cloud->points.size();
		}
    

		// line coeff

		a=((cloud->points[i2].y-cloud->points[i1].y)*(cloud->points[i3].z-cloud->points[i1].z))-((cloud->points[i2].z-cloud->points[i1].z)*(cloud->points[i3].y-cloud->points[i1].y));
		b=((cloud->points[i2].z-cloud->points[i1].z)*(cloud->points[i3].x-cloud->points[i1].x))-((cloud->points[i2].x-cloud->points[i1].x)*(cloud->points[i3].z-cloud->points[i1].z)); 
		c=((cloud->points[i2].x-cloud->points[i1].x)*(cloud->points[i3].y-cloud->points[i1].y))-((cloud->points[i2].y-cloud->points[i1].y)*(cloud->points[i3].x-cloud->points[i1].x));
		d=-((a*cloud->points[i1].x)+(b*cloud->points[i1].y)+(c*cloud->points[i1].z));


		for (auto point_iter=cloud->points.begin(); point_iter!=cloud->points.end();point_iter++)
		{
			float distance;
			distance=fabs((a*point_iter->x)+(b*point_iter->y)+(c*point_iter->z)+d)/sqrt((a*a)+(b*b)+(c*c));
			//std::cout<<distance;
			if (distance < distanceTol)
			{
				tempInliers.insert(idx);
			}
				idx+=1;
		}



		if(tempInliers.size() > inliersOut->indices.size())
		{ 
			//inliersResult = tempInliers;
		  inliersOut->indices.clear();
          for ( auto i=tempInliers.begin();i!=tempInliers.end();++i)
          {
            inliersOut->indices.push_back(*i);
          }
		}

	}

}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

	typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
	typename pcl::PointCloud<PointT>::Ptr cloud_region (new pcl::PointCloud<PointT>);

	pcl::VoxelGrid<PointT> vg;
	vg.setInputCloud(cloud);
	vg.setLeafSize(filterRes,filterRes,filterRes);
	vg.filter(*cloud_filtered);


	pcl::CropBox<PointT> cb(true);
	cb.setMin(minPoint);
	cb.setMax(maxPoint);
	cb.setInputCloud(cloud_filtered);
	cb.filter(*cloud_region);

	std::vector<int> indices;

	pcl::CropBox<PointT> roof(true);
	roof.setMin(Eigen::Vector4f(-1.5,-1.7,-1,1));
	roof.setMax(Eigen::Vector4f(2.6,1.7,4,1));
	roof.setInputCloud(cloud_region);
	roof.filter(indices);

	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

	for(int i : indices)
	{
		inliers->indices.push_back(i);
	}

	pcl::ExtractIndices<PointT> extractor;
	extractor.setInputCloud(cloud_region);
	extractor.setIndices(inliers);
	extractor.setNegative(true);
	extractor.filter(*cloud_region);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_region;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr cloudObjects (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr cloudFree (new pcl::PointCloud<PointT> ());

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloudObjects);

    extract.setNegative(true);
    extract.filter(*cloudFree);


    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudFree,cloudObjects);
    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	//pcl::PointIndices::Ptr inliers;
    //pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
    //pcl::SACSegmentation<PointT> seg;

    /*seg.setOptimizeCoefficients(false);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    seg.setInputCloud(cloud);
    seg.segment(*inliers,*coefficients);*/
  
	RansacPlaneSeg(inliers,cloud,maxIterations,distanceThreshold);


    if(inliers->indices.size()==0)
    {
        std::cout<<"No planes found in the given point cloud"<<std::endl;
    }

	std::cout<<"Indices: "<<inliers->indices.size()<<std::endl;

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
  	std::vector<pcl::PointIndices> clusterIndices;
  
  	/*typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  	tree->setInputCloud(cloud);
  	pcl::EuclideanClusterExtraction<PointT> ec;
  	ec.setClusterTolerance(clusterTolerance);
  	ec.setMinClusterSize(minSize);
  	ec.setMaxClusterSize(maxSize);
  	ec.setSearchMethod(tree);
  	ec.setInputCloud(cloud);
  	ec.extract(clusterIndices);*/

    clusterPointCloudsKdTree(clusterIndices,cloud,clusterTolerance);
  for (std::vector<pcl::PointIndices>::const_iterator cluster_iterator=clusterIndices.begin(); cluster_iterator!=clusterIndices.end(); ++cluster_iterator)
  {
    typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
    for (std::vector<int>::const_iterator point_iterator=cluster_iterator->indices.begin(); point_iterator!=cluster_iterator->indices.end(); ++point_iterator)
    {cloud_cluster->points.push_back(cloud->points[*point_iterator]);}
    cloud_cluster->width=cloud_cluster->points.size();
    cloud_cluster->height=1;
    cloud_cluster->is_dense=true;
    clusters.push_back(cloud_cluster);

  }
  

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}