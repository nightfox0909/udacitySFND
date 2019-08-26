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
void ProcessPointClouds<PointT>::proximity (std::vector<std::pair<PointT,bool>>& traverseMap,pcl::PointIndices& cluster, KdTree3D<PointT>* tree, float distanceTol,int index,int minClusterSize,int maxClusterSize)
{
  traverseMap[index].second=true;
  cluster.indices.push_back(index);
  std::vector<int> indices=tree->search(traverseMap[index].first,distanceTol);
  
  for (int i : indices)
	{
    if(!traverseMap[i].second)
    {
      proximity(traverseMap,cluster,tree,distanceTol,i,minClusterSize,maxClusterSize);
    }
  }
}

template<typename PointT>
void ProcessPointClouds<PointT>::euclideanCluster3D(std::vector<pcl::PointIndices>& clusters, typename pcl::PointCloud<PointT>::Ptr cloud, float distanceTol,int minClusterSize,int maxClusterSize)
{
  //auto startTime = std::chrono::steady_clock::now(); 
   std::vector<std::pair<PointT,bool>> traverseMap;
   KdTree3D<PointT>* tree = new KdTree3D<PointT>; 

    for (int i=0; i<cloud->points.size(); ++i)
    {
      std::pair<PointT,bool> pointMap (cloud->points[i],false);
      traverseMap.push_back(pointMap);
      tree->insert(cloud->points[i],i);
    }
   
  	for ( int i=0 ; i<cloud->points.size(); ++i)
    {
      if (!traverseMap[i].second)
      {
      pcl::PointIndices newCluster;
      proximity(traverseMap,newCluster,tree,distanceTol,i,minClusterSize,maxClusterSize);
      if(newCluster.indices.size()>minClusterSize && newCluster.indices.size()<maxClusterSize)
       {clusters.push_back(newCluster);}
      }
    }
	
    //auto endTime = std::chrono::steady_clock::now();
  	//auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  	//std::cout << "clustering found " << clusters.size() << " and took " << elapsedTime.count() << " milliseconds" << std::endl;
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
    den=sqrt((a*a)+(b*b)+(c*c));


		for (auto point_iter=cloud->points.begin(); point_iter!=cloud->points.end();point_iter++)
		{
			float distance;
			distance=fabs((a*point_iter->x)+(b*point_iter->y)+(c*point_iter->z)+d)/den;
			//std::cout<<distance;
			if (distance < distanceTol)
			{
				tempInliers.insert(idx);
			}
				idx+=1;
		}



		if(tempInliers.size() > inliersOut->indices.size())
		{ 
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
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
    
    /*
    pcl::SACSegmentation<PointT> seg;
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());
    seg.setOptimizeCoefficients(false);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);
    seg.setInputCloud(cloud);
    seg.segment(*inliers,*coefficients);
    */
  
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
  
  	/*
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  	tree->setInputCloud(cloud);
  	pcl::EuclideanClusterExtraction<PointT> ec;
  	ec.setClusterTolerance(clusterTolerance);
  	ec.setMinClusterSize(minSize);
  	ec.setMaxClusterSize(maxSize);
  	ec.setSearchMethod(tree);
  	ec.setInputCloud(cloud);
  	ec.extract(clusterIndices);*/

  euclideanCluster3D(clusterIndices,cloud,clusterTolerance,minSize,maxSize);
  
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
BoxQ ProcessPointClouds<PointT>::BoundingBoxQ(typename pcl::PointCloud<PointT>::Ptr cloud)
{

    BoxQ box;

    // Compute principal directions
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cloud, pcaCentroid);
    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

    // Transform the original cloud to the origin where the principal components correspond to the axes.
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
    typename pcl::PointCloud<PointT>::Ptr cloudPointsProjected (new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*cloud, *cloudPointsProjected, projectionTransform);

    // Get the minimum and maximum points of the transformed cloud.
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    // Final transform
    box.bboxQuaternion = eigenVectorsPCA; 
    box.bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();
    box.cube_length = maxPoint.x - minPoint.x;
    box.cube_width = maxPoint.y - minPoint.y;
    box.cube_height = maxPoint.z - minPoint.z;

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