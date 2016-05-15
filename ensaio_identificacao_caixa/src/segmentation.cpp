#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
//#include <Eigen/Dense>

#include <pcl/filters/passthrough.h>


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;




int
main (int argc, char *argv[])
{

	PointCloudT::Ptr	cloud (new PointCloudT),
				cloud_inliers (new PointCloudT),
				cloud_outliers (new PointCloudT),
				cloud_edge (new PointCloudT);


	// Load point cloud
	if (pcl::io::loadPCDFile ("caixacomobjectos.pcd", *cloud) < 0) {
		PCL_ERROR ("Could not load PCD file !\n");
		return (-1);
	}

///////////////////////////////////////////
//             Plane Extract             //
///////////////////////////////////////////

	// Segment the ground
	pcl::ModelCoefficients::Ptr plane (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
	PointCloudT::Ptr cloud_plane (new PointCloudT);

	// Make room for a plane equation (ax+by+cz+d=0)
	plane->values.resize (4);

	pcl::SACSegmentation<PointT> seg;	// Create the segmentation object
	seg.setOptimizeCoefficients (true);			// Optional
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setDistanceThreshold (0.025f);
	seg.setInputCloud (cloud);
	seg.segment (*inliers_plane, *plane);

	if (inliers_plane->indices.size () == 0) {
		PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
		return (-1);
	}

	// Extract inliers
	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud (cloud);
	extract.setIndices (inliers_plane);
	extract.setNegative (false);			// Extract the inliers
	extract.filter (*cloud_inliers);		// cloud_inliers contains the plane

	// Extract outliers
	//extract.setInputCloud (cloud);		// Already done line 50
	//extract.setIndices (inliers);			// Already done line 51
	extract.setNegative (true);				// Extract the outliers
	extract.filter (*cloud_outliers);		// cloud_outliers contains everything but the plane

	printf ("Plane segmentation equation [ax+by+cz+d]=0: [%3.4f | %3.4f | %3.4f | %3.4f]     \t\n", 
			plane->values[0], plane->values[1], plane->values[2] , plane->values[3]);


	//guarda a nuvem filtrada num novo ficheiro 
	//pcl::io::savePCDFileASCII ("modelfiltered.pcd", *cloud_outliers);

///////////////////////////////////////////
//           Box Edge Extract            //
///////////////////////////////////////////
	
	pcl::PassThrough<PointT> pass;
    	pass.setInputCloud (cloud_outliers);
    	pass.setFilterFieldName ("z");
    	pass.setFilterLimits (0.73, 0.76);
    	pass.filter (*cloud_edge);


///////////////////
// Visualization //
///////////////////
	pcl::visualization::PCLVisualizer viewer ("PCL visualizer");

	// Plane in RED
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_inliers_handler (cloud, 255, 20, 20); 
	viewer.addPointCloud (cloud_inliers, cloud_inliers_handler, "cloud inliers");

	// Everything else in GRAY
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_outliers_handler (cloud, 200, 200, 200); 
	viewer.addPointCloud (cloud_outliers, cloud_outliers_handler, "cloud outliers");

	// Edge in Green
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_edge_handler (cloud, 20, 255, 20); 
	viewer.addPointCloud (cloud_edge, cloud_edge_handler, "cloud edge");	

	while (!viewer.wasStopped ()) {
		viewer.spinOnce ();
	}
	return (0);
}
