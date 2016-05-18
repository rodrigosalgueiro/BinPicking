#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#include <pcl/ModelCoefficients.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>

#include <pcl/segmentation/extract_polygonal_prism_data.h>

#include <pcl/visualization/pcl_visualizer.h>
//#include <Eigen/Dense>




typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;




int
main (int argc, char *argv[])
{

	PointCloudT::Ptr	cloud (new PointCloudT),
				cloud_inliers_table (new PointCloudT),
				cloud_outliers_table (new PointCloudT),
				cloud_edge (new PointCloudT),
				cloud_edge_projected (new PointCloudT),
				cloud_poligonal_prism (new PointCloudT);


	// Load point cloud
	if (pcl::io::loadPCDFile ("caixacomobjectos.pcd", *cloud) < 0) {
		PCL_ERROR ("Could not load PCD file !\n");
		return (-1);
	}

///////////////////////////////////////////
//          Table Plane Extract          //
///////////////////////////////////////////

	// Segment the ground
	pcl::ModelCoefficients::Ptr plane_table (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_plane_table (new pcl::PointIndices);
	PointCloudT::Ptr cloud_plane_table (new PointCloudT);

	// Make room for a plane equation (ax+by+cz+d=0)
	plane_table->values.resize (4);

	pcl::SACSegmentation<PointT> seg_table;	// Create the segmentation object
	seg_table.setOptimizeCoefficients (true);			// Optional
	seg_table.setMethodType (pcl::SAC_RANSAC);
	seg_table.setModelType (pcl::SACMODEL_PLANE);
	seg_table.setDistanceThreshold (0.025f);
	seg_table.setInputCloud (cloud);
	seg_table.segment (*inliers_plane_table, *plane_table);

	if (inliers_plane_table->indices.size () == 0) {
		PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
		return (-1);
	}

	// Extract inliers
	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud (cloud);
	extract.setIndices (inliers_plane_table);
	extract.setNegative (false);			// Extract the inliers
	extract.filter (*cloud_inliers_table);		// cloud_inliers contains the plane

	// Extract outliers
	//extract.setInputCloud (cloud);		// Already done line 50
	//extract.setIndices (inliers);			// Already done line 51
	extract.setNegative (true);				// Extract the outliers
	extract.filter (*cloud_outliers_table);		// cloud_outliers contains everything but the plane

	printf ("Plane segmentation equation [ax+by+cz+d]=0: [%3.4f | %3.4f | %3.4f | %3.4f]     \t\n", 
			plane_table->values[0], plane_table->values[1], plane_table->values[2] , plane_table->values[3]);


	//guarda a nuvem filtrada num novo ficheiro 
	//pcl::io::savePCDFileASCII ("modelfiltered.pcd", *cloud_outliers);

///////////////////////////////////////////
//           Box Edge Extract            //
///////////////////////////////////////////
	
	pcl::PassThrough<PointT> pass;
    	pass.setInputCloud (cloud_outliers_table);
    	pass.setFilterFieldName ("z");
    	pass.setFilterLimits (0.73, 0.76);
    	pass.filter (*cloud_edge);

	pcl::ModelCoefficients::Ptr coefficients_edge (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_edge (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg_edge;
	// Optional
	seg_edge.setOptimizeCoefficients (true);
	// Mandatory
	seg_edge.setModelType (pcl::SACMODEL_PLANE);
	seg_edge.setMethodType (pcl::SAC_RANSAC);
	seg_edge.setDistanceThreshold (0.01);

	seg_edge.setInputCloud (cloud_edge);
	seg_edge.segment (*inliers_edge, *coefficients_edge);


	///////////////////////////////
	// Project the model inliers //
	///////////////////////////////
	pcl::ProjectInliers<pcl::PointXYZ> proj_edge;
	proj_edge.setModelType (pcl::SACMODEL_PLANE);
	proj_edge.setIndices (inliers_edge);
	proj_edge.setInputCloud (cloud_edge);
	proj_edge.setModelCoefficients (coefficients_edge);
	proj_edge.filter (*cloud_edge_projected);
	
	////////////////////////////////////////////////////////////////////
	//       Create a Concave Hull representation of the box edge     //
	////////////////////////////////////////////////////////////////////
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_edge_hull (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ConvexHull<pcl::PointXYZ> chull;
	chull.setInputCloud (cloud_edge_projected);
	//chull.setAlpha (0.1);
	chull.reconstruct (*cloud_edge_hull);

	///////////////////////////////////////////////
	// Create a Poligonal Prism of the box edge  //
	///////////////////////////////////////////////
	pcl::PointIndices::Ptr inliers_poligonal_prism (new pcl::PointIndices);
	double z_min = 2.1, z_max = 2.1; // we want the points above the plane, no farther than 5 cm from the surface
	pcl::ExtractPolygonalPrismData<pcl::PointXYZ> prism;
	prism.setInputCloud (cloud_edge_projected);
	prism.setInputPlanarHull (cloud_edge_hull);
	prism.setHeightLimits (z_min, z_max);
	prism.segment (*inliers_poligonal_prism);

	// Extract poligonal inliers
	pcl::ExtractIndices<PointT> extract_polygonal_data;
	extract_polygonal_data.setInputCloud (cloud);
	extract_polygonal_data.setIndices (inliers_poligonal_prism);
	extract_polygonal_data.setNegative (false);		// Extract the inliers
	extract_polygonal_data.filter (*cloud_poligonal_prism);	// cloud_poligonal_prism contains the box



///////////////////
// Visualization //
///////////////////
	pcl::visualization::PCLVisualizer viewer ("PCL visualizer");

	// Table Plane
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_inliers_table_handler (cloud, 20, 255, 255); 
	viewer.addPointCloud (cloud_inliers_table, cloud_inliers_table_handler, "cloud inliers");

	// Everything else in GRAY
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_outliers_table_handler (cloud, 200, 200, 200); 
	viewer.addPointCloud (cloud_outliers_table, cloud_outliers_table_handler, "cloud outliers");
	/*
	// Edge in Green
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_edge_handler (cloud, 20, 255, 20); 
	viewer.addPointCloud (cloud_edge, cloud_edge_handler, "cloud edge");

	// Edge projected
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_edge_projected_handler (cloud, 255, 69, 20); 
	viewer.addPointCloud (cloud_edge_projected, cloud_edge_projected_handler, "cloud edge projected");
	*/
	// Edge hull
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_edge_hull_handler (cloud, 255, 20, 20); 
	viewer.addPointCloud (cloud_edge_hull, cloud_edge_hull_handler, "cloud edge hull");

	// Poligonal Prism data
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_poligonal_prism_handler (cloud, 20, 20, 255); 
	viewer.addPointCloud (cloud_poligonal_prism, cloud_poligonal_prism_handler, "cloud Poligonal Prism");
	

	while (!viewer.wasStopped ()) {
		viewer.spinOnce ();
	}
	return (0);
}
