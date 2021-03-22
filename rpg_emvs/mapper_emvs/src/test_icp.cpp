
#include <iostream>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

void PointsRegistration()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr targetCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr finalCloud(new pcl::PointCloud<pcl::PointXYZ>);
  
  //Assign both point clouds as source and target
  pcl::io::loadPCDFile<pcl::PointXYZ>("plate_pcd.pcd", *sourceCloud);
  pcl::io::loadPCDFile<pcl::PointXYZ>("Voxel_holes_inertial_.pcd", *targetCloud);
  
  
  // ICP object.
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> registration;
	registration.setInputSource(sourceCloud);
	registration.setInputTarget(targetCloud);

	registration.align(*finalCloud);
	if (registration.hasConverged())
	{
		std::cout << "ICP converged." << std::endl
				  << "The score is " << registration.getFitnessScore() << std::endl;
		std::cout << "Transformation matrix:" << std::endl;
		std::cout << registration.getFinalTransformation() << std::endl;
	}
	else std::cout << "ICP did not converge." << std::endl;

    pcl::io::savePCDFileASCII ("test_icp.pcd", *finalCloud);
}

int main()
{
    PointsRegistration();

    return(0);
}