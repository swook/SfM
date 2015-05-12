#include "opencv2/core.hpp"

#include "Viewer.hpp"
#include "util.hpp"

Viewer::Viewer()
:
	_viewer("Point Cloud")
{
	// Set viewer settings
	//_viewer.setFullScreen(true);
	_viewer.setShowFPS(true);
	//_viewer.setWindowBorders(false);

	//
	_viewer.setBackgroundColor(0.1, 0.1, 0.1);
	_viewer.addCoordinateSystem(0.1, "global");
}

void Viewer::showCloudPoints(PointCloud& cloud)
{
	typedef pcl::PointXYZ point_t;
	Logger _log("PCL");

	// Fill cloud structure
	pcl::PointCloud<point_t>::Ptr pcl_cloud(new pcl::PointCloud<point_t>);

	for (int i = 0; i < cloud.size(); i++)
	{
		cv::Point3f point = cloud[i];
		point_t pcl_point;
		pcl_point.x = point.x;
		pcl_point.y = point.y;
		pcl_point.z = point.z;

		pcl_cloud->points.push_back(pcl_point);
	}

	// Show cloud
	_log("Showing %d points", pcl_cloud->points.size());
	_viewer.addPointCloud<point_t>(pcl_cloud);

	// Wait until closed
	while (!_viewer.wasStopped())
	{
		_viewer.spinOnce(15);
	}
}

