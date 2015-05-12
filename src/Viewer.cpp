#include "opencv2/core.hpp"

#include "Viewer.hpp"

Viewer::Viewer()
:
	_log("PCL"),
	_viewer("Point Cloud")
{
	// Set viewer settings
	_viewer.setShowFPS(true);
	_viewer.setWindowBorders(false);

	_viewer.setBackgroundColor(0.1, 0.1, 0.1);
	_viewer.addCoordinateSystem(0.1, "global");
}

void Viewer::showCloudPoints(PointCloud& cloud, Images& images,
	CamFrames& camFrames, PointMap& pointMap)
{
	// Get rgb colours for each point in pointCloud
	RGBCloud rgb(cloud.size());
	for (auto& kv: pointMap)
	{
		int i = kv.first.first,
		    k = kv.first.second,
		    p = kv.second;
		cv::Vec3b bgr = images[i].rgb.at<cv::Vec3b>(camFrames[i].key_points[k].pt);
		rgb[p][0] = bgr[2];
		rgb[p][1] = bgr[1];
		rgb[p][2] = bgr[0];
	}

	// Fill cloud structure
	typedef pcl::PointXYZRGB point_t;
	pcl::PointCloud<point_t>::Ptr pcl_points(new pcl::PointCloud<point_t>);

	for (int i = 0; i < cloud.size(); i++)
	{
		point_t pcl_point;
		pcl_point.x = cloud[i].x;
		pcl_point.y = cloud[i].y;
		pcl_point.z = cloud[i].z;
		pcl_point.r = rgb[i][0];
		pcl_point.g = rgb[i][1];
		pcl_point.b = rgb[i][2];
		pcl_points->points.push_back(pcl_point);
	}

	// Show cloud
	_log("Showing %d points", pcl_points->points.size());
	vis::PointCloudColorHandlerRGBField<point_t> rgb_handler(pcl_points);
	_viewer.addPointCloud<pcl::PointXYZRGB>(pcl_points, rgb_handler);
	_log.tok();

	// Wait until closed
	while (!_viewer.wasStopped())
	{
		_viewer.spinOnce(15);
	}
}

