#include <opencv2/core.hpp>

#include <pcl/io/pcd_io.h>

#include "Viewer.hpp"

Viewer::Viewer(const char* title)
:
	_log("PCL"),
	_title(title)
{
}

void Viewer::reduceCloud(cloud_t::Ptr& cloud)
{
	const float voxel_resolution = 20.f;

	// Configure grid
	_grid.setInputCloud(cloud);
	_grid.setLeafSize(voxel_resolution, voxel_resolution, voxel_resolution);

	// Filter
	cloud_t::Ptr reduc(new cloud_t);
	_grid.filter(*reduc);

	std::swap(cloud, reduc);
}

Viewer::cloud_t::Ptr Viewer::createPointCloud(const Images& images, const CameraPoses& poses,
	const cv::Mat& cameraMatrix)
{
	// Fill cloud structure
	cloud_t::Ptr pcl_points(new cloud_t);

	// Per camera
	for (int c = 0; c < poses.size(); c++)
	{
		Image   image = images[c];
		cv::Mat R     = poses[c].R;
		cv::Mat t     = poses[c].t;
		if (R.empty()) continue;

		// Per pixel
		const cv::Vec3b* rgbs;
		const float*     deps;

		cv::Vec3b rgb;
		float     dep;

		cv::Point3f point;
		cv::Mat     gPoint;

		for (int i = 0; i < image.dep.rows; i++)
		{
			rgbs = image.rgb.ptr<cv::Vec3b>(i);
			deps = image.dep.ptr<float>(i);
			for (int j = 0; j < image.dep.cols; j++)
			{
				rgb = rgbs[j];
				dep = deps[j];

				// Valid depth is between 40cm and 8m
				if (dep < 400 || dep > 8000) continue;

				// Calculate point pos in global coordinates
				point  = backproject3D(j, i, dep, cameraMatrix);
				gPoint = R.t() * cv::Mat(point) - t;
				point  = cv::Point3f(gPoint);

				point_t pcl_p;
				pcl_p.x = point.x;
				pcl_p.y = point.y;
				pcl_p.z = point.z;
				pcl_p.r = rgb[2];
				pcl_p.g = rgb[1];
				pcl_p.b = rgb[0];
				pcl_points->points.push_back(pcl_p);
			}
		}
		// Reduce points every 10 cameras
		if (c % 10 == 0) reduceCloud(pcl_points);
	}

	// Final reduction of points
	reduceCloud(pcl_points);

	_log("Generated %d points.", pcl_points->points.size());

	return pcl_points;
}

void Viewer::saveCloud(cloud_t::Ptr pcl_points, const char* fname)
{
	// TODO: Save to disk with timestamp
	pcl::PCDWriter writer;
	writer.writeBinary(fname, *pcl_points);
	_log("Saved point cloud to: %s", fname);
	_log.tok();
}

void Viewer::showCloudPoints(const Images& images, const CameraPoses& poses,
	const cv::Mat& cameraMatrix)
{
	cloud_t::Ptr pcl_points = createPointCloud(images, poses, cameraMatrix);
	showCloudPoints(pcl_points);
}

void Viewer::showCloudPoints(const cloud_t::Ptr pcl_points, bool wait)
{
	auto _viewer = vis::PCLVisualizer(_title);

	// Set viewer settings
	_viewer.setShowFPS(true);
	_viewer.setWindowBorders(false);

	_viewer.setBackgroundColor(0.1, 0.1, 0.1);
	_viewer.addCoordinateSystem(0.1, "global");

	// Show cloud
	_log("Showing %d points", pcl_points->points.size());
	vis::PointCloudColorHandlerRGBField<point_t> rgb_handler(pcl_points);
	_viewer.addPointCloud<pcl::PointXYZRGB>(pcl_points, rgb_handler);
	_viewer.spinOnce(1000);

	// Wait until closed
	while (wait && !_viewer.wasStopped())
		_viewer.spinOnce(15);
}

