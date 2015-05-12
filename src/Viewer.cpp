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

void Viewer::showCloudPoints(const Images& images, const CameraPoses& poses,
	const cv::Mat& cameraMatrix)
{
	// Fill cloud structure
	typedef pcl::PointXYZRGB point_t;
	pcl::PointCloud<point_t>::Ptr pcl_points(new pcl::PointCloud<point_t>);

	// Per camera
#pragma omp parallel for
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
#pragma omp critical
				pcl_points->points.push_back(pcl_p);
			}
		}
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

