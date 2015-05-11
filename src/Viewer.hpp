#pragma once

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
namespace vis = pcl::visualization;

#include "structures.hpp"

class Viewer
{
public:
	Viewer();
	void showCloudPoints(PointCloud& cloud);

private:
	vis::PCLVisualizer _viewer;
};

