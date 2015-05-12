#pragma once

#include <pcl/visualization/pcl_visualizer.h>
namespace vis = pcl::visualization;

#include "structures.hpp"
#include "util.hpp"

class Viewer
{
public:
	Viewer();
	void showCloudPoints(PointCloud& cloud, Images& images,
		CamFrames& camFrames, PointMap& pointMap);

private:
	vis::PCLVisualizer _viewer;
	Logger             _log;
};

