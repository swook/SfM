#pragma once

#include <pcl/visualization/pcl_visualizer.h>
namespace vis = pcl::visualization;

#include "structures.hpp"
#include "util.hpp"

class Viewer
{
public:
	Viewer();
	void showCloudPoints(const Images& images, const CameraPoses& poses,
		const cv::Mat& cameraMatrix);

private:
	vis::PCLVisualizer _viewer;
	Logger             _log;
};

