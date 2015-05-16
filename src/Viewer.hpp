#pragma once

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
namespace vis = pcl::visualization;

#include "structures.hpp"
#include "util.hpp"

class Viewer
{
public:
	typedef pcl::PointXYZRGB         point_t;
	typedef pcl::PointCloud<point_t> cloud_t;
	typedef pcl::VoxelGrid<point_t>  grid_t;

	Viewer();
	cloud_t::Ptr createPointCloud(const Images& images, const CameraPoses& poses,
	                              const cv::Mat& cameraMatrix);
	void         saveCloud(cloud_t::Ptr cloud, const char* fname);
	void         showCloudPoints(const Images& images, const CameraPoses& poses,
	                             const cv::Mat& cameraMatrix);
	void         showCloudPoints(const cloud_t::Ptr pcl_points, bool wait=true);

private:
	vis::PCLVisualizer _viewer;
	Logger             _log;
	grid_t             _grid;

	void reduceCloud(cloud_t::Ptr& cloud);
};

