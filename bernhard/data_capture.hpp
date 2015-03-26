/*
 * data_capture.hpp
 *
 *  Created on: March 7, 2012
 *      Author: Bernhard Zeisl
 *              Computer Vision and Geometry Group, ETH Zurich
 *              zeislb@inf.ethz.ch
 *     Version: 1.0
 */

#ifndef DATA_CAPTURE_HPP_
#define DATA_CAPTURE_HPP_

#include <data_capture.h>

#include <pcl/common/time.h>

#include <opencv2/core/core.hpp>

///////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
FrameCloudView<PointT>::show ()
{
  if (cloud_ && cld_mutex_.try_lock())
  {
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> color_handler (cloud_);

    if (!visualizer_.updatePointCloud<PointT> (cloud_, color_handler, cloud_id_))
    {
      visualizer_.addPointCloud<PointT> (cloud_, color_handler, cloud_id_);
      visualizer_.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloud_id_);
    }
    cld_mutex_.unlock();
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
DataCapture::cloud_callback (const typename pcl::PointCloud<PointT>::ConstPtr &cloud)
{
  evaluateFrameRate();

  if (continuous_capture_)
  {
    std::string filename = generateFilename();
    saveCloud<PointT> (filename, *cloud);
  }
  else if (save_)
  {
    save_ = false;
    std::string filename = generateFilename();
    saveCloud<PointT> (filename, *cloud);
  }

}

///////////////////////////////////////////////////////////////////////////////////////////////////
// template specialization for RGB point cloud, which is also visualized
template <> void
DataCapture::cloud_callback<pcl::PointXYZRGBA> (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
{
  // provide new cloud to visualization
  cloud_viewer_.setCloud (cloud);

  // only save the cloud, if requested so
  if (capture_method_ == CLOUD_RGB)
  {
    evaluateFrameRate();

    if (continuous_capture_)
    {
      std::string filename = generateFilename();
      saveCloud<pcl::PointXYZRGBA> (filename, *cloud);
    }
    else if (save_)
    {
      save_ = false;
      std::string filename = generateFilename();
      saveCloud<pcl::PointXYZRGBA> (filename, *cloud);
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
DataCapture::saveCloud (const std::string &filename, const pcl::PointCloud<PointT> &cloud)
{
  switch (format_)
  {
    case BINARY:
      writer_.writeBinary<PointT> (filename + ".pcd", cloud);
      // std::cout << "Pointcloud saved in BINARY format to " << filename << ".pcd" << std::endl;
      break;
    case BINARY_COMPRESSED:
      writer_.writeBinaryCompressed<PointT> (filename + ".pcd", cloud);
      // std::cout << "Pointcloud saved in BINARY COMPRESSED format to " << filename << ".pcd" << std::endl;
      break;
    case ASCII:
      writer_.writeASCII<PointT> (filename + ".pcd", cloud);
      // std::cout << "Pointcloud saved in ASCII format to " << filename << ".pcd" << std::endl;
      break;
  }
}


#endif /* DATA_CAPTURE_HPP_ */
