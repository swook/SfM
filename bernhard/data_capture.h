/*
 * data_capture.h
 *
 *  Created on: March 7, 2012
 *      Author: Bernhard Zeisl
 * 				Computer Vision and Geometry Group, ETH Zurich
 *              zeislb@inf.ethz.ch
 *     Version: 1.0
 */

#ifndef DATA_CAPTURE_H_
#define DATA_CAPTURE_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>

///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
/* \brief Class for visualizing a point cloud */
template <typename PointT>
class FrameCloudView
{
public:

  /* \brief Constructor initializing the PCLVisualizer */
  FrameCloudView (const std::string &cloud_id)
    : visualizer_ ("Frame Cloud Viewer"),
      cloud_id_ (cloud_id),
      cloud_ ()
  {
    visualizer_.setBackgroundColor (0, 0, 0.15);
    visualizer_.addCoordinateSystem (1.0);
    visualizer_.initCameraParameters ();
    visualizer_.setCameraPosition (0.0, 0.0, -5.0,   // position
                                   0.0, -1.0, 0.0,   // view
                                   0);               // viewport
  }

  /* \brief Visualize the given point cloud */
  void
  show ();

  void
  setCloud (const typename pcl::PointCloud<PointT>::ConstPtr &cloud)
  {
    boost::mutex::scoped_lock lock (cld_mutex_);
    cloud_ = cloud;
  }

  /* member variables */
  pcl::visualization::PCLVisualizer visualizer_;

  typename pcl::PointCloud<PointT>::ConstPtr cloud_;
  std::string cloud_id_;
  boost::mutex cld_mutex_;
};

///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
/* \brief Class for capturing data from an OpenNI device, such as MS Kinect */
class DataCapture
{
public:

  /* \brief Constructor for data capture */
  DataCapture ()
    : cloud_viewer_ ("rgbd_cloud"),
      capture_method_ (default_capture_method_),
      save_ (false),
      continuous_capture_ (false),
      quit_ (false),
      format_ (default_pcd_format_)
  {
    cloud_viewer_.visualizer_.registerKeyboardCallback (keyboard_callback, (void*)this);
  };

  /* \brief Possible modes for the capturing process */
  enum CAPTURE_METHOD {
    CLOUD = 0,
    CLOUD_RGB,
    RGB,
    DEPTH,
    DEPTH_RGB
  };

  /* \brief Possible formats for saving poind clouds */
  enum PCD_FILE_FORMAT {
    BINARY = 0,
    BINARY_COMPRESSED,
    ASCII
  };

  /* \brief Default parameters */
  static const CAPTURE_METHOD  default_capture_method_    = CLOUD_RGB;
  static const PCD_FILE_FORMAT default_pcd_format_        = BINARY_COMPRESSED;

  /* \brief Callback method for point cloud capturing (both with and without color contained)
   * \param[out] cloud Reference to a point cloud structure where the captured data should be stored
   */
  template <typename PointT> void
  cloud_callback (const typename pcl::PointCloud<PointT>::ConstPtr &cloud);

  /* \brief Callback method for RGB image capturing
   * \param[out] image Reference to an image structure for storing the current RGB image
   */
  void
  rgb_callback (const openni_wrapper::Image::Ptr &image);

  /* \brief Callback method for depth map capturing
   * \param[out] depthmap Reference to a depth-map structure for storing the current depth data
   */
  void
  depth_callback (const openni_wrapper::DepthImage::Ptr &depthmap);

  /* \brief Callback method for RGB image and depth map capturing
   * \param[out] image Reference to an image structure for storing the current RGB image
   * \param[out] depthmap Reference to a depth-map structure for storing the current depth data
   * \param[in] constant ...
   * \note RGB and depth data streams are not synchronized in Kinect. Streams are synchronized, such that RGB image and depth map
   *       are returned once both are available. However, this doesn't guarantee a real synchronization!
   */
  void
  depthrgb_callback (const openni_wrapper::Image::Ptr &image, const openni_wrapper::DepthImage::Ptr &depthmap, float constant);

  /* \brief Set options for the following data capture
   * \param[in] filename Filename (prefix) for files
   * \param[in] cpature_method Capture method as defined in $enum CAPTURE_METHOD$
   * \param[in] pcd_format File format for point cloud storing ("bc" for binary compressed, "b" for binary, "ascii" for ASCII file)
   * \param[in] loop Single frame or continous capturing
   */
  void
  setOptions (CAPTURE_METHOD capture_method = default_capture_method_, PCD_FILE_FORMAT pcd_format = default_pcd_format_, const std::string &filename = "");

  /* \brief Executes the capturing; Options need to be set beforehand
   */
  void
  run ();

///////////////////////////////////////////////////////////////////////////////////////////////////
private:

  /* \brief Callback method handling keyboard events in the PCLVisualizer */
  static void
  keyboard_callback (const pcl::visualization::KeyboardEvent &event, void *cookie);

  /* \brief Provides a filename for the current capture */
  std::string
  generateFilename () const;

  /* \brief Stores the point cloud in the previously specified format */
  template <typename PointT> void
  saveCloud (const std::string &filename, const pcl::PointCloud<PointT> &cloud);

  /* \brief Saves the RGB image */
  void
  saveRgb (const std::string &filename, const openni_wrapper::Image &image) const;

  /* \brief Saves the depth map */
  void
  saveDepthmap (const std::string &filename, const openni_wrapper::DepthImage &depthmap, unsigned char depthRes = 16) const;

  /* Calculates and prints out the current recording frame rate */
  void
  evaluateFrameRate () const;

  /* member variables */
  pcl::OpenNIGrabber interface_;
  pcl::PCDWriter writer_;
  CAPTURE_METHOD capture_method_;

  FrameCloudView<pcl::PointXYZRGBA> cloud_viewer_;
  std::string cloud_id_;

  bool save_;
  bool continuous_capture_;
  bool quit_;

  std::string filename_;
  PCD_FILE_FORMAT format_;
};


#endif /* DATA_CAPTURE_H_ */
