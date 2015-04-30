/*
 * data_capture.cpp
 *
 *  Created on: March 7, 2012
 *      Author: Bernhard Zeisl
 *              Computer Vision and Geometry Group, ETH Zurich
 *              zeislb@inf.ethz.ch
 *     Version: 1.0
 */

#include "data_capture.h"
#include "data_capture.hpp"

#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/common/file_io.h>

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace pc = pcl::console;

///////////////////////////////////////////////////////////////////////////////////////////////////
void
DataCapture::rgb_callback (const openni_wrapper::Image::Ptr &image)
{
  evaluateFrameRate ();

  if (continuous_capture_)
  {
    std::string filename = generateFilename ();
    saveRgb (filename, *image);
  }
  else if (save_)
  {
    save_ = false;
    std::string filename = generateFilename ();
    saveRgb (filename, *image);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void
DataCapture::depth_callback (const openni_wrapper::DepthImage::Ptr &depthmap)
{
  evaluateFrameRate ();

  if (continuous_capture_)
  {
    std::string filename = generateFilename ();
    saveDepthmap (filename, *depthmap, 16);
  }
  else if (save_)
  {
    save_ = false;
    std::string filename = generateFilename ();
    saveDepthmap (filename, *depthmap, 16);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void
DataCapture::depthrgb_callback (const openni_wrapper::Image::Ptr &image, const openni_wrapper::DepthImage::Ptr &depthmap, float constant)
{
  evaluateFrameRate ();

  // save depth map with 16 bit, as provided by the OpenNI driver
  if (continuous_capture_)
  {
    std::string filename = generateFilename ();
    saveRgb (filename, *image);
    saveDepthmap (filename, *depthmap, 16);
  }
  else if (save_)
  {
    save_ = false;
    std::string filename = generateFilename ();
    saveRgb (filename, *image);
    saveDepthmap (filename, *depthmap, 16);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void
DataCapture::evaluateFrameRate () const
{
  static unsigned count = 0;
  static double last = pcl::getTime ();

  // print out the frame rate every 30 frames
  if (++count == 30)
  {
    double now = pcl::getTime ();
    std::cout << "Average framerate: " << double(count)/double(now - last) << " Hz" <<  std::endl;
    count = 0;
    last = now;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
std::string
DataCapture::generateFilename () const
{
  // output directory
  std::string output_dir;
  if (output_dir.empty ())
    output_dir = ".";

  std::stringstream ss;

  // us timestamp as filename, if not specified
  if (filename_.empty())
  {
    ss << output_dir << "/frame_"
       << boost::posix_time::to_iso_string (boost::posix_time::microsec_clock::local_time ());
  }
  // add counter state at end of specified filename, if constantly recordings
  else if (continuous_capture_)
  {
    static unsigned int counter = 0;
    char buffer [10];
    sprintf (buffer, "%04d", counter);
    ss << output_dir << "/" << filename_ << "_cont_" << std::string (buffer);
    ++counter;
  }
  // use specified filename
  else
  {
	static unsigned int counter = 0;
    char buffer [10];
    sprintf (buffer, "%04d", counter);
    ss << output_dir << "/" << filename_ << "_snap_" << std::string (buffer);
    ++counter;
  }

  return ss.str();
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void
DataCapture::saveRgb (const std::string &filename, const openni_wrapper::Image &image) const
{
  // get the meta data associated with the image
  xn::ImageMetaData& md = const_cast<xn::ImageMetaData&> (image.getMetaData());

/*  cout << "image data:" << endl;
	cout << " - size        = " << image.getWidth() << "x" << image.getHeight() << endl;
	cout << " - size (md)   = " << md.FullXRes() << "x" << md.FullYRes() << endl;
	cout << " - encoding    = " << image.getEncoding() << endl;
	cout << " - bytes/pixel = " << md.BytesPerPixel() << endl;
	cout << " - data size   = " << md.DataSize() << endl;
	cout << " - pixel format (enum) = " << md.PixelFormat() <<  endl;
*/

  // copy the rgb values in a buffer
  int width = image.getWidth(), height = image.getHeight();
  unsigned char* buffer = new unsigned char[width*height*3];
  image.fillRGB (width, height, buffer, width*3);

  cv::Mat img (height, width, CV_8UC3);

  // openCV saves color values in order BGR, so rearrange
  int idx = 0;
  for (int r = 0; r < height; ++r) {
    for (int c = 0; c < width; ++c, idx += 3) {
      img.data[idx+2] = buffer[idx];
      img.data[idx+1] = buffer[idx+1];
      img.data[idx]   = buffer[idx+2];
    }
  }

  // save as PNG image
  cv::imwrite(filename + "_rgb.png", img);

  // std::cout << "RGB image saved as PNG to " << filename << "_rgb.png" << std::endl;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void
DataCapture::saveDepthmap (const std::string &filename, const openni_wrapper::DepthImage &depthmap, unsigned char depth_res) const
{
  // get meta data associated with the depht image
  xn::DepthMetaData& md = const_cast<xn::DepthMetaData&> (depthmap.getDepthMetaData());

/*  cout << "depth image data:" << endl;
	cout << " - focal length = " << depthmap.getFocalLength() << endl;
	cout << " - base line    = " << depthmap.getBaseline() << endl;
	cout << " - depth res.   = " << md.ZRes() << endl;
*/

  // copy depth values into a buffer
  int width = depthmap.getWidth(), height = depthmap.getHeight();
  unsigned short* buffer = new unsigned short[width*height];
  depthmap.fillDepthImageRaw (width, height, buffer);
  cv::Mat_<unsigned short> mat16bit (height, width, buffer);

  // scale according to the depth resolution provided by the sensor
  double scale = (std::pow(2.0, depth_res) - 1.0) / md.ZRes();

  switch (depth_res)
  {
    case 8: {
      // convert from 16 to 8 bit per depth value
      cv::Mat mat8bit (height, width, CV_8U);
      mat16bit.convertTo ( mat8bit, CV_8U, scale);
      cv::imwrite (filename + "_depth.png", mat8bit);
      break;
    }
    case 16: {
      mat16bit *= scale;
      cv::imwrite (filename + "_depth.png", mat16bit);
      break;
    }
    default: {
      std::cerr << "[saveDepthmap] Depth size of " << depth_res << " not supported (use 8 or 16)" << std::endl;
      break;
    }
  }

  // std::cout << "Depth image saved as PNG (" << (int)depth_res << ") to " << filename << "_depth.png" << std::endl;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void
DataCapture::keyboard_callback (const pcl::visualization::KeyboardEvent &event, void *cookie)
{
  DataCapture* capture = reinterpret_cast<DataCapture*> (cookie);

  if (event.keyDown())
  {
    switch (event.getKeyCode())
    {
      case 27:
      case (int)'q': case (int)'Q':
      case (int)'e': case (int)'E':
        // stop the grabber
        capture->quit_ = true;
        pc::print_info ("Exiting program.\n");
        break;

      case (int)'s': case (int)'S':
        // capture snapshot (if not currently capturing continuously)
        if (!capture->continuous_capture_)
        {
          capture->save_ = true;
          pc::print_highlight ("Recording snapshot.\n");
        }
        break;

      case (int)'t': case (int)'T':
          // start / stop continuous capture
          capture->continuous_capture_ = !capture->continuous_capture_;
          pc::print_highlight ("%s continuous recording of data.\n", (capture->continuous_capture_) ? "Starting" : "Stopping");
          break;

      default:
        break;
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void
DataCapture::setOptions (DataCapture::CAPTURE_METHOD capture_method, DataCapture::PCD_FILE_FORMAT pcd_format, const std::string &filename)
{
  if (!filename.empty())
    filename_ = pcl::getFilenameWithoutExtension (filename);

  capture_method_ = capture_method;
  format_ = pcd_format;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void
DataCapture::run ()
{
  /***
   * Register the appropriate callback functions
   ***/

  // capture RGB point cloud in any instance for visualization
  boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> fcr = boost::bind (&DataCapture::cloud_callback<pcl::PointXYZRGBA>, this, _1);
  interface_.registerCallback (fcr);

  // register remaining callbacks if something else than the RGB cloud should be recorded
  switch (capture_method_)
  {
    case CLOUD: {
      boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> fc = boost::bind (&DataCapture::cloud_callback<pcl::PointXYZ>, this, _1);
      interface_.registerCallback (fc);
      break;
    }
    case RGB: {
      boost::function<void (const openni_wrapper::Image::Ptr&)> fr = boost::bind (&DataCapture::rgb_callback, this, _1);
      interface_.registerCallback (fr);
      break;
    }
    case DEPTH: {
      boost::function<void (const openni_wrapper::DepthImage::Ptr&)> fd = boost::bind (&DataCapture::depth_callback, this, _1);
      interface_.registerCallback (fd);
      break;
    }
    case DEPTH_RGB: {
      boost::function<void (const openni_wrapper::Image::Ptr&, const openni_wrapper::DepthImage::Ptr&, float)> fdr = boost::bind (&DataCapture::depthrgb_callback, this, _1, _2, _3);
      interface_.registerCallback (fdr);
      break;
    }
  }

  /***
   * Start the capturing process
   ***/

  pc::print_highlight ("Starting to receive data ...\n");

  interface_.start ();

  while (!quit_ && !cloud_viewer_.visualizer_.wasStopped())
  {
    cloud_viewer_.show();
    cloud_viewer_.visualizer_.spinOnce();
    boost::this_thread::sleep (boost::posix_time::milliseconds (10));
  }

  // stop receiving data
  interface_.stop();
}



///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
void
usage (char ** argv)
{
  std::cout << "usage: " << argv[0] << " [<options>]\n\n";

  pc::print_info ("  where options are:\n");
  pc::print_info ("          -method   = type of data to capture (0 = cloud; 1 = RGB cloud, 2 = RGB image,\n");
  pc::print_info ("                                             3 = depth map, 4 = depth map + RGB image (synced)) (default: ");
  pc::print_value ("%d", (int)DataCapture::default_capture_method_);
  pc::print_info (")\n");
  pc::print_info ("                      Note: The depthmap is always registered to the RGB camera\n");
  pc::print_info ("          -format   = PCD file format (0=binary; 1=binary compressed; 2=acsii) (default: ");
  pc::print_value ("%d", (int)DataCapture::default_pcd_format_);
  pc::print_info (")\n");
  pc::print_info ("          -filename = filename (prefix) for captured data (default: generic timestamp)\n\n");
  pc::print_info ("  During runtime (visualization needs focus) use:\n");
  pc::print_info ("          t         = start or stop continuous data capture\n");
  pc::print_info ("          s         = record a single frame\n");
  pc::print_info ("          e/q/Esc   = exit the program\n\n");
}

///////////////////////////////////////////////////////////////////////////////////////////////////
int
main (int argc, char** argv)
{
  pc::print_info ("\nRecords a point clouds, images or depth maps from an OpenNI device such as Kinect.\nFor more information, use: %s --help\n\n", argv[0]);

  /***
   * Parse command line
   ***/

  // print help usage information
  if (pc::find_switch (argc, argv, "--help") || pc::find_switch (argc, argv, "-h"))
  {
    usage (argv);
    exit (EXIT_SUCCESS);
  }
  
  pc::print_info ("  During runtime (visualization needs focus) use:\n");
  pc::print_info ("          t       = start / stop continuous data capture\n");
  pc::print_info ("          s       = record a single frame\n");
  pc::print_info ("          e/q/Esc = exit the program\n\n");

  // determine capture method
  int md;
  DataCapture::CAPTURE_METHOD method;
  if (pc::parse_argument (argc, argv, "-method", md) > 0)
    method = (DataCapture::CAPTURE_METHOD) md;
  else
    method = DataCapture::default_capture_method_;

  // file format for point clouds
  int ff;
  DataCapture::PCD_FILE_FORMAT format;
  if (pc::parse_argument (argc, argv, "-format", ff) > 0)
    format = (DataCapture::PCD_FILE_FORMAT) ff;
  else
    format = DataCapture::default_pcd_format_;

  // file name for storing
  std::string filename;
  pc::parse_argument (argc, argv, "-filename", filename);

  /***
   * Capturing data
   */

  DataCapture capture;
  capture.setOptions (method, format, filename);
  capture.run ();

  return EXIT_SUCCESS;
}
