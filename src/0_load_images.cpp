#include <map>
#include <regex>
using namespace std;

#include "boost/filesystem.hpp"
#include "boost/format.hpp"
namespace fs  = boost::filesystem;
typedef boost::format fmt;

#include "opencv2/opencv.hpp"
#include "opencv2/xphoto.hpp"
using namespace cv;

#include "Pipeline.hpp"

void Pipeline::load_images(string folder_path, Images& images)
{
	// Get list of image files from dataset path
	if (!fs::exists(folder_path) || !fs::is_directory(folder_path)) {
		throw runtime_error("Invalid data path: " + folder_path);
	}

	// Reset images list
	images = Images(0);

	// Regex to parse image names
	auto fname_regex = regex(".*/frame_([0-9T\\.]+)_(rgb|depth)\\.png$");

	// Create intermediate map structure
	map<string, bool> tstamps;
	fs::directory_iterator end_itr;
	string path, time_str;
	smatch match;
	for (fs::directory_iterator itr(folder_path); itr != end_itr; itr++)
	{
		if (fs::is_regular_file(itr->status()))
		{
			path = itr->path().string();
			regex_match(path, match, fname_regex);

			if (match.size() == 3)
			{
				/**
				 * 0: ../data/frame_20150312T172452.627702_depth.png
				 * 1: 20150312T172452.627702
				 * 2: depth
				 */
				time_str = match[1];
				tstamps[time_str] = true;
			}
		}
	}

	string rgb_path, dep_path;
	int i = 0;
	for (auto it = tstamps.begin(); it != tstamps.end(); it++)
	{
		// Get time string
		time_str = it->first;

		// Get file paths
		rgb_path = (fmt("%s/frame_%s_rgb.png")   % folder_path % time_str).str();
		dep_path = (fmt("%s/frame_%s_depth.png") % folder_path % time_str).str();
		Mat rgb_img = imread(rgb_path);
		Mat depth_img = imread(dep_path);

		// Some preprocessing
		Mat gray_img,depth_undist_img,depth_smooth_img;
		cvtColor(rgb_img,gray_img,COLOR_RGB2GRAY);
		balanceWhite(rgb_img, rgb_img, xphoto::WHITE_BALANCE_SIMPLE);
		undistort(gray_img,depth_undist_img, cameraMatrix, distCoeffs);
		depth_undist_img.convertTo(depth_img,CV_32F);
		bilateralFilter(depth_img,depth_smooth_img,3,10,10);
		depth_smooth_img.convertTo(depth_img,CV_16U);

		// Store Image struct with image read using imread
		images.push_back((Image) {
			i,
			pt::from_iso_string(time_str),
			rgb_img,
			gray_img,
			depth_img,
			rgb_path,
			dep_path
		});
		// increment index
		i++;
	}
}

