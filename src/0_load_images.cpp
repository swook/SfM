#include <map>
using namespace std;

#include "boost/filesystem.hpp"
#include "boost/format.hpp"
#include "boost/regex.hpp"
namespace fs = boost::filesystem;
typedef boost::format fmt;
using namespace boost;

#include "opencv2/opencv.hpp"
#include "opencv2/xphoto.hpp"
using namespace cv;

#include "Pipeline.hpp"
#include "util.hpp"

void Pipeline::load_images(string folder_path, Images& images)
{
	Logger _log("Step 0 (preprocess)");

	// Get list of image files from dataset path
	if (!fs::exists(folder_path) || !fs::is_directory(folder_path)) {
		throw runtime_error("Invalid data path: " + folder_path);
	}

	// Reset images list
	images = Images(0);

	// Regex to parse image names
	auto fname_regex = regex("^.*/frame_([\\dT\\.]+)_(rgb|depth)\\.png$",
		regex_constants::ECMAScript);

	// Create intermediate map structure
	map<string, bool> _tstamps;
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
				_tstamps[time_str] = true;
			}
		}
	}

	// Convert to vector of timestamps
	const int n = _tstamps.size();
	std::vector<string> tstamps;
	tstamps.reserve(n);
	for (auto tstamp: _tstamps) tstamps.push_back(tstamp.first);

	_log("Found %d timestamps.", n);

	images = Images();

	int new_n = 0;
#pragma omp parallel for
	for (int i = 0; i < n; i++)
	{
		// Get time string
		string time_str = tstamps[i];

		// Get file paths
		string rgb_path = (fmt("%s/frame_%s_rgb.png")   % folder_path % time_str).str();
		string dep_path = (fmt("%s/frame_%s_depth.png") % folder_path % time_str).str();
		Mat rgb_img = imread(rgb_path);
		Mat _depth_img = imread(dep_path,CV_LOAD_IMAGE_ANYDEPTH); // load 16bit img
		if (!rgb_img.data || !_depth_img.data) continue;

		// White-balance RGB image
		// TODO: undistort?
		Mat rgb_undist_img;
		balanceWhite(rgb_img, rgb_img, xphoto::WHITE_BALANCE_SIMPLE);
		rgb_undist_img = rgb_img;
		//undistort(rgb_img, rgb_undist_img, cameraMatrix, distCoeffs);

		// Get grayscale image
		Mat gray_undist_img;
		cvtColor(rgb_undist_img, gray_undist_img, COLOR_RGB2GRAY);

		// Convert depth map to floats
		Mat depth_img;
		_depth_img.convertTo(depth_img, CV_32F);

		// Smooth depth map
		Mat depth_smooth_img;
		//bilateralFilter(InputArray src, OutputArray dst, int d, double sigmaColor, double sigmaSpace, int borderType=BORDER_DEFAULT )
		bilateralFilter(depth_img, depth_smooth_img, 5, 10, 10);
		// FINAL RGB:   White-balance 3-channels
		// FINAL GRAY:  White-balance 1-channel
		// FINAL DEPTH: 1-channel, floats, bilateralFilter smoothing

		// Store Image struct with image read using imread
		Image image = {
			i,
			pt::from_iso_string(time_str),
			rgb_undist_img,
			gray_undist_img,
			depth_smooth_img,
			rgb_path,
			dep_path
		};
		images.push_back(image);

#pragma omp atomic
		new_n++;
	}

	images.resize(new_n);
	_log("Loaded %d RGB-D images.", new_n);

	_log.tok();
}

