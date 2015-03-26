#include <regex>

#include "boost/filesystem.hpp"
#include "boost/program_options.hpp"
namespace fs = boost::filesystem;
namespace po = boost::program_options;

extern "C" {
#include "vl/generic.h"
#include "vl/sift.h"
}

#include "opencv2/opencv.hpp"
using namespace cv;

#include "util.hpp"

int main(int argc, char** argv)
{
	/*
	 * Argument parsing
	 */
	po::options_description desc("Available options");
	desc.add_options()
	    ("help", "Show this message")
	    ("data-path,d", po::value<std::string>(), "Input dataset folder path")
	;

	po::positional_options_description p;
	p.add("data-path", -1);

	po::variables_map vm;
	po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
	po::notify(vm);

	if (vm.size() == 0 || vm.count("help") || !vm.count("data-path")) {
		std::cout << "Usage: " << argv[0]
			<< " [options] data-path" << std::endl
			<< desc;
		return 1;
	}

	/*
	 * Get list of image files from dataset path
	 */
	std::string const dpath = vm["data-path"].as<std::string>();
	if (!fs::exists(dpath) || !fs::is_directory(dpath)) {
		throw std::runtime_error("Invalid data path: " + dpath);
		return -1;
	}

	auto rgbf_regex = std::regex(".*/([^/]+_rgb\\.png)$");
	auto depf_regex = std::regex(".*/([^/]+_depth\\.png)$");

	auto rgbf_list = std::vector<std::string>(0);
	auto depf_list = std::vector<std::string>(0);

	fs::directory_iterator end_itr;
	std::string path;
	std::smatch match;
	for (fs::directory_iterator itr(dpath); itr != end_itr; ++itr)
	{
		if (fs::is_regular_file(itr->status()))
		{
			path = itr->path().string();

			std::regex_match(path, match, rgbf_regex);
			if (match.size() > 0) rgbf_list.push_back(path);

			std::regex_match(path, match, depf_regex);
			if (match.size() > 0) depf_list.push_back(path);
		}
	}


	/**
	 * Extract SIFT features from RGB images
	 */
	for (int i = 0; i < rgbf_list.size(); i++)
	{
		path = rgbf_list[i];

		/*
		 * Read image file
		 */
		Mat const img = imread(path, CV_LOAD_IMAGE_COLOR);
		showImageAndWait("Input Image", img);
	}



	/*
	 * Call retargeting methods
	 */
	//Mat out = crop(img);

	// Show output image
	//showImageAndWait("Output Image", out);

	/*
	 * Save output if necessary
	 */
	//if (vm.count("output-file")) {
	//	Mat out_norm;
	//	normalize(out, out_norm, 0.f, 255.f, NORM_MINMAX);
	//	imwrite(vm["output-file"].as<std::string>(), out_norm);
	//}

	return 0;

}
