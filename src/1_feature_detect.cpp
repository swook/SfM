#include "opencv2/opencv.hpp"

#include "Pipeline.hpp"

using namespace cv;

/*
std::vector<Mat>& Pipeline::load_images(std::string folder_path)
{
	/**
	 * Extract SIFT features from RGB images
	 *
	auto filt = vl_sift_new(640, 480, -1, 3, 0);
	int noctave;
	for (int i = 0; i < rgbf_list.size(); i++)
	{
		path = rgbf_list[i];

		/*
		 * Read image file
		 *
		Mat const img = imread(path, CV_LOAD_IMAGE_COLOR);
		Mat gray;
		cvtColor(img, gray, CV_BGR2GRAY);
		//showImageAndWait("Input Image", gray);

		// Convert to column-major for vlfeat
		std::vector<float> imgvec;
		for (int i = 0; i < img.cols; i++)
			for (int j = 0; j < img.rows; j++)
				imgvec.push_back((float)img.ptr<uchar>(j)[i] / 255.f);

		int n = 0;
		for (int o = 0; o < vl_sift_get_noctaves(filt); ++o) {
			if (o == 0)
				vl_sift_process_first_octave(filt, imgvec.data());
			else
				vl_sift_process_next_octave(filt);

			vl_sift_detect(filt);
			vl_sift_get_keypoints(filt);
			n += vl_sift_get_nkeypoints(filt);
		}

		std::cout << n << " features." << std::endl;
	}
	vl_sift_delete(filt);
}*/
