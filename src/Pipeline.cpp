#include "opencv2/opencv.hpp"
using namespace cv;

#include "Pipeline.hpp"
#include "Associativity.hpp"
#include "util.hpp"

float _intrinsic_array[9]= {524,0,316.7,0,524,238.5,0,0,1};
float _distcoeff_array[4] = {0.2402,-0.6861,-0.0015,0.0003};

Pipeline::Pipeline(std::string _folder_path)
:
	cameraMatrix(Mat(3, 3, CV_32F, _intrinsic_array)),
	distCoeffs(Mat(1, 4, CV_32F, _distcoeff_array))
{
	folder_path = _folder_path;
}

void Pipeline::run()
{
	Logger _log("Pipeline");
	/**
	 * Stage 0: Load images from file
	 */
	Images images;
	load_images(folder_path, images);


	/**
	 * Stage 1: Detect features in loaded images
	 */
	CamFrames cam_Frames;
	DescriptorsVec descriptors_vec;
	extract_features(images, cam_Frames, descriptors_vec);


	/**
	 * Stage 2: Calculate descriptors and find image pairs through matching
	 */
	ImagePairs image_pairs;
	find_matching_pairs(images, cam_Frames, descriptors_vec, image_pairs);


	// Free some memory
	//Images().swap(images);
	DescriptorsVec().swap(descriptors_vec);



	/**
	* State 3: Compute pairwise R and t
	*/
	register_camera(image_pairs, cam_Frames);

	/**
	 * Stage 4: Construct associativity matrix and spanning tree
	 */
	Associativity assocMat(cam_Frames.size());
	for (int p = 0; p < image_pairs.size(); p++)
	{
		ImagePair* pair = &image_pairs[p];
		int i = pair->pair_index.first,
		    j = pair->pair_index.second;
		if(pair -> R.empty()) continue;
		assocMat(i, j) = pair;
		assocMat(j, i) = pair;

		assert(assocMat(i, j) == assocMat(j, i));
		assert(assocMat(i, j)->pair_index.first  == i &&
		       assocMat(i, j)->pair_index.second == j);
	}

	Associativity tree;
	build_spanning_tree(image_pairs, assocMat, tree);


	/**
	 * Stage 5: Compute global Rs and ts
	 */
	CameraPoses gCameraPoses;
	glo_cam_poses(images, gCameraPoses, image_pairs, tree);

	/**
	 * Stage 6: Find and cluster depth points from local camera frame to global camera frame
	 */ 
	PointClusters pointClusters;
	PointMap pointMap;
	find_clusters(assocMat,gCameraPoses,cam_Frames,pointClusters,pointMap);

	/**
	 * Stage 7: get center of mass from clusters
	 */ 
	PointCloud pointCloud(pointClusters.size());
	// find_CoM(pointClusters, image_pairs, pointCloud);
	find_CoM(pointClusters,images,cam_Frames,pointMap, pointCloud);
	// End
	_log.tok();
}
