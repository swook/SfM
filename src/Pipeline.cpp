#include "opencv2/opencv.hpp"

#include <boost/date_time/posix_time/posix_time.hpp>
namespace ptime = boost::posix_time;

#include "boost/filesystem.hpp"
namespace fs  = boost::filesystem;

#include <boost/format.hpp>
typedef boost::format fmt;

#include "Pipeline.hpp"
#include "Associativity.hpp"
#include "Viewer.hpp"
#include "util.hpp"

float _intrinsic_array[9]= {524,0,316.7,0,524,238.5,0,0,1};
float _distcoeff_array[4] = {0.2402,-0.6861,-0.0015,0.0003};

Pipeline::Pipeline(std::string _folder_path)
:
	cameraMatrix(cv::Mat(3, 3, CV_32F, _intrinsic_array)),
	distCoeffs(cv::Mat(1, 4, CV_32F, _distcoeff_array))
{
	int n = _folder_path.size();
	if (_folder_path[n - 1] == '/') _folder_path.resize(n - 1);

	folder_path = _folder_path;
}

void Pipeline::run(const bool save_clouds, const bool show_clouds)
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

	// Free Image.gray
	for (int i = 0; i < images.size(); i++)
		const_cast<cv::Mat&>(images[i].gray).release();


	/**
	 * Stage 2: Calculate descriptors and find image pairs through matching
	 */
	ImagePairs image_pairs;
	find_matching_pairs(images, cam_Frames, descriptors_vec, image_pairs);


	// Free some memory
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

		if (pair -> R.empty()) continue;

		assocMat(i, j) = pair;
		assocMat(j, i) = pair;

		assert(assocMat(i, j) == assocMat(j, i));
		assert(assocMat(i, j)->pair_index.first  == i &&
		       assocMat(i, j)->pair_index.second == j);
	}

	Associativity tree;
	const int camera_num = build_spanning_tree(image_pairs, assocMat, tree);


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
	find_clusters(assocMat, gCameraPoses, cam_Frames, pointClusters, pointMap);

	// Free some memory
	ImagePairs().swap(image_pairs);


	/**
	 * Stage 7: get center of mass from clusters
	 */
	PointCloud pointCloud(pointClusters.size());
	find_CoM(pointClusters, pointCloud);

	// Free pointClusters
	for (int i = 0; i < pointClusters.size(); i++)
		PointCluster().swap(pointClusters[i]);
	PointClusters().swap(pointClusters);


	// Save cloud before BA
	Viewer viewer("Before BA");
	auto cloud  = viewer.createPointCloud(images, gCameraPoses, cameraMatrix);
	int  n      = cloud->points.size();
	auto time   = ptime::second_clock::local_time();
	auto tstamp = ptime::to_iso_string(time);
	auto folder = fs::path(folder_path).filename().string();
	auto fname  = (fmt("%s_%s_%d_noBA.pcd") % folder % tstamp % n).str().c_str();

	if (save_clouds)
		viewer.saveCloud(cloud, fname);
	if (show_clouds)
		viewer.showCloudPoints(cloud, false);


	/**
	 * State 8: Bundle Adjustment
	 */
	bundle_adjustment(pointMap, cam_Frames, false, gCameraPoses, pointCloud);
	_log.tok();

	/**
	 * Show calculated point cloud
	 */
	Viewer viewer_ba("After BA no Depth");
	cloud = viewer_ba.createPointCloud(images, gCameraPoses, cameraMatrix);
	n     = cloud->points.size();
	fname = (fmt("%s_%s_%d_BA_noD.pcd") % folder % tstamp % n).str().c_str();


	if (save_clouds)
		viewer_ba.saveCloud(cloud, fname);
	if (show_clouds)
		viewer_ba.showCloudPoints(cloud,false);

	bundle_adjustment(pointMap, cam_Frames, true, gCameraPoses, pointCloud);

	// Free some memory
	PointMap().swap(pointMap);
	CamFrames().swap(cam_Frames);
	PointCloud().swap(pointCloud);

	/**
	 * Show calculated point cloud
	 */
	Viewer viewer_baD("After BA with Depth");
	cloud = viewer_baD.createPointCloud(images, gCameraPoses, cameraMatrix);
	n     = cloud->points.size();
	fname = (fmt("%s_%s_%d_BA_D.pcd") % folder % tstamp % n).str().c_str();

	// Free some memory
	Images().swap(images);
	CameraPoses().swap(gCameraPoses);

	if (save_clouds)
		viewer_baD.saveCloud(cloud, fname);
	if (show_clouds)
		viewer_baD.showCloudPoints(cloud);



}
