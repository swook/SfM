#include "opencv2/core.hpp"
using namespace cv;

#include "Pipeline.hpp"
#include "Associativity.hpp"
#include "util.hpp"

void Pipeline::find_clusters(Associativity& assocMat, const CameraPoses& cameraPoses,
	const CamFrames& camFrames, PointClusters& pointClusters, PointMap& pointMap)
{

	Logger _log("Step 6 (clusters)");


	// function when walking through the pair tree
	assocMat.walk([&cameraPoses,&pointClusters, &pointMap,&_log,this](const int i, const int j, const ImagePair* pair) -> bool
	{
		Depths depths_i, depths_j;							//depth values of keypoints in current camera
		std::vector<Point2f> keypoints_i,keypoints_j;		//matched keypoints in current camera
		std::vector<int> kpIdx_i,kpIdx_j;					//index of matched keypoings in the other camera in pair

		// Find index of other side of pair
		if (pair->pair_index.first ==j){
			kpIdx_i = pair -> matched_indices.second;
			keypoints_i = pair -> matched_points.second;
			depths_i = pair -> pair_depths.second;

			kpIdx_j = pair -> matched_indices.first;
			depths_j = pair -> pair_depths.first;
			keypoints_j = pair -> matched_points.first;
		}else{
			kpIdx_i = pair -> matched_indices.first;
			keypoints_i = pair -> matched_points.first;
			depths_i = pair -> pair_depths.first;

			kpIdx_j = pair -> matched_indices.second;
			depths_j = pair -> pair_depths.second;
			keypoints_j = pair -> matched_points.second;
		}

		// get R and t of each camera in this pair
		Mat R_i = cameraPoses[i].R;
		Mat t_i = cameraPoses[i].t;
		Mat R_j = cameraPoses[j].R;
		Mat t_j = cameraPoses[j].t;

		if (R_i.empty()||R_j.empty()) return true;

		// for all matching in camera j, get their global coordinate
		// and insert them in corresponding cluster
		for(size_t m = 0; m < keypoints_j.size(); m++)
		{
			// back project keypoints in camera j into local 3D frame
			float x_j = keypoints_j[m].x;
			float y_j = keypoints_j[m].y;
			Point3f point3D_j = backproject3D(x_j,y_j,depths_j[m],cameraMatrix);

			// transform into global frame
			Mat gPoint3D_j_tmp = R_j.t()*Mat(point3D_j) - t_j;
			Point3f gPoint3D_j = Point3f(gPoint3D_j_tmp);

			// std:: cout << "cam idx j"<<std::endl << j <<std::endl;
			// std:: cout << "R_j "<<std::endl << R_j <<std::endl;
			// std:: cout << "t_j "<<std::endl << t_j <<std::endl;
			// std:: cout << "local 3D j"<<std::endl << Mat(point3D_j)<<std::endl;
			// std:: cout << "global 3D j"<<std::endl<< gPoint3D_j <<std::endl;

			// find matching global point from pointMap
			std::pair<int,int> key(i, kpIdx_i[m]);
			int p3D_idx;
			PointMap::const_iterator entry = pointMap.find(key);
			if (entry == pointMap.end()) {
				// if matching keypoint in camera i was not added to any cluster yet
				float x_i = keypoints_i[m].x;
				float y_i = keypoints_i[m].y;

				Point3f point3D_i = backproject3D(x_i,y_i,depths_i[m],cameraMatrix);

				// transform into global frame
				Mat gPoint3D_i_tmp = R_i.t()*Mat(point3D_i) - t_i;
				Point3f gPoint3D_i = Point3f(gPoint3D_i_tmp);

				// std:: cout << "cam idx i"<<std::endl << i <<std::endl;
				// std:: cout << "R_i "<<std::endl << R_i <<std::endl;
				// std:: cout << "t_i "<<std::endl << t_i <<std::endl;
				// std:: cout << "local 3D i"<<std::endl << Mat(point3D_i)<<std::endl;
				// std:: cout << "global 3D i"<<std::endl<< gPoint3D_i <<std::endl;

				// add new entry in pointMap for camera i
				p3D_idx = pointClusters.size();
				pointMap[key] = p3D_idx;

				// add 3D point in i into a new cluster
				pointClusters.push_back(std::vector<Point3f> {gPoint3D_i});
			}else{
				// if matching keypoint in camera j was added to a cluster already
				p3D_idx = entry -> second;
			}
			// add current keypoint in camera j in pointmap
			key.first = j;
			key.second = kpIdx_j[m];
			pointMap[key] = p3D_idx;

			// add 3D point in j into the cluster
			pointClusters[p3D_idx].push_back(gPoint3D_j);
		}
		return true;
	});
	// for (int i = 0;i!=pointClusters.size();i++)
	// {
	// 	std::vector<Point3f> cluster;
	// 	std::cout << "cluster " << i << std::endl;
	// 	cluster = pointClusters[i];
	// 	for (int j =0; j < cluster.size(); j++)
	// 		std::cout << cluster[j] << ',';
	// 	std::cout << std::endl;
	// }
	_log.tok();
}
