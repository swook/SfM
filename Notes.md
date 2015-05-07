PIPELINE
========

# Feature detection

Detect SIFT/SURF features using [OpenCV][opencv-sift-example].

## Inputs

* Image (MxN)

## Outputs

* Vector of descriptors

## Code Example

For each image

* detect keypoint `SiftFeatureDetector` or `SurfFeatureDetector`
    SurfFeatureDetector detector(400);
    vector<KeyPoint> keypoints1;
    detector.detect(img1, keypoints1);
* compute descriptors from keypoints `SurfDescriptorExtractor`
    SurfDescriptorExtractor extractor;
    Mat descriptors1;
    extractor.compute(img1, keypoints1, descriptors1);

# Feature matching

Feature matching

## Inputs

* N descriptors

## Outputs

* K matches

## Code Example

For image k, pair (k,j) j = k+1 to N

* match using `FlannBasedMatcher`
    FlannBasedMatcher matcher;
    std::vector< DMatch > matches;
    matcher.match( descriptors_k, descriptors_j, matches );

* get match pairs (i,j) if matches.size()>threshold

# Find camera-pairs

# Pairwise camera registration
consider choosing one/several of the following three methods and take average.
average rotation matrix by averaging their quaternions.
Need camera parameters [kinect-calib]

## 2D-2D camera registration

See [cv::findFundamentalMat][findFundamentalMat].

### Inputs

* Keypoint coordinates in 2D (of camera i)
* Keypoint coordinates in 2D (of camera j)

### Outputs

* R: rotation vector i-to-j
* t: translation vector i-to-j

## 2D-3D camera registration RANSAC and PnP

See [cv::solvePnPRansac][solvePnPRansac].

### Inputs

Per camera-pair

* Camera parameters: focal length, [principal point][wiki-cam-intrins],
distortion coefficients (?)
* Keypoints in 2D (of camera i)
* Keypoints in 3D (of camera j)
* Optionally, R & t

### Outputs

* R: rotation vector i-to-j
* t: translation vector i-to-j

## 3D-3D camera registration ICP


# Convert to global (single) coordinate frame

Convert i-to-j vectors to be w.r.t single coordinate frame.

Minimum spanning tree?

## Inputs

## Outputs



# Bundle Adjustment

[Ceres][ceres-api] is used for bundle adjustment. The aim is to optimise global
camera pose and feature pose estimates.

We need to define a `CostFunctor`, pick a `LinearSolver`.

It is possible to define a `NumericalDiffCostFunction` or `AutoDiffCostFunction`.

See the [basic tutorial](http://ceres-solver.org/nnls_tutorial.html#hello-world)
for more information. In particular, see the [Bundle-Adjustment section][ceres-BA]

## Inputs

Per camera-pair there are n keypoints and associated values are:

* nx3 for keypoints
* 3 for rotation
* 3 for translation
* [2 for radial distortion]
* [1 for focal length]

## Outputs

Same as inputs



# Visualisation

PCL

## Inputs

* A

## Outputs

* B





<!-- Reference URLs -->
[vlfeat-sift]: http://www.vlfeat.org/api/sift.html
[vlfeat-docs]: http://www.vlfeat.org/api/index.html

[opencv-sift-example]: http://docs.opencv.org/doc/user_guide/ug_features2d.html

[findFundamentalMat]: http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#findfundamentalmat
[solvePnPRansac]: http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#solvepnpransac

[ceres-api]: http://ceres-solver.org/api.html
[ceres-BA]: http://ceres-solver.org/nnls_tutorial.html#bundle-adjustment

[wiki-cam-intrins]: https://en.wikipedia.org/wiki/Camera_resectioning#Intrinsic_parameters
[kinect-calib] http://cmp.felk.cvut.cz/ftp/articles/pajdla/Smisek-CDC4CV-2011.pdf
