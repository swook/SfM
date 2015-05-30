Compile
=======

Compile output using `latexmk -pdf report.tex` or `pdflatex report.tex` or similar.


Introduction
============

Structure from Motion (SfM) concerns the recreation of a real environment
through the inferring of motion from images. (1) First, a 3D object or
environment is imaged from multiple different perspectives. (2) In each image,
it is possible to find keypoints via feature detection algorithms such as SIFT
and SURF, and calculate descriptors with which correspondences can be found in
other images. (3) After finding corresponding pairs, the relative pose of each
camera can be inferred in various ways.

With early implementations of SfM, keypoints are projected into 3D space and
optimisation algorithms such as RANSAC and Levenberg-Marquardt are used to find
the homography or perspective transformation between two image planes. With the
advent of low-cost commercial depth sensing cameras, it is now possible to
incorporate depth information in hopes to improve or speed this procedure up. An
example is the Microsoft Kinect, which casts and reads an infra-red pattern
which is used to calculate depth values per pixel imaged *(zhang2012microsoft)*.

For problems of corresponding 2D points (image plane) and 3D points (using depth
values), one can use PnP or Perspective-n-Point algorithms *(d2013p3p)*. A P3P
algorithm uses a minimum of 3 data points to infer the relative pose between a
pair of cameras. There are optimised PnP algorithms available, which when
coupled with RANSAC can discard outliers and estimate relative camera pose
accurately *(lepetit2009epnp)*.

Once an initial estimate for relative camera pose is made, this can be optimised
via bundle adjustment, an optimisation step operated on sparse keypoints from
the perspective of all cameras. To do so, (4) pairwise camera pose estimates
need to be transformed into a single coordinate system. This is achieved through
the construction of a minimum spanning tree with cameras as nodes and the
existence of corresponding pairs as edges. When camera pose estimates are
relative to a single coordinate system, it is possible to project keypoints into
the coordinate system from the perspective of each camera. Minimising this
reprojection is the (5) bundle adjustment step.

A final model can be constructed by projecting all known data into 3D space
using final camera pose estimates. In such a way a dense 3D reconstruction is
possible.

This study concerns the use of depth data in hopes to improve the accuracy of
the final 3D reconstruction output.



Theory
======
*NOTE: Algorithms*

*NOTE: P3P equations*


Methodology
===========

As outlined in the previous sections, our structure from motion pipeline is
composed of 5 main steps listed below.

1. Data acquisition
2. Feature detection and matching
3. Pairwise camera pose estimation
4. Transform to global coordinate system
5. Bundle adjustment

It is important to note that the acquired images in this case is RGB-D and is
acquired using a Microsoft Kinect (first generation).

The pipeline is implemented in C++. Third party libraries used include OpenCV
3.0, Ceres Solver and PCL 1.8.



## Data acquisition

OpenNI and OpenCV are used to acquire RGB images and depth maps from a Kinect.
The two output images are stored with the same timestamps.

It is worth noting that the Kinect returns depth in mm in range $[0, 10000]$
which is stored as a 16-bit unsigned integer. Camera parameters are taken from
*(smisek20133d)* and used in methods in the following steps.

While acquiring data, we attempt to find areas with sufficient potential
features and try to maximise the overlap between each shot to retain enough
correspondences.

*Panorama of BeersNMore*

This resulted in the BeersNMore dataset, taken at a craft beer shop on
Universitätstrasse. The interior of the shop (as seen in figure #) exhibits
numerous unique and repeating features in the form of labelled beer bottles
boxes, and crates. The dataset consists of 229 RGB and depth images.



## Feature detection and matching

For each given RGB image, SIFT features are found, and 128-dimensional are
descriptors calculated. Standard OpenCV parameters are used for this step.

A matching algorithm is then run between all potential image pairs. This is
an $\mathcal{O}(n^2)$ operation. The matcher compares the distance between
descriptors and finds those with minimum distance. This produces numerous
incorrect matches, often visible by the violation of epipolar geometry.

The matching is therefore performed in a bi-directional manner, both from image
i to j, and j to i. This results in a better sample of matches. RANSAC can be
used in the camera registration step to further eliminate outliers.



## Pairwise camera pose estimation

The previous step yields a list of image pairs which have matching features.
The discovered features from image i can then be projected into 3D space
using its depth map values, and reprojected into image j. The minimisation
of this error is carried out using OpenCV's EPnP solver.

The mentioned solver also identifies outlier matches via RANSAC. Only inliers
are retained to improve any further optimisations. To ensure that only good
image pairs are retained, we also filter these image pairs based on an absolute
minimum number of inliers of 30. If two images have less then 30 feature matches
which are inliers in the registration process, it is assumed that the image pair
is not good enough for subsequent steps.

Similar to the feature matching step, the PnP solver is run in both directions,
projecting 3D points from image i into image j, and projecting 3D points from
image j into image i. This allows for two things, (1) the averaging of pose
estimates to improve accuracy and (2) reduction of bad matches where relative
rotation vectors do not add up to 0.

The final output from this step is a list of camera pairs which are deemed to
have good matching features, and associated pairwise camera pose estimates.



## Transform to global coordinate system

The final goal of this SfM pipeline is to combine the data from all images
acquired. To do so, previously acquired pairwise camera pose estimates must
be transformed into a single coordinate frame.

*Spanning Tree diagram*

The 0th camera is selected to be the reference coordinate frame. A breadth-first
algorithm is used to construct a minimum spanning tree with cameras as nodes and
the existence of camera pose estimate as edges (whether an image pair exists).
The spanning tree can be walked to calculate camera pose estimates relative to
the 0th camera.

*Cluster of keypoints diagram*

The outcome of this step needs to be a cloud of keypoints and camera pose
estimates in global coordinate frame. However, each keypoint is observed by a
minimum of two cameras, resulting in a cluster of keypoint coordinate estimates.
The centre of mass (CoM) of this cluster is calculated by averaging the
coordinate estimations. This results in a single keypoint coordinate estimate,
and consequently an initial point cloud of sparse features.




## Bundle adjustment

With global keypoint and camera pose estimates, it is now possible to perform a
global optimisation known as bundle adjustment.



Results
=======
*NOTE: + discussion*



Conclusion
==========



Abstract
========



