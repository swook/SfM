Compile
=======

Compile output using `latexmk -pdf report.tex` or `pdflatex report.tex` or similar.


Introduction
============
*NOTE: Problem statement*

# Structure from Motion

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


## Dataset

*Panorama of BeersNMore*

Our dataset is acquired from BeersNMore (figure #), a craft beer shop on
Universitätstrasse. The interior of the shop contains repeating as well
as numerous unique features in the form of labelled beer bottles and crates.

The dataset consists of 229 colour (RGB) and depth (11bit) images which were
acquired with the intent to create sufficient correspondences between the
images.


## Data acquisition

*NOTE: Kinect parameters, f, kx*



## Feature detection and matching


## Pairwise camera pose estimation


## Transform to global coordinate system


## Bundle adjustment



Results
=======
*NOTE: + discussion*



Conclusion
==========



Abstract
========



