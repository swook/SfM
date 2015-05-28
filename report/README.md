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

With early implementations of SfM, epipolar geometry is used on RGB image pairs
to
*NOTE: Epipolar geometry, fundamental matrix*
*NOTE: Kinect, RGB-D, ICP, PnP*



Theory
======
*NOTE: Algorithms*



Methodology
===========

As outlined in the previous section, our structure from motion pipeline is
composed of 5 main steps listed below.

1. Data acquisition
2. Feature detection and matching
3. Relative camera pose estimation
4. Transform to a single coordinate system
5. Bundle adjustment

It is important to note that the acquired images in this case is RGB-D

*NOTE: Kinect parameters, f, kx*
*NOTE: P3P equations*


# Dataset

Our dataset is acquired from BeersNMore, a craft beer shop on
Universitätstrasse. The interior of the shop contains repeating as well
as numerous unique features in the form of labelled beer bottles and crates.

The dataset consists of 226 colour (RGB) and depth (11bit) images which were
acquired with the intent to cover




Results
=======
*NOTE: + discussion*



Conclusion
==========



Abstract
========



