Structure from Motion Pipeline using RGB-D
==========================================

A project by Seonwook Park and Yifan Wang for the [3D Photography](http://www.cvg.ethz.ch/teaching/3dphoto/)
course at [ETH Zurich](https://www.ethz.ch/en.html) in Spring 2015.


Install
=======

1. Clone this repository
2. Setup [OpenCV 3][cv3-install], [Ceres Solver][ceres-install] and [PCL][pcl-install]
3. `mkdir build && cd build && cmake .. && make`

[cv3-install]: http://docs.opencv.org/doc/tutorials/introduction/table_of_content_introduction/table_of_content_introduction.html
[ceres-install]: http://ceres-solver.org/building.html
[pcl-install]: http://pointclouds.org/downloads/


Data Acquisition
================

1. Descend into `bernhard/`
2. Compile using `mkdir build && cd build && cmake .. && make`
3. Plug a Microsoft Kinect (gen 1) in
4. Run the binary from a directory in which data should be stored in. Use the
flag `-method 4` to store RGB and D images separately.
5. While the application is running, press `s` to record single frames. Attempt
to cover the target object or environment as much as possible.


Running the pipeline
====================

1. Get our dataset by running `bash get_dataset.sh`
1. Descend into with `cd build/`
2. Run the binary and provide the path to our dataset. For example,
`./main ../dataset BeersNMore`

