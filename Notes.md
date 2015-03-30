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

# Initial (relative) camera pose estimates

[Text](https://en.wikipedia.org/)

## Inputs

* A

## Outputs

* B



# Pairwise camera registration

RANSAC and PnP

## Inputs

* A

## Outputs

* B



# Bundle Adjustment

Ceres

## Inputs

* A

## Outputs

* B


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
