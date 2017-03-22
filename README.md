This repository is an implementation of the [Stereo Odometry based on careful Feature selection
and Tracking](ieeexplore.ieee.org/iel7/7320493/7324045/07324219.pdf), as a part of the course project for [Probabilistic Mobile Robotics](http://home.iitk.ac.in/~gpandey/ee_698g.html).

# Keypoint Detection

In this section, we split the [keypoint detection and matching pipeline](http://mesh.brown.edu/engn1610/szeliski/04-featuredetectionandmatching.pdf) into four separate stages:
* __feature detection (extraction) stage:__ each image is searched for locations that are likely to match well in other images
* __feature description stage:__ each region around detected keypoint locations in converted into a more compact and stable (invariant)
descriptor that can be matched against other descriptors
* __feature matching stage:__ efficiently searching for likely matching candidates in other images
* __feature tracking stage:__ alternative to the third stage that only searches a small neighborhood around each detected feature and is therefore more suitable for video processing


## Feature matching

This part of the algorithm is concerned with finding out the features for the egomotion estimation. It is based on the process used in the paper [here](http://citeseerx.ist.psu.edu/viewdoc/download;jsessionid=6354CB2CADA3BB234F8F58A3B1C28707?doi=10.1.1.229.914&rep=rep1&type=pdf).

This process can be broken down into following steps:

1. Extraction of corner- like features in the pair of images at instant t: Blob and corner masks used over the input image
![Blob and Corner Mask](https://github.com/Mayankm96/Stereo-Odometry-SOFT/blob/master/images/detector-masks.PNG)

2. [Non- maximum and non-minimum suppression](https://pdfs.semanticscholar.org/52ca/4ed04d1d9dba3e6ae30717898276735e0b79.pdf) used on the filtered images, producing in feature candidates in either of the following classes: blob max, blob min, corner max, and corner min

3. Correspondences betwen corners found using Sum of Absolute Differences(SAD) over sparse set of pixels, that is given two feature points, we simply compare 11x11 block windows of horizontal and vertical Sobel filter responses to each other by using the sum of absolute differences (SAD) error metric. To speed-up matching, we quantize the Sobel responses to 8 bits and sum the differences over a sparse set of 16 locations instead of summing over the whole block window

4. The above step is susceptible to ouliers so circular matching is used to reject them out 

5. If circle matching is successful, normalized cross- correlation (NCC) on 25x25 path around feature positions is evaluated to reject outliers

6. Finally RANSAC is implemented to reject remaining outliers, using epipolar constraints.
