This repository is an implementation of the [Stereo Odometry based on careful Feature selection
and Tracking](https://ieeexplore.ieee.org/iel7/7320493/7324045/07324219.pdf), as a part of the course project for [Probabilistic Mobile Robotics](http://home.iitk.ac.in/~gpandey/ee_698g.html).

__NOTE:__ Demo Video Run on KITTI Dataset City Sequence 01 is available [here](https://youtu.be/AxtgUxlO3FY).


# How to run the repository?

__Pre-Requisites:__ Tested on PC with [MATLAB](https://in.mathworks.com/?s_tid=gn_logo) R2014a installed

1. Clone the repository using the following command:
```
git clone https://github.com/Mayankm96/Stereo-Odometry-SOFT.git
```

2. Import the dataset to the folder [`code/data`](https://github.com/Mayankm96/Stereo-Odometry-SOFT/tree/master/code/data). In case you wish to use the [KITTI](http://www.cvlibs.net/datasets/kitti/) Dataset, such as the [Residential dataset](http://www.cvlibs.net/datasets/kitti/raw_data.php?type=residential), the following command might be useful:
```bash
cd PATH/TO/Stereo-Odometry-SOFT
## For Reseidential Sequence: 61 (2011_09_46_drive_0061)
# synced+rectified data
wget -c http://kitti.is.tue.mpg.de/kitti/raw_data/2011_09_26_drive_0061/2011_09_26_drive_0061_sync.zip -P code/data
# calib.txt
wget -c http://kitti.is.tue.mpg.de/kitti/raw_data/2011_09_26_calib.zip -P code/data
```
__NOTE:__ The implementation here uses synced and rectified stereo images as inputs.

3. Change the corresponding paramters in the configuration directory [`code/config`](https://github.com/Mayankm96/Stereo-Odometry-SOFT/tree/master/code/config)
    * [`configFile1.m`](https://github.com/Mayankm96/Stereo-Odometry-SOFT/blob/master/code/config/configFile1.m): Set the path to dataset folders where images are stored
    * [`configFile2.m`](https://github.com/Mayankm96/Stereo-Odometry-SOFT/blob/master/code/config/configFile2.m): Set the camera paramters from the calibration file downloaded

4. Run the script [`code/src/main.m`](https://github.com/Mayankm96/Stereo-Odometry-SOFT/blob/master/code/src/main.m) to get a plot of the odometry estimated


# Proposed Implementation of the Algorithm

## Keypoint Detection

In this section, we split the [keypoint detection and matching pipeline](http://mesh.brown.edu/engn1610/szeliski/04-featuredetectionandmatching.pdf) into four separate stages:
* __feature detection (extraction) stage:__ each image is searched for locations that are likely to match well in other images
* __feature description stage:__ each region around detected keypoint locations in converted into a more compact and stable (invariant)
descriptor that can be matched against other descriptors
* __feature matching stage:__ efficiently searching for likely matching candidates in other images
* __feature tracking stage:__ alternative to the third stage that only searches a small neighborhood around each detected feature and is therefore more suitable for video processing


### Feature matching

This part of the algorithm is concerned with finding out the features for the egomotion estimation. It is based on the process used in the paper [here](http://citeseerx.ist.psu.edu/viewdoc/download;jsessionid=6354CB2CADA3BB234F8F58A3B1C28707?doi=10.1.1.229.914&rep=rep1&type=pdf).

This process can be broken down into following steps:

1. Extraction of corner- like features in the pair of images at instant t: Blob and corner masks used over the input image
![Blob and Corner Mask](https://github.com/Mayankm96/Stereo-Odometry-SOFT/blob/master/images/detector-masks.PNG)

2. [Non- maximum and non-minimum suppression](https://pdfs.semanticscholar.org/52ca/4ed04d1d9dba3e6ae30717898276735e0b79.pdf) used on the filtered images, producing in feature candidates in either of the following classes: blob max, blob min, corner max, and corner min

3. Correspondences betwen corners found using Sum of Absolute Differences(SAD) over sparse set of pixels, that is given two feature points, we simply compare 11x11 block windows of horizontal and vertical Sobel filter responses to each other by using the sum of absolute differences (SAD) error metric. To speed-up matching, we quantize the Sobel responses to 8 bits and sum the differences over a sparse set of 16 locations instead of summing over the whole block window

4. The above step is susceptible to ouliers so circular matching is used to reject them.

### Feature Selection

In this we carefully select only the strongest features in the image by means of bucketing. Each bucket is a 50 pixels x 50 pixels part of the image. Bucketing helps in maintaing a uniform distribution of feature points across the image.

![Feature Selection by Bucketing](https://github.com/Mayankm96/Stereo-Odometry-SOFT/blob/master/images/feature-selection.png)

In above image, the crosses are all the features detected using minimum eigenvalue algorithm in our image. The red and green ones are the features that were selected after circular matching, and finally, the green ones are the features that have been selected through bucketing.

### Feature Tracking

In our implementation we have used the  Kanade-Lucas-Tomasi (KLT) algorithm to track the features in the left camera at time instant t.

## Egomotion Estimation

### Rotation Estimation

We have used [Nister's Five Point Algorithm](http://ieeexplore.ieee.org/document/1288525/) in conjuction with RANSAC to find the best estimate of our rotation matric. The procedure used is similar to that useed to estimate structutre from motion using monocular vision.

### Translation Estimation

We build a 3D point cloud using triangulation from the previous view (i.e. at time t-1) and reproject it onto the current image frame (i.e. at time t). The reprojection error function is then minimized with respect to translation, to estimate the translation vecto
