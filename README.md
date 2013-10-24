Structure from Motion  
========================
A Qt based gui for SfM using OpenCV and PCL library.
(Currently under development)



3D point cloud without texture mapping
----------------------------------------
This project is for buidling 3D point cloud from multiple images of a rigid scene. The method for implementing SfM is described in 
paper [1]. The basic idea is to first obtain 3D points from a pair of images. SURF feature matching is done between two images
having widest baseline. Corresponding points are used to estimate the motion (or transformation) between two images based on epipolar geometry.
Transformation between two images are used to compute 3D points by triangulation method. This pairing up of images based on width of baseline is 
done for rest of the images. This yields multiple 3D point clouds but have different scale.

So as to merge these point clouds one way is to built ratio histogram. 
For example consider two point cloud and that they share some number of points that are common to both point clouds. A ratio of distance of a common point is computed. This is done for all available common points. And finally we built a histogram of these ratio.
The ratio which is repeated most is considered as a scaling factor for the two point clouds. Currently I am working on this part , that is multiple cloud merging.


Tools and Libraries Required:
--------------------
- Qt
- OpenCV
- Point Cloud Library (PCL)

Usage:
------
Built the project in Qt. 


To run SfM project, images and intrinsic camera parameters are required. For the Camera calibration matrix, xml files of OpenCV format is used. The calibration matrix can also be entered through dialogbox.


Datasets:
--------
Standard datasets for SfM can be found here: http://cvlabwww.epfl.ch/~strecha/testimages.html


References:
-----------
1. Modeling the World from Internet Photo Collections, N. Snavely, S. M. Seitz, R. Szeliski, IJCV 2007 
http://phototour.cs.washington.edu/ModelingTheWorld_ijcv07.pdf

1. Multiple View Geometry in Computer Vision, Hartley, R. I. and Zisserman, A., 2004, Cambridge University Press
http://www.robots.ox.ac.uk/~vgg/hzbook/

1. Mastering OpenCV with Practical Computer Vision Projects
 https://github.com/royshil/SfM-Toy-Library

