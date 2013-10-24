#pragma once

#ifndef COMMON_H
#define COMMON_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <vector>
#include <iostream>
#include <list>
#include <set>

using namespace cv;


/* Function to write point cloud in .txt file */
void WritePointClouds(std::vector<Point3d> pointCloud,int ImgNo);

/* Function to write point cloud in .XML file */
void WritePointCloudsXML(std::vector<Point3d> pointCloud,int ImgNo);

/* Function to write matching points in .txt file */
void WriteMatches(std::vector<Point2f> FeaturePts1,std::vector<Point2f> FeaturePts2,int ImgNo);
#endif // COMMON_H
