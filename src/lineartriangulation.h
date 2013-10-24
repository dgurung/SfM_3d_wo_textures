#pragma once

#ifndef LINEARTRIANGULATION_H
#define LINEARTRIANGULATION_H
#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;
class lineartriangulation
{
public:

    lineartriangulation();
    cv::Mat_<double> LinearLSTriangulation(cv::Point3d u, cv::Matx34d P, cv::Point3d u1, cv::Matx34d P1);
    vector<Point3d> pointcloud;
    double TriangulatePoints(const vector<Point2f> & pt_left,const vector<Point2f> & pt_right,
              const Mat&Kinv,const Matx34d& P,const Matx34d& P1,vector<Point3d>& pointcloud,bool ValidProjMat);

};

#endif // LINEARTRIANGULATION_H
