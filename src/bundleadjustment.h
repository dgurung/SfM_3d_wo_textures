#pragma once

#ifndef BUNDLEADJUSTMENT_H
#define BUNDLEADJUSTMENT_H
#include <vector>
#include <set>
#include <iostream>
#include <iterator>
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace cv;
using namespace std;

class BundleAdjustment
{
public:
    vector< vector <Point3d> > Pts3D;   /* 3D pointset */
    vector< vector <Point3d> > AffinedPts3D; /* Affined 3D points */
    vector< Point3d > All3DPts; /* all 3D points without overlap */
    vector< vector <Point2f> > Pts2D;   /* 2D points curresponding to the 3D points*/
    vector< cv::Matx34d > ProjMat;      /* Store the projection matrices */
    vector< cv::Mat_<double> > RotationMat;
    vector< cv::Mat_<double> > TranslationMat;
    Mat_<double> K;                     /* K is the intrinsic parameters of the camera */

    int m_first_view;
    int m_second_view; //baseline's second view other to 0
    std::set<int> done_views;
    std::set<int> good_views;

    BundleAdjustment();
    void _3DCospaceTransformation(std::vector< cv::Mat> &images);
//    double BackProjErr(vector <Point3d> Ptscloudold,vector <Point3d> Ptscloudnew);
    void LocalBundleAdjustment(void);
    void GlobalBundleAdjustment(void);
};

#endif // BUNDLEADJUSTMENT_H
