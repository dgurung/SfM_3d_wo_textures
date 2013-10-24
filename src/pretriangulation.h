#pragma once

#ifndef PRETRIANGULATION_H
#define PRETRIANGULATION_H
#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/nonfree/features2d.hpp>

using namespace cv;
using namespace std;
class PreTriangulation
{
public:
    PreTriangulation();
    cv::Mat img1;
    cv::Mat img2;
    vector<DMatch> matches;
    vector<Point2f> imgpts1, imgpts2;
    std::vector<KeyPoint> left_keypoints;
    vector<Point2f> left_points;
    std::vector<KeyPoint> right_keypoints;
    vector<Point2f> right_points;
    vector<Point2f> Dimgpts1, Dimgpts2;
    cv::Mat HomographyMat;
    cv::Mat F;
    cv::Mat_<double> K;
    cv::Mat_<double> Kinv;
    cv::Mat_<double> E;
    cv::Mat_<double> RotationMat;
    cv::Mat_<double> TranslationMat;
    cv::Mat_<double> RotationMat1;
    cv::Mat_<double> TranslationMat1;
    cv::Mat_<double> RotationMat2;
    cv::Mat_<double> TranslationMat2;
    cv::Matx34d ProjectionMat1;
    cv::Matx34d ProjectionMat2;
    cv::Matx34d ProjectionMat21;
    cv::Matx34d ProjectionMat22;
    cv::Matx34d ProjectionMat23;
    cv::Matx34d ProjectionMat24;
    void Feature_Extraction(void);
    void Homography2D(void);
    void findMatrices(vector<Point2f> left_pts,vector<Point2f> right_pts);
    void WriteOutResult(void);
    bool CheckCoherentRotation(cv::Mat_<double> RotationMat);
};

#endif // PRETRIANGULATION_H
