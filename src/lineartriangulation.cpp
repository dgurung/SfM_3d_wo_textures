#pragma once

#include "lineartriangulation.h"
#include "pretriangulation.h"
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
lineartriangulation::lineartriangulation()
{
}


/*From "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image understanding, 1997
    cv::Point3d u;  //homogenous image point (u,v,1);
    cv::Matx34d P;  //camera1 matrix;
    cv::Point3d u1;     //homogenous image point in 2nd camera
    cv::Matx3d P1;      //camera2 matrix
*/

 cv::Mat_<double> lineartriangulation::LinearLSTriangulation(cv::Point3d u, cv::Matx34d P, cv::Point3d u1, cv::Matx34d P1){
    //build matrix A for homogenous equation system Ax = 0
    //assume X = (x,y,z,1), for Linear-LS method
    //which turns it into a AX = B system, where A is 4x3, X is 3x1 and B is 4x1
    Matx43d A(u.x*P(2,0)-P(0,0),    u.x*P(2,1)-P(0,1),      u.x*P(2,2)-P(0,2),
          u.y*P(2,0)-P(1,0),    u.y*P(2,1)-P(1,1),      u.y*P(2,2)-P(1,2),
          u1.x*P1(2,0)-P1(0,0), u1.x*P1(2,1)-P1(0,1),   u1.x*P1(2,2)-P1(0,2),
          u1.y*P1(2,0)-P1(1,0), u1.y*P1(2,1)-P1(1,1),   u1.y*P1(2,2)-P1(1,2)
              );
    cv::Matx41d B(-(u.x*P(2,3) - P(0,3)),  -(u.y*P(2,3) - P(1,3)), -(u1.x*P1(2,3) - P1(0,3)), -(u1.y*P1(2,3) - P1(1,3)));
    cv::Mat_<double> X_;
    Mat_<double> X(4,1);
    solve(A,B,X_,DECOMP_SVD);
    X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); X(3) = 1.0;

//    for(int ii=0;ii<4;ii++){
//        cout <<"Check X values"<<endl<< X.at<double>(ii)<<'\t';
//    }
//    for(int ii=0;ii<4;ii++){
//        cout <<"Check X_ values"<<endl<< X_.at<double>(ii)<<'\t';
//    }
//    cout<<endl<<endl;
    return X;
}




double lineartriangulation::TriangulatePoints(const vector<Point2f> & pt_left,const vector<Point2f>& pt_right,
          const Mat& K,const Matx34d& P,const Matx34d& P1,vector<Point3d>& pointcloud,bool validProjMat)
{
    std::vector<KeyPoint> pt_set1;
    cv::KeyPoint::convert(pt_left,pt_set1);
    std::vector<KeyPoint> pt_set2;
    cv::KeyPoint::convert(pt_right,pt_set2);

    vector<double> reproj_error;
    for (unsigned int i=0; i<pt_set1.size(); i++) {
    //convert to normalized homogeneous coordinates
        Point2f kp = pt_set1[i].pt;
        Point3d u(kp.x,kp.y,1.0);
        Mat_<double> um = K.inv() * Mat_<double>(u);
        u = um.at<Point3d>(0);
        Point2f kp1 = pt_set2[i].pt;
        Point3d u1(kp1.x,kp1.y,1.0);
        Mat_<double> um1 = K.inv() * Mat_<double>(u1);
        u1 = um1.at<Point3d>(0);
        //triangulate
        Mat_<double> X = LinearLSTriangulation(u,P,u1,P1);
        //calculate reprojection error

     //if Z value less than 0, then the projection matrix is not correct;
        if( X(2) < -0.01 ) {
            if(i<50&&validProjMat){
//            cout <<"Check X values"<<endl<< X.at<double>(0)<<'\t'<<X.at<double>(1)<<'\t'<<X.at<double>(2)<<'\t';
            pointcloud.clear(); return -1.0;}
        }

        Mat_<double> xPt_img = K * Mat(P1) * X;
        Point2f xPt_img_(xPt_img(0)/xPt_img(2),xPt_img(1)/xPt_img(2));
                         reproj_error.push_back(norm(xPt_img_-kp1));
        //store 3D point
        pointcloud.push_back(Point3d(X(0),X(1),X(2)));
    }
    //return mean reprojection error
    Scalar me = mean(reproj_error);
    return me[0];
}
