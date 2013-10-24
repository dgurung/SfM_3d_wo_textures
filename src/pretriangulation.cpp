#pragma once

#include "pretriangulation.h"
#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <set>
#include <iostream>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/nonfree/features2d.hpp>



PreTriangulation::PreTriangulation()
{
}


bool PreTriangulation::CheckCoherentRotation(cv::Mat_<double> RotationMat) {
    if( fabsf(cv::determinant(RotationMat))-1.0 > 1e-07) {
        cerr << "det(R) != +-1.0, this is not a rotation matrix" << endl;
        return false;
    }
    return true;
}


void PreTriangulation::Feature_Extraction(void){

    int minHessian = 400;
    SurfFeatureDetector detector( minHessian );
//    DenseFeatureDetector detector;
//    FastFeatureDetector detector;
    detector.detect( img1, left_keypoints );
    detector.detect( img2, right_keypoints );


    //KeyPointsToPoints( left_keypoints, left_points );
    cv::KeyPoint::convert(left_keypoints, left_points);
    //right_points(left_points.size());

    // making sure images are grayscale
    Mat prevgray,gray;
    if (img1.channels() == 3) {
        cv::cvtColor(img1,prevgray,CV_RGB2GRAY);
        cv::cvtColor(img2,gray,CV_RGB2GRAY);
    }
    else {
        prevgray = img1;
        gray = img2;
    }

    // Calculate the optical flow field:
    // how each left_point moved across the 2 images
    vector<uchar> vstatus;
    vector<float> verror(left_points.size());

    cv::calcOpticalFlowPyrLK(prevgray, gray, left_points, right_points,vstatus, verror);

    // First, filter out the points with high error
    vector<Point2f> right_points_to_find;
    vector<int> right_points_to_find_back_index;
    for (unsigned int i=0; i < vstatus.size(); i++) {
        if (vstatus[i] &&verror[i] < 12.0) {
            // Keep the original index of the point in the
            // optical flow array, for future use
            right_points_to_find_back_index.push_back(i);
            // Keep the feature point itself
            right_points_to_find.push_back(right_points[i]);//right_points_to_find.push_back(j_pts[i]);
        }
        else {
            vstatus[i] = 0; // a bad flow
        }
    }

    // for each right_point see which detected feature it belongs to
    Mat right_points_to_find_flat = Mat(right_points_to_find).reshape(1,right_points_to_find.size()); //flatten array

    vector<Point2f> right_features; // detected features
    cv::KeyPoint::convert(right_keypoints,right_features);//KeyPointsToPoints(right_keypoints,right_features);

    Mat right_features_flat = Mat(right_features).reshape(1,right_features.size());
    // Look around each OF point in the right image
    // for any features that were detected in its area
    // and make a match.
    cv::BFMatcher matcher(CV_L2);
    vector< vector <DMatch> > nearest_neighbors;
    matcher.radiusMatch( right_points_to_find_flat, right_features_flat, nearest_neighbors,2.0f);
    //matcher.radiusMatch(right_features_flat, right_points_to_find_flat, nearest_neighbors,2.0f);
    // Check that the found neighbors are unique (throw away neighbors
    // that are too close together, as they may be confusing)
    std::set<int> found_in_right_points; // for duplicate prevention

    DMatch _m;

    for(int i=0; i < nearest_neighbors.size(); i++) {

        if(nearest_neighbors[i].size()==1) {
            _m = nearest_neighbors[i][0];} // only one neighbor

        else if(nearest_neighbors[i].size()>1) {
            // 2 neighbors – check how close they are
            double ratio = nearest_neighbors[i][0].distance / nearest_neighbors[i][1].distance;

            if(ratio < 0.85) { // not too close
                // take the closest (first) one
                _m = nearest_neighbors[i][0];
            }
            else { // too close – we cannot tell which is better
                continue;} // did not pass ratio test – throw away
            }
        else {
            continue;} // no neighbors... :(

            // prevent duplicates
        if (found_in_right_points.find(_m.trainIdx) == found_in_right_points.end()) {
            // The found neighbor was not yet used:
            // We should match it with the original indexing
            // ofthe left point
            _m.queryIdx = right_points_to_find_back_index[_m.queryIdx];
            //matches->push_back(_m); // add this match
            matches.push_back(_m);
            found_in_right_points.insert(_m.trainIdx);
        }
    }

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    cout<<"pruned "<< matches.size() <<" / "<<nearest_neighbors.size()<<" matches"<<endl;
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
//    //-- Draw only "good" matches
//    Mat img_matches;
//    cv::drawMatches( img1, left_keypoints, img2, right_keypoints,
//                 matches, img_matches, Scalar::all(-1), Scalar::all(-1),
//                 vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

//    //-- Show detected matches
//    cv::namedWindow("window",CV_WINDOW_NORMAL);
//    cv::imshow( "window", img_matches );
//    waitKey(0);
    for( unsigned int i = 0; i<matches.size(); i++ )
    {
        // queryIdx is the "left" image
        imgpts1.push_back(left_keypoints[matches[i].queryIdx].pt);
        // trainIdx is the "right" image
        imgpts2.push_back(right_keypoints[matches[i].trainIdx].pt);
    }
}


void PreTriangulation::Homography2D(void){



}


void PreTriangulation::findMatrices(vector<Point2f> left_pts,vector<Point2f> right_pts){

    // Finding fundamental matrix

    F = cv::findFundamentalMat(left_pts, right_pts, CV_FM_RANSAC, 0.1, 0.99);
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    for(int ii=0;ii<3;ii++){
        for(int jj=0;jj<3;jj++){
            cout << F.at<double>(ii,jj)<<'\t';
        }
        cout<<endl;
    }
    cout<<endl;
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

    ProjectionMat1 = Matx34d( 1, 0, 0, 0,
                              0, 1, 0, 0,
                              0, 0, 1, 0);
    Kinv = K.inv();

    E = K.t() * F * K; //according to HZ (9.12)

    SVD svd(E);
    Matx33d W(0,-1,0, 1,0,0, 0,0,1);//HZ 9.13

    RotationMat1 = svd.u * Mat(W) * svd.vt;     //HZ 9.19
    RotationMat2 = svd.u * Mat(W).t() * svd.vt; //HZ 9.19
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    cout<<"RotationMat1"<<endl;
    for(int ii=0;ii<3;ii++){
        for(int jj=0;jj<3;jj++){
            cout << RotationMat1.at<double>(ii,jj)<<'\t';
        }
        cout<<endl<<endl;
    }
    cout<<"RotationMat2"<<endl;
    for(int ii=0;ii<3;ii++){
        for(int jj=0;jj<3;jj++){
            cout << RotationMat2.at<double>(ii,jj)<<'\t';
        }
        cout<<endl<<endl;
    }
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
    TranslationMat1 = svd.u.col(2);     //u3
    TranslationMat2 = - svd.u.col(2);   //-u3

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    cout <<"TranslationMat1: "<<endl;
    for(int ii=0;ii<3;ii++){
        cout << TranslationMat1.at<double>(ii)<<'\t';
    }
    cout <<endl<<"TranslationMat2: "<<endl;
    for(int ii=0;ii<3;ii++){
        cout << TranslationMat2.at<double>(ii)<<'\t';
    }
    cout<<endl<<endl;
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
    if (!CheckCoherentRotation(RotationMat1)) {
        cout<<"resulting rotation1 is not coherent\n";
        ProjectionMat21 = 0;
        ProjectionMat22 = 0;
     }
    else{
        ProjectionMat21 = Matx34d(  RotationMat1(0,0), RotationMat1(0,1), RotationMat1(0,2), TranslationMat1(0),
                                    RotationMat1(1,0), RotationMat1(1,1), RotationMat1(1,2), TranslationMat1(1),
                                    RotationMat1(2,0), RotationMat1(2,1), RotationMat1(2,2), TranslationMat1(2));

        ProjectionMat22 = Matx34d(  RotationMat1(0,0), RotationMat1(0,1), RotationMat1(0,2), TranslationMat2(0),
                                    RotationMat1(1,0), RotationMat1(1,1), RotationMat1(1,2), TranslationMat2(1),
                                    RotationMat1(2,0), RotationMat1(2,1), RotationMat1(2,2), TranslationMat2(2));
     }

    if (!CheckCoherentRotation(RotationMat2)) {
        cout<<"resulting rotation2 is not coherent\n";
        ProjectionMat23 = 0;
        ProjectionMat24 = 0;
     }
    else{

        ProjectionMat23 = Matx34d(  RotationMat2(0,0), RotationMat2(0,1), RotationMat2(0,2), TranslationMat1(0),
                                    RotationMat2(1,0), RotationMat2(1,1), RotationMat2(1,2), TranslationMat1(1),
                                    RotationMat2(2,0), RotationMat2(2,1), RotationMat2(2,2), TranslationMat1(2));

        ProjectionMat24 = Matx34d(  RotationMat2(0,0), RotationMat2(0,1), RotationMat2(0,2), TranslationMat2(0),
                                    RotationMat2(1,0), RotationMat2(1,1), RotationMat2(1,2), TranslationMat2(1),
                                    RotationMat2(2,0), RotationMat2(2,1), RotationMat2(2,2), TranslationMat2(2));
    }
}


void PreTriangulation::WriteOutResult(){

    // Write files*************************************************
//        ofstream myfile1;
//        myfile1.open ("imgpts1.txt");
//        myfile1 << imgpts1;
//        myfile1.close();

//        ofstream myfile2;
//        myfile2.open ("imgpts2.txt");
//        myfile2 << imgpts2;
//        myfile2.close();

//        ofstream myfile3;
//        myfile3.open ("FundamentalMat.txt");
//        myfile3 << F;
//        myfile3.close();

//        ofstream myfile7;
//        myfile7.open("LeftKeypoints.txt");
//        myfile7 << left_points;
//        myfile7.close();

//        ofstream myfile8;
//        myfile8.open("RightKeypoints.txt");
//        myfile8 << right_points;
//        myfile8.close();
}
