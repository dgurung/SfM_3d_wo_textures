/*****************************************************************************
*   ExploringSfMWithOpenCV
******************************************************************************
*   by Roy Shilkrot, 5th Dec 2012
*   http://www.morethantechnical.com/
******************************************************************************
*   Ch4 of the book "Mastering OpenCV with Practical Computer Vision Projects"
*   Copyright Packt Publishing 2012.
*   http://www.packtpub.com/cool-projects-with-opencv/book
*****************************************************************************/

#include "Triangulation.h"

#include <iostream>

using namespace std;
using namespace cv;



/**
 From "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image understanding, 1997
 */
Mat_<double> LinearLSTriangulation(Point3d u,		//homogenous image point (u,v,1)
								   Matx34d P,		//camera 1 matrix
								   Point3d u1,		//homogenous image point in 2nd camera
								   Matx34d P1		//camera 2 matrix
								   ) 
{
    Matx43d A(u.x*P(2,0)-P(0,0),	u.x*P(2,1)-P(0,1),		u.x*P(2,2)-P(0,2),
			  u.y*P(2,0)-P(1,0),	u.y*P(2,1)-P(1,1),		u.y*P(2,2)-P(1,2),		
			  u1.x*P1(2,0)-P1(0,0), u1.x*P1(2,1)-P1(0,1),	u1.x*P1(2,2)-P1(0,2),	
			  u1.y*P1(2,0)-P1(1,0), u1.y*P1(2,1)-P1(1,1),	u1.y*P1(2,2)-P1(1,2)
			  );
	Matx41d B(-(u.x*P(2,3)	-P(0,3)),
			  -(u.y*P(2,3)	-P(1,3)),
			  -(u1.x*P1(2,3)	-P1(0,3)),
			  -(u1.y*P1(2,3)	-P1(1,3)));
	
	Mat_<double> X;
	solve(A,B,X,DECOMP_SVD);
	
	return X;
}



/**
 From "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image understanding, 1997
 */
Mat_<double> IterativeLinearLSTriangulation(Point3d u,	//homogenous image point (u,v,1)
											Matx34d P,			//camera 1 matrix
											Point3d u1,			//homogenous image point in 2nd camera
											Matx34d P1			//camera 2 matrix
											) {
	double wi = 1, wi1 = 1;
	Mat_<double> X(4,1); 
	for (int i=0; i<10; i++) { //Hartley suggests 10 iterations at most
		Mat_<double> X_ = LinearLSTriangulation(u,P,u1,P1);
		X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); X(3) = 1.0;
		
		//recalculate weights
		double p2x = Mat_<double>(Mat_<double>(P).row(2)*X)(0);
		double p2x1 = Mat_<double>(Mat_<double>(P1).row(2)*X)(0);
		
		//breaking point
		if(fabsf(wi - p2x) <= EPSILON && fabsf(wi1 - p2x1) <= EPSILON) break;
		
		wi = p2x;
		wi1 = p2x1;
		
		//reweight equations and solve
		Matx43d A((u.x*P(2,0)-P(0,0))/wi,		(u.x*P(2,1)-P(0,1))/wi,			(u.x*P(2,2)-P(0,2))/wi,		
				  (u.y*P(2,0)-P(1,0))/wi,		(u.y*P(2,1)-P(1,1))/wi,			(u.y*P(2,2)-P(1,2))/wi,		
				  (u1.x*P1(2,0)-P1(0,0))/wi1,	(u1.x*P1(2,1)-P1(0,1))/wi1,		(u1.x*P1(2,2)-P1(0,2))/wi1,	
				  (u1.y*P1(2,0)-P1(1,0))/wi1,	(u1.y*P1(2,1)-P1(1,1))/wi1,		(u1.y*P1(2,2)-P1(1,2))/wi1
				  );
		Mat_<double> B = (Mat_<double>(4,1) <<	  -(u.x*P(2,3)	-P(0,3))/wi,
												  -(u.y*P(2,3)	-P(1,3))/wi,
												  -(u1.x*P1(2,3)	-P1(0,3))/wi1,
												  -(u1.y*P1(2,3)	-P1(1,3))/wi1
						  );
		
		solve(A,B,X_,DECOMP_SVD);
		X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); X(3) = 1.0;
	}
	return X;
}

//Triagulate points
double TriangulatePoints(const vector<KeyPoint>& pt_set1, 
						const vector<KeyPoint>& pt_set2, 
						const Mat& K,
                        const Matx34d &P,
                        const Matx34d &P1,
                        vector<CloudPoint>& pointcloud)
{	
    cout << "Triangulating...";

    Matx44d P1_(P1(0,0),P1(0,1),P1(0,2),P1(0,3),
                P1(1,0),P1(1,1),P1(1,2),P1(1,3),
                P1(2,0),P1(2,1),P1(2,2),P1(2,3),
                0,		0,		0,		1);

    cout << "Triangulating...";
    double t = getTickCount();
    vector<double> reproj_error;
    unsigned int pts_size = pt_set1.size();

    Mat_<double> KP1 = K * Mat(P1);

    cv::Mat_<double> Kinv;
    invert(K, Kinv);

    for (int i=0; i<pts_size; i++) {
        Point2f kp = pt_set1[i].pt;
        Point3d u(kp.x,kp.y,1.0);
        Mat_<double> um = Kinv * Mat_<double>(u);
        u.x = um(0); u.y = um(1); u.z = um(2);

        Point2f kp1 = pt_set2[i].pt;
        Point3d u1(kp1.x,kp1.y,1.0);
        Mat_<double> um1 = Kinv * Mat_<double>(u1);
        u1.x = um1(0); u1.y = um1(1); u1.z = um1(2);

        Mat_<double> X = IterativeLinearLSTriangulation(u,P,u1,P1);


        Mat_<double> xPt_img = KP1 * X;				//reproject

        Point2f xPt_img_(xPt_img(0)/xPt_img(2),xPt_img(1)/xPt_img(2));

            double reprj_err = norm(xPt_img_-kp1);
            reproj_error.push_back(reprj_err);

            CloudPoint cp;
            cp.pt = Point3d(X(0),X(1),X(2));
            cp.reprojection_error = reprj_err;

            pointcloud.push_back(cp);

    }
    Scalar mse = mean(reproj_error);
    t = ((double)getTickCount() - t)/getTickFrequency();
    cout << "Done. ("<<pointcloud.size()<<"points, " << t <<"s, mean reproj err = " << mse[0] << ")"<< endl;

    return mse[0];
}



double TriangulatePoints(const vector< Point2f >& pt_set1,
                         const vector< Point2f >& pt_set2,
                         const Mat& K,
                         const Matx34d& P1,
                         const Matx34d& P2,
                         vector<Point3d>& pointcloud){


    vector<double> reproj_error;
    Point3d pts1,pts2;
    Mat_<double> um,X,xPt_img_hom;
    Point2f xPt_img;

    for (unsigned int i=0;i<pt_set1.size();i++){

        pts1.x = pt_set1[i].x; pts1.y = pt_set1[i].y; pts1.z = 1.0;
        pts2.x = pt_set2[i].x; pts2.y = pt_set2[i].y; pts2.z = 1.0;

        um = K.inv() * Mat_<double>(pts1);
        pts1 = um.at<Point3d>(0);

        um = K.inv() * Mat_<double>(pts2);
        pts2 = um.at<Point3d>(0);

        X = LinearLSTriangulation(pts1,P1,pts2,P2);

        Mat P1_mat(P1);

        xPt_img_hom = K * P1_mat * X;

        xPt_img.x = (xPt_img_hom(0)/xPt_img_hom(2));
        xPt_img.y = (xPt_img_hom(1)/xPt_img_hom(2));

        reproj_error.push_back(norm(xPt_img-pt_set2[i]));

        pointcloud.push_back(Point3d(X(0),X(1),X(2)));
    }

    Scalar me = mean(reproj_error);

    return me[0];
}
