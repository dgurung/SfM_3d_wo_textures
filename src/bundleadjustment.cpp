#pragma once
#include "bundleadjustment.h"
#include "pretriangulation.h"
#include "lineartriangulation.h"
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/contrib/contrib.hpp>

using namespace Eigen;
BundleAdjustment::BundleAdjustment()
{

}

void BundleAdjustment::_3DCospaceTransformation(std::vector< cv::Mat> &images){
    /* Function to calculate the BackProjection Errors */
    double BackProjErr(vector <Point3f> Ptscloudold,vector <Point3d> Ptscloudnew);
    /* Function to remove the bad reprojection points */
    void RemoveBackProjOutliners(vector <Point3f> Ptscloudold,vector <Point3d> Ptscloudnew,
                                 vector <Point3f> & Updateold, vector <Point3f> & Updatenew,
                                 vector<Point2f> Pts2dold,vector<Point2f> Pts2dnew,
                                 vector<Point2f> & Update2dold,vector<Point2f> & Update2dnew);


    /* This part start to add new images */
    for(int ImgNo=2,loopCount = 1; ImgNo<images.size(); ImgNo++,loopCount+=2){
        PreTriangulation _NewTriangulation;
        _NewTriangulation.K = (*this).K;

        _NewTriangulation.img1 = images[ImgNo-1];

        _NewTriangulation.img2 = images[ImgNo];
        _NewTriangulation.Feature_Extraction();
        Pts2D.push_back(_NewTriangulation.imgpts1);
        Pts2D.push_back(_NewTriangulation.imgpts2);
/******************************************************************************************/
        /* Find the trangulated 3d points in the reference coordinate */
/******************************************************************************************/

        vector<Point3d> Old3d = Pts3D[ImgNo-2];
        vector<Point2f> Old2d = Pts2D[loopCount];
        vector<Point2f> New2d = Pts2D[loopCount+1];
        vector<Point2f> NewImg = Pts2D[loopCount+2];
        vector<Point3f> New3DPts;
        vector<Point2f> Common2d;
        vector<Point2f> Common2dold;

        //std::cout << "data read ok " << std::endl;

        vector<int> vecidx3d,vecidx2d;
        for(int iold=0;iold<Old2d.size();iold++){
            Point2f ptsOld = Old2d[iold];
            for(int inew=0;inew<New2d.size();inew++){
                Point2f ptsNew = New2d[inew];
                Point2f ptsImg = NewImg[inew];
/* Common Points in old images with new image are found, store them for camera position estimation */
                if(int(ptsOld.x) == int(ptsNew.x) && int(ptsOld.y) == int(ptsNew.y)){
                    New3DPts.push_back(Old3d[iold]);
//                    Common2d.push_back(ptsNew);
                    Common2d.push_back(ptsImg);
                    Common2dold.push_back(ptsOld);
                    vecidx3d.push_back(iold);
                    vecidx2d.push_back(inew);
                }
            }
        }


/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
                                 /* Camera Position Estimation*/
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
        cv::Mat_<double> t,rvec,R;
        cv::Mat_<double> distcoeff = cv::Mat_<double>::zeros(1,4);

        //cout << "Updateold: " << New3DPts.size() <<  '\t' << "Update2new" << Common2d.size() << endl;
        cv::solvePnPRansac(New3DPts, Common2d, K, distcoeff, rvec, t, false);
        //get rotation in 3x3 matrix form
        Rodrigues(rvec, R);
        _NewTriangulation.ProjectionMat2 = cv::Matx34d(R(0,0),R(0,1),R(0,2),t(0),
                                                       R(1,0),R(1,1),R(1,2),t(1),
                                                       R(2,0),R(2,1),R(2,2),t(2));
        _NewTriangulation.ProjectionMat1 = cv::Matx34d( 1, 0, 0, 0,
                                                        0, 1, 0, 0,
                                                        0, 0, 1, 0);
        //std::cout<< _NewTriangulation.ProjectionMat2<< std::endl;
        lineartriangulation _New3Dpts1;
        _New3Dpts1.TriangulatePoints(_NewTriangulation.imgpts1,_NewTriangulation.imgpts2,_NewTriangulation.K,
        _NewTriangulation.ProjectionMat1,_NewTriangulation.ProjectionMat2,_New3Dpts1.pointcloud,false);
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
                /*+ remove outliners +*/
        lineartriangulation _BackProjPts;
        _BackProjPts.TriangulatePoints(Common2dold,Common2d,K,_NewTriangulation.ProjectionMat1,
                                       _NewTriangulation.ProjectionMat2,_BackProjPts.pointcloud,false);

//        double MeanProjErr = BackProjErr(New3DPts,_BackProjPts.pointcloud);
//        cout<<MeanProjErr<<endl;

        vector <Point3f> Updateold;
        vector <Point3f> Updatenew;
        vector<Point2f> Update2dold;
        vector<Point2f> Update2dnew;

        RemoveBackProjOutliners(New3DPts,_BackProjPts.pointcloud,Updateold,Updatenew,
                                Common2dold,Common2d,Update2dold,Update2dnew);

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
          /* Re-estimate the camera position, and triangulate the new points */

        //if()

        //cout << "Updateold: " << Updateold.size() <<  '\t' << "Update2new" << Updatenew.size() << endl;
        //if(Updateold)
        cv::solvePnPRansac(Updateold, Update2dnew, K, distcoeff, rvec, t, false);
        //get rotation in 3x3 matrix form
        Rodrigues(rvec, R);
        _NewTriangulation.ProjectionMat2 = cv::Matx34d(R(0,0),R(0,1),R(0,2),t(0),
                                                       R(1,0),R(1,1),R(1,2),t(1),
                                                       R(2,0),R(2,1),R(2,2),t(2));
        _NewTriangulation.ProjectionMat1 = cv::Matx34d( 1, 0, 0, 0,
                                                        0, 1, 0, 0,
                                                        0, 0, 1, 0);
        lineartriangulation _New3Dpts;
        _New3Dpts.TriangulatePoints(_NewTriangulation.imgpts1,_NewTriangulation.imgpts2,_NewTriangulation.K,
        _NewTriangulation.ProjectionMat1,_NewTriangulation.ProjectionMat2,_New3Dpts.pointcloud,false);

///*++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        /* Add new points to the reffrence coordinate */
        int idxold=0;
        for(int i3d=0;i3d<_New3Dpts.pointcloud.size();i3d++){
            if((std::find(vecidx2d.begin(),vecidx2d.end(),i3d)!=vecidx2d.end()) && (i3d<New2d.size())){
                _New3Dpts.pointcloud[i3d] = Old3d[vecidx3d[idxold]];
                idxold++;
            }
            else{
            All3DPts.push_back(_New3Dpts.pointcloud[i3d]);}
        }
        Pts3D.push_back(_New3Dpts.pointcloud);
    }
}

/* Function to calculate the back-projection errors */

double BackProjErr(vector <Point3f> Ptscloudold,vector <Point3d> Ptscloudnew){

    Point3f Ptsold;
    Point3d Ptsnew;
    double MeanProjErr = 0;
    vector<double> ProjErr;
    double absvalue = 0;
    for(int i=0;i<Ptscloudold.size();i++){
        Ptsold = Ptscloudold[i];
        Ptsnew = Ptscloudnew[i];
        absvalue += sqrt(pow((Ptsold.x),2)+pow((Ptsold.y-Ptsnew.y),2)+pow((Ptsold.z),2));
        ProjErr.push_back(sqrt(pow((Ptsold.x-Ptsnew.x),2)+ pow((Ptsold.y-Ptsnew.y),2)+pow((Ptsold.z-Ptsnew.z),2)));
        MeanProjErr += sqrt(pow((Ptsold.x-Ptsnew.x),2)+ pow((Ptsold.y-Ptsnew.y),2)+pow((Ptsold.z-Ptsnew.z),2));
    }
    std::sort(ProjErr.begin(),ProjErr.end());
    //cout<<"median error:  "<<ProjErr[(Ptscloudold.size()/2)];
    absvalue = absvalue/Ptscloudold.size();
    return MeanProjErr = MeanProjErr/Ptscloudold.size();
}

/* Function to remove the outliners */
void RemoveBackProjOutliners(vector <Point3f> Ptscloudold,vector <Point3d> Ptscloudnew,
                             vector <Point3f> & Updateold, vector <Point3f> & Updatenew,
                             vector<Point2f> Pts2dold,vector<Point2f> Pts2dnew,
                             vector<Point2f> & Update2dold,vector<Point2f> & Update2dnew){

    Point3f Ptsold;
    Point3d Ptsnew;
    vector <Point3f> * Updold = &Updateold;
    vector <Point3f> * Updnew = &Updatenew;
    vector <Point2f> * Upd2dold = &Update2dold;
    vector <Point2f> * Upd2dnew = &Update2dnew;
    set <double> ProjectErr;
    vector<double> ProjErr;
    for(int i=0;i<Ptscloudold.size();i++){
        Ptsold = Ptscloudold[i];
        Ptsnew = Ptscloudnew[i];
        ProjErr.push_back(sqrt(pow((Ptsold.x-Ptsnew.x),2)+ pow((Ptsold.y-Ptsnew.y),2)+pow((Ptsold.z-Ptsnew.z),2)));
        ProjectErr.insert(sqrt(pow((Ptsold.x-Ptsnew.x),2)+ pow((Ptsold.y-Ptsnew.y),2)+pow((Ptsold.z-Ptsnew.z),2)));
        }
    std::sort(ProjErr.begin(),ProjErr.end());

    //cout<<"Back Projection Error size"<<ProjErr.size()<<endl;
    //cout<<"Back Projection Error set size:"<<ProjectErr.size()<<endl;
//    std::set<double>::iterator lit;
//    int kkk=0;
//    for(lit=ProjectErr.begin();kkk<10;lit++,kkk++){
//        cout<<*lit<<endl;
//    }
//    for(kkk=0,lit=ProjectErr.end();kkk<10;lit--,kkk++){
//        cout<<*lit<<endl;
//    }




    double MedianErr = 0;
    if(Ptscloudold.size()/2>300){MedianErr = ProjErr[Ptscloudold.size()/4];cout<<"Median Error:"<<MedianErr<<endl<<endl;}
    else MedianErr = ProjErr[Ptscloudold.size()/2]; //cout<<"Median Error:"<<MedianErr<<endl<<endl;

    //cout<<"Back Projection Median Error equals to:"<<MedianErr;
    if(MedianErr != 0){
        for(int i=0;i<Ptscloudold.size();i++){
            Ptsold = Ptscloudold[i];
            Ptsnew = Ptscloudnew[i];
            double Err = sqrt(pow((Ptsold.x-Ptsnew.x),2)+ pow((Ptsold.y-Ptsnew.y),2)+pow((Ptsold.z-Ptsnew.z),2));
            if(Err<MedianErr){
                (*Upd2dold).push_back(Pts2dold[i]);
                (*Upd2dnew).push_back(Pts2dnew[i]);
                (*Updold).push_back(Ptsold);
                (*Updnew).push_back(Ptsnew);
            }
        }
    }
    else{
        for(int i=0;i<Ptscloudold.size();i++){
            Ptsold = Ptscloudold[i];
            Ptsnew = Ptscloudnew[i];
                (*Upd2dold).push_back(Pts2dold[i]);
                (*Upd2dnew).push_back(Pts2dnew[i]);
                (*Updold).push_back(Ptsold);
                (*Updnew).push_back(Ptsnew);

        }
    }
}


void BundleAdjustment::LocalBundleAdjustment(void){

    // load the data;
    // 3D points transformation;
    // reproject the 3D points to images;
    // build cost function;
    // solve the function;
    // save the new data point;

}



void BundleAdjustment::GlobalBundleAdjustment(void){

//    cv::LevMarqSparse::bundleAdjust();

}
