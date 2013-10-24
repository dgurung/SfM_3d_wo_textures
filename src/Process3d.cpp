#include "Process3d.h"

#include <QDebug>
Process3d::Process3d()
{
}

Process3d::Process3d(std::vector< cv::Mat > &images, double *camera_intrinsic)
{
    images_Process3d = images;

    /* K is intrinsic parameter;
 * InitRoMat is the initial Rotation matrix;
 * InitTranMat is the initial translation matrix. */
    double Kt[] =  {2759.48, 0.0, 1520.69, 0.0, 2764.16, 1006.81, 0.0, 0.0, 1.0};//{3317.41012, 0.0, 1615.50200, 0, 3312.69870, 901.78285, 0, 0, 1};
    for(int i =0; i<9; i++)
    {
        qDebug() << "count: " << i;
        qDebug() << Kt[i];
        qDebug() << camera_intrinsic[i];
    }
    BundleAdjustment _BundleAdjustment;
    _BundleAdjustment.K = cv::Mat(3, 3, CV_64FC1, camera_intrinsic);
    double Rt[] = {1,0,0,0,1,0,0,0,1};
    double Tt[] = {0,0,0};
    cv::Mat InitRoMat = cv::Mat(3,3,CV_64FC1,Rt);
    cv::Mat InitTranMat = cv::Mat(3,1,CV_64FC1,Tt);
    _BundleAdjustment.RotationMat.push_back(InitRoMat);
    _BundleAdjustment.TranslationMat.push_back(InitTranMat);

    /*****************************************************************************************
                Initial triangulation for building base line
***************************************************************************************** */

    int ImgNo=0;

    PreTriangulation _PreTriangulation;
    lineartriangulation _LinearTriangulation;
    _PreTriangulation.K = cv::Mat(3, 3, CV_64FC1, camera_intrinsic);

    cout << "See size: Before " << images_Process3d.size() << endl;
    // Ensure a single homography does not exist between matching points of images
    FindHomographyInliers2Views();


    cout << "After homography" << endl;
    cout << "See size: " << images_Process3d.size() << endl;
    /* Read the images. */
    _PreTriangulation.img1 = images_Process3d[0];
    _PreTriangulation.img2 = images_Process3d[1];

    _PreTriangulation.Feature_Extraction();

    _PreTriangulation.findMatrices(_PreTriangulation.imgpts1,_PreTriangulation.imgpts2);
    _BundleAdjustment.Pts2D.push_back(_PreTriangulation.imgpts1);
    _BundleAdjustment.Pts2D.push_back(_PreTriangulation.imgpts2);

    /*****************************************************************************************
                       Triangulate 3D Cloud Points
***************************************************************************************** */

    cout << "trangulate start" << endl;
    double MeanProjError = -1.0;//return mean reprojection error
    int ProjMat = 1;
    while (MeanProjError == -1.0 && ProjMat <= 4){

        switch (ProjMat) {
        case 1 : {  _PreTriangulation.ProjectionMat2 = _PreTriangulation.ProjectionMat21;
            _PreTriangulation.RotationMat    = _PreTriangulation.RotationMat1;
            _PreTriangulation.TranslationMat = _PreTriangulation.TranslationMat1; break;}

        case 2 : {  _PreTriangulation.ProjectionMat2 = _PreTriangulation.ProjectionMat22;
            _PreTriangulation.RotationMat    = _PreTriangulation.RotationMat1;
            _PreTriangulation.TranslationMat = _PreTriangulation.TranslationMat2; break;}

        case 3 : {  _PreTriangulation.ProjectionMat2 = _PreTriangulation.ProjectionMat23;
            _PreTriangulation.RotationMat    = _PreTriangulation.RotationMat2;
            _PreTriangulation.TranslationMat = _PreTriangulation.TranslationMat1; break;}

        case 4 : {  _PreTriangulation.ProjectionMat2 = _PreTriangulation.ProjectionMat24;
            _PreTriangulation.RotationMat    = _PreTriangulation.RotationMat2;
            _PreTriangulation.TranslationMat = _PreTriangulation.TranslationMat2; break;}
        }


        MeanProjError = _LinearTriangulation.TriangulatePoints(_PreTriangulation.imgpts1,_PreTriangulation.imgpts2,_PreTriangulation.K,
                                                               _PreTriangulation.ProjectionMat1,_PreTriangulation.ProjectionMat2,_LinearTriangulation.pointcloud,true);

        if(MeanProjError != -1.0){
            _BundleAdjustment.ProjMat.push_back(_PreTriangulation.ProjectionMat2);

        }
        else{
            ProjMat ++;
        }
    }
    _BundleAdjustment.Pts3D.push_back(_LinearTriangulation.pointcloud);


    WritePointClouds(_LinearTriangulation.pointcloud,ImgNo);
    WritePointCloudsXML(_LinearTriangulation.pointcloud,ImgNo);


    /******************************************************************************/
    /* Building base line finished
                   Add new images           */

    cout << "Incremental SfM Starts here......." << endl;
    _BundleAdjustment.All3DPts = _LinearTriangulation.pointcloud;
    _BundleAdjustment._3DCospaceTransformation(images_Process3d);

    WritePointClouds(_BundleAdjustment.All3DPts,100);
    WritePointCloudsXML(_BundleAdjustment.All3DPts,100);
    for(int ii=0;ii<_BundleAdjustment.Pts3D.size();ii++){
        WritePointClouds(_BundleAdjustment.Pts3D[ii],ii);
        WritePointCloudsXML(_BundleAdjustment.Pts3D[ii],ii);
    }
}

void Process3d::FindHomographyInliers2Views()
{
        PreTriangulation pretestobject;

        std::vector< cv::Mat > img_temp;
        int inliers =0;
        for(int i =0; i < images_Process3d.size()-1; i++ ){
            for(int j=i+1; j< images_Process3d.size(); j++) {
                pretestobject.img1 = images_Process3d[i];
                pretestobject.img2 = images_Process3d[j];
                pretestobject.Feature_Extraction();


                double minVal,maxVal; cv::minMaxIdx(pretestobject.imgpts1,&minVal,&maxVal);
                vector<uchar> status;
                cv::Mat H = cv::findHomography(pretestobject.imgpts1,pretestobject.imgpts2,status,CV_RANSAC, 0.004 * maxVal); //threshold from Snavely07
                inliers = cv::countNonZero(status); //number of inliers
                int totatlMatch = pretestobject.imgpts1.size();
                int percent = (int)(((double)inliers) / ((double)totatlMatch) * 100.0);
                if(percent <60.0 && (i == images_Process3d.size()-1)){
                    img_temp.push_back(images_Process3d[i]);
                    img_temp.push_back(images_Process3d[j]);}
                else if(percent < 60.0 )        // Ensure less than 40% of match points are outliers that does not contribute to a single homography
                { img_temp.push_back(images_Process3d[i]);
                    if(i == images_Process3d.size()-2)
                        img_temp.push_back(images_Process3d[j]);

                    break;}

            }
        }
        images_Process3d.clear();
        images_Process3d = img_temp;
}


