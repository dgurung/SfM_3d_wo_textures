#ifndef RECONSTRUCT3D_H
#define RECONSTRUCT3D_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <list>

class reconstruct3d{

public:
    reconstruct3d(  const std::vector<cv::Mat>& images,
                const std::vector<std::string>& imgs_names_,
                const std::string& imgs_path_);

    void ExtractAndMatchFeatures();

private:
    bool features_present;  // variable to ensure feature extraction is carried out once

    /***********************************************************/
    /* Declaration of Camera parameters */
private:
    /// Field for Camera parameter variables
    cv::Mat cam_matrix;

    cv::Mat K;
    cv::Mat_<double> Kinv;

    /***********************************************************/
    /* Declaration of Image associated features and  keypoints */
private:
    /// Field for image variables
    std::vector<cv::Mat_<cv::Vec3b> > imgs_orig;
    std::vector<cv::Mat> imgs;
    std::vector<std::string> imgs_names;

    /// Associated key points of extracted features from image
    std::vector<std::vector<cv::KeyPoint> > imgpts;
    std::vector<std::vector<cv::KeyPoint> > fullpts;
    std::vector<std::vector<cv::KeyPoint> > imgpts_good;

    /*************************************************************/
    /* Function for feauter */
public:
    void MatchFeatures(int idx_i, int idx_j, std::vector<cv::DMatch>* matches);

public:
    std::map< std::pair<int,int>, std::vector<cv::DMatch> > matches_matrix;


    /*******************************************************/
public:
    void ComputeFMatrixAndCorPoints();

    /******************************* Triangulation **********************************/
private:
    std::map<int,cv::Matx34d> Pmats;
    int m_first_view;
    int m_second_view;

    std::vector<CloudPoint> pcloud;

    int FindHomographyInliers2Views(int vi, int vj);
    void TriangulationOfPoints();
    bool TriangulatePointsBetweenViews(
        int working_view,
        int second_view,
        std::vector<struct CloudPoint>& new_triangulated,
        std::vector<int>& add_to_cloud
        );

public:
    std::vector<std::string> imageforreconstruction;
    std::string image_path;













    };
#endif // RECONSTRUCT3D_H
