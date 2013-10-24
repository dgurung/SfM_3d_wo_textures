#include "reconstruct3d.h"

using namespace std;

using namespace cv;

#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/flann/flann.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/legacy/legacy.hpp>

#include <vector>
#include <set>
#include <list>


bool sort_by_first(pair<int,pair<int,int> > a, pair<int,pair<int,int> > b) { return a.first < b.first; }

reconstruct3d::reconstruct3d(const vector<cv::Mat> &images,
                            const vector<std::string> &images_names,
                            const string &imgs_path_)
{
    imgs_names = images_names;
    image_path = imgs_path_;

    // Allocate memory
    for (unsigned int i=0; i<images.size(); i++) {
        imgs_orig.push_back(cv::Mat_<cv::Vec3b>());
        imgs.push_back(cv::Mat());
        cvtColor(imgs_orig[i],imgs[i], CV_BGR2GRAY);

        imgpts.push_back(std::vector<cv::KeyPoint>());
        imgpts_good.push_back(std::vector<cv::KeyPoint>());
    }
    std::cout << std::endl;

    //load calibration matrix
    cv::FileStorage fs;
    if(fs.open(imgs_path_+ "\\out_camera_data.yml",cv::FileStorage::READ)) {
        fs["camera_matrix"]>>cam_matrix;
    }
    else {
        //no calibration matrix file - mockup calibration
        cv::Size imgs_size = images[0].size();
        double max_w_h = MAX(imgs_size.height,imgs_size.width);
        cam_matrix = (cv::Mat_<double>(3,3) <<	max_w_h ,           0	,		imgs_size.width/2.0,
                                                        0,			max_w_h,	imgs_size.height/2.0,
                                                        0,			0,			1);
    }

    K = cam_matrix;
    //Extract features and store all the keypoints
    ExtractAndMatchFeatures();
}


bool sort_by_first(pair<int,pair<int,int> > a, pair<int,pair<int,int> > b) { return a.first < b.first; }



/***********************/
void reconstruct3d::ExtractAndMatchFeatures()
{
    std::cout <<"****************** detect features and match features among images **************" << std::endl;
    // detect keypoints for all images
    FastFeatureDetector ffd;
    // DenseFeatureDetector ffd;
    ffd.detect(imgs, imgpts);

    int loop1_top = imgs.size() - 1, loop2_top = imgs.size();

    for (int frame_num_i = 0; frame_num_i < loop1_top; frame_num_i++) {
        for (int frame_num_j = frame_num_i + 1; frame_num_j < loop2_top; frame_num_j++)
        {
            //std::cout << "------------ Match " << imgs_names[frame_num_i] << ","<<imgs_names[frame_num_j]<<" ------------\n";
            std::vector<cv::DMatch> matches_tmp;

            // call a big one with OF matching and KNN pruning

            this->MatchFeatures(frame_num_i,frame_num_j,&matches_tmp);
            matches_matrix[std::make_pair(frame_num_i,frame_num_j)] = matches_tmp;
        }
    }
    features_present = true;
}


/****  codebook ***********/
/**** Is robust as it tracks features using optical flow and matches these features using Brute
force method and later prunes data according to nearest neighborhood algorithm ***************/
void reconstruct3d::MatchFeatures(int left_idx, int right_idx, vector<DMatch> *matches)
{

    // i = left
    // j = right
    vector<Point2f> left_pts;
    // Convert from KeyPoints To Points
    for (unsigned int i=0; i< imgpts[right_idx].size(); i++)
        left_pts.push_back(imgpts[right_idx][i].pt);


    vector<Point2f> right_pts(left_pts.size());

    // making sure images are grayscale
    Mat prevgray,gray;
    if (imgs[left_idx].channels() == 3) {
        cvtColor(imgs[left_idx],prevgray,CV_RGB2GRAY);
        cvtColor(imgs[right_idx],gray,CV_RGB2GRAY);
    } else {
        prevgray = imgs[left_idx];
        gray = imgs[right_idx];
    }

    // Calculate the optical flow field:
    vector<uchar> vstatus(left_pts.size());
    vector<float> verror(left_pts.size());

    calcOpticalFlowPyrLK(prevgray, gray, left_pts, right_pts, vstatus, verror);


    // First, filter out the points with high error
    // Convert from KeyPoints To Points
    vector<Point2f> right_pts_to_find;
    vector<int> right_pts_to_find_back_idx;
    for (unsigned int i=0; i<vstatus.size(); i++) {
        if (vstatus[i] && verror[i] < 12.0) {
            // Keep the original index of the point in the
            // optical flow array, for future use
            right_pts_to_find_back_idx.push_back(i);
            // Keep the feature point itself

            right_pts_to_find.push_back(right_pts[i]);
        } else {
            vstatus[i] = 0; // a bad flow

        }
    }

    // Check that the found neighbors are unique (throw away neighbors
    // that are too close together, as they may be confusing)
    std::set<int> found_in_right_pts;

    // for each right_point see which detected feature it belongs to
    Mat right_pts_to_find_flat = Mat(right_pts_to_find).
            reshape(1,right_pts_to_find.size()); //flatten array

    vector<Point2f> right_features; // detected features
    // Convert from KeyPoints To Points
    for (unsigned int i=0; i< imgpts[right_idx].size(); i++)
        right_features.push_back(imgpts[right_idx][i].pt);

    Mat right_features_flat = Mat(right_features).reshape(1,right_features.size());


    vector<vector<DMatch> > knn_matches;

    // Look around each OF point in the right image
    // for any features that were detected in its area
    // and make a match.
    BFMatcher matcher(CV_L2);
    matcher.radiusMatch(right_pts_to_find_flat, right_features_flat
                        ,knn_matches,2.0f);


    for(int i=0;i<knn_matches.size();i++) {
        DMatch _m;
        if(knn_matches[i].size()==1) {
            _m = knn_matches[i][0]; // only one neighbor
        } else if(knn_matches[i].size()>1) {
            // 2 neighbors – check how close they are
            double ratio = knn_matches[i][0].distance / knn_matches[i][1].distance;
            if( ratio < 0.7) {// not too close
                // take the closest (first) one
                _m = knn_matches[i][0];
            } else {
                continue; // did not pass ratio test
            }
        } else {
            continue; // no match
        }
        if (found_in_right_pts
                .find(_m.trainIdx) == found_in_right_pts
                .end()) {   // prevent duplicates
            // The found neighbor was not yet used:
            // We should match it with the original indexing of the left point

            _m.queryIdx = right_pts_to_find_back_idx[_m.queryIdx]; //back to original indexing of points for <i_idx>
            matches->push_back(_m); // add this match
            found_in_right_pts
                    .insert(_m.trainIdx);
        }
    }

    cout<<"pruned "<< matches->size() <<" / "<<knn_matches.size()
       <<" matches"<<endl;
}


void reconstruct3d::ComputeFMatrixAndCorPoints()
{
    for(int prev_view=0; prev_view < imgs.size() - 1; prev_view++)
    {
        for(int cur_view = prev_view+1; cur_view < imgs.size(); cur_view++)
        {

            vector<DMatch> matches = matches_matrix[std::make_pair(prev_view,cur_view)];

            // Keep Keypoints satsifying
            vector<KeyPoint> keypttemp1;
            vector<KeyPoint> keypttemp2;

            if (matches.size() <= 0) {
                //points already aligned...
                keypttemp1 = imgpts[prev_view];
                keypttemp2 = imgpts[cur_view];
            } else {
                GetAlignedPointsFromMatch(imgpts[prev_view], imgpts[cur_view], matches, keypttemp1, keypttemp2);
            }

            vector<Point2f> pts1,pts2;
            for (unsigned int i=0; i<keypttemp1.size(); i++)
                pts1.push_back(keypttemp1[i].pt);
            for (unsigned int i=0; i<keypttemp2.size(); i++)
                pts2.push_back(keypttemp2[i].pt);

            double minVal,maxVal;
            cv::minMaxIdx(pts1,&minVal,&maxVal);

            vector<uchar> status(imgpts[prev_view].size());
            cv::Mat F = findFundamentalMat(pts1, pts2, FM_RANSAC, 0.006 * maxVal, 0.99, status); //threshold from [Snavely07 4.1]

            vector<DMatch> new_matches;

            for(unsigned int i=0; i<status.size(); i++)
            {
                if (status[i])
                {
                    imgpts_good[prev_view].push_back(keypttemp1[i]);
                    imgpts_good[cur_view].push_back(keypttemp2[i]);

                    if (matches.size() <= 0) { //points already aligned...
                        new_matches.push_back(DMatch(matches[i].queryIdx,matches[i].trainIdx,matches[i].distance));
                    }
                    else
                    {
                        new_matches.push_back(matches[i]);
                    }
                }
            }

            matches_matrix[std::make_pair(prev_view, cur_view)] = new_matches; // keep only those points who survived the fundamental matrix

            //matches_matrix[std::make_pair(working_view,older_view)] = FlipMatches(matches_matrix[std::make_pair(older_view,working_view)]);
        }
    }
}

void reconstruct3d::TriangulationOfPoints()
{
    //sort pairwise matches to find the lowest Homography inliers [Snavely07 4.2]
    cout << "Find highest match...";
    list<pair<int,pair<int,int> > > matches_sizes;

    for(std::map<std::pair<int,int> ,std::vector<cv::DMatch> >::iterator i = matches_matrix.begin(); i != matches_matrix.end(); ++i) {
        if((*i).second.size() < 100)
            matches_sizes.push_back(make_pair(100,(*i).first));
        else {
            int Hinliers = FindHomographyInliers2Views((*i).first.first,(*i).first.second);
            int percent = (int)(((double)Hinliers) / ((double)(*i).second.size()) * 100.0);
            cout << "[" << (*i).first.first << "," << (*i).first.second << " = "<<percent<<"] ";
            matches_sizes.push_back(make_pair((int)percent,(*i).first));
        }
    }
    cout << endl;
    matches_sizes.sort(sort_by_first);

    cv::Matx34d P(1,0,0,0,
                  0,1,0,0,
                  0,0,1,0),
            P1(1,0,0,0,
               0,1,0,0,
               0,0,1,0);

    std::vector<CloudPoint> tmp_pcloud;

    bool goodF = false;

    m_first_view = m_second_view = 0;
    //reverse iterate by number of matches
    for(list<pair<int,pair<int,int> > >::iterator highest_pair = matches_sizes.begin();
        highest_pair != matches_sizes.end() && !goodF;
        ++highest_pair)
    {
        m_second_view = (*highest_pair).second.second;
        m_first_view  = (*highest_pair).second.first;

        std::cout << " -------- " << imgs_names[m_first_view] << " and " << imgs_names[m_second_view] << " -------- " <<std::endl;
        //what if reconstrcution of first two views is bad? fallback to another pair
        //See if the Fundamental Matrix between these two views is good
        goodF = FindCameraMatrices(K,
                                   imgpts[m_first_view],
                                   imgpts[m_second_view],
                                   imgpts_good[m_first_view],
                                   imgpts_good[m_second_view],
                                   P,
                                   P1,
                                   matches_matrix[std::make_pair(m_first_view,m_second_view)],tmp_pcloud);

        if (goodF) {
            vector<CloudPoint> new_triangulated;
            vector<int> add_to_cloud;

            Pmats[m_first_view] = P;
            Pmats[m_second_view] = P1;

            bool good_triangulation = TriangulatePointsBetweenViews(m_second_view,m_first_view,new_triangulated,add_to_cloud);
            if(!good_triangulation || cv::countNonZero(add_to_cloud) < 10) {
                std::cout << "triangulation failed" << std::endl;
                goodF = false;
                Pmats[m_first_view] = 0;
                Pmats[m_second_view] = 0;
                m_second_view++;
            } else {
                //std::cout << "before triangulation: " << pcloud.size();
                for (unsigned int j=0; j<add_to_cloud.size(); j++) {
                    if(add_to_cloud[j] == 1)
                        pcloud.push_back(new_triangulated[j]);
                }
                //std::cout << " after " << pcloud.size() << std::endl;
            }
        }
    }
    //cout << "Taking baseline from " << imgs_names[m_first_view] << " and " << imgs_names[m_second_view] << endl;
    imageforreconstruction.push_back(imgs_names[m_first_view]);
    imageforreconstruction.push_back(imgs_names[m_second_view]);

}



bool reconstruct3d::TriangulatePointsBetweenViews(int working_view, int older_view, vector<struct CloudPoint>& new_triangulated, vector<int>& add_to_cloud)
{

    //cout << " Triangulate " << imgs_names[working_view] << " and " << imgs_names[older_view] << endl;
    //get the left camera matrix
    cv::Matx34d P = Pmats[older_view];
    cv::Matx34d P1 = Pmats[working_view];

    std::vector<cv::KeyPoint> pt_set1,pt_set2;
    std::vector<cv::DMatch> matches = matches_matrix[std::make_pair(older_view,working_view)];
    GetAlignedPointsFromMatch(imgpts[older_view],imgpts[working_view],matches,pt_set1,pt_set2);


    //adding more triangulated points to general cloud
    double reproj_error = TriangulatePoints(pt_set1, pt_set2, K, P, P1, new_triangulated);
    //std::cout << "triangulation reproj error " << reproj_error << std::endl;

    vector<uchar> trig_status;
    if(!TestTriangulation(new_triangulated, P, trig_status) || !TestTriangulation(new_triangulated, P1, trig_status)) {
        cerr << "Triangulation did not succeed" << endl;
        return false;
    }

    //filter out outlier points with high reprojection
    vector<double> reprj_errors;
    for(int i=0;i<new_triangulated.size();i++) { reprj_errors.push_back(new_triangulated[i].reprojection_error); }
    std::sort(reprj_errors.begin(),reprj_errors.end());
    //get the 80% precentile
    double reprj_err_cutoff = reprj_errors[4 * reprj_errors.size() / 5] * 2.4; //threshold from Snavely07 4.2

    vector<CloudPoint> new_triangulated_filtered;
    std::vector<cv::DMatch> new_matches;
    for(int i=0;i<new_triangulated.size();i++) {
        if(trig_status[i] == 0)
            continue; //point was not in front of camera
        if(new_triangulated[i].reprojection_error > 16.0) {
            continue; //reject point
        }
        if(new_triangulated[i].reprojection_error < 4.0 ||
                new_triangulated[i].reprojection_error < reprj_err_cutoff)
        {
            new_triangulated_filtered.push_back(new_triangulated[i]);
            new_matches.push_back(matches[i]);
        }
        else
        {
            continue;
        }
    }

    cout << "filtered out " << (new_triangulated.size() - new_triangulated_filtered.size()) << " high-error points" << endl;

    //all points filtered?
    if(new_triangulated_filtered.size() <= 0) return false;

    new_triangulated = new_triangulated_filtered;

    matches = new_matches;
    matches_matrix[std::make_pair(older_view,working_view)] = new_matches; //just to make sure, remove if unneccesary
    add_to_cloud.clear();
    add_to_cloud.resize(new_triangulated.size(),1);
    int found_other_views_count = 0;
    int num_views = imgs.size();

    //scan new triangulated points, if they were already triangulated before - strengthen cloud
    //#pragma omp parallel for num_threads(1)
    for (int j = 0; j<new_triangulated.size(); j++) {
        new_triangulated[j].imgpt_for_img = std::vector<int>(imgs.size(),-1);

        new_triangulated[j].imgpt_for_img[older_view] = matches[j].queryIdx;	//2D reference to <older_view>
        new_triangulated[j].imgpt_for_img[working_view] = matches[j].trainIdx;		//2D reference to <working_view>
        bool found_in_other_view = false;
        for (unsigned int view_ = 0; view_ < num_views; view_++) {
            if(view_ != older_view) {
                //Look for points in <view_> that match to points in <working_view>
                std::vector<cv::DMatch> submatches = matches_matrix[std::make_pair(view_,working_view)];
                for (unsigned int ii = 0; ii < submatches.size(); ii++) {
                    if (submatches[ii].trainIdx == matches[j].trainIdx &&
                            !found_in_other_view)
                    {
                        //Point was already found in <view_> - strengthen it in the known cloud, if it exists there

                        //cout << "2d pt " << submatches[ii].queryIdx << " in img " << view_ << " matched 2d pt " << submatches[ii].trainIdx << " in img " << i << endl;
                        for (unsigned int pt3d=0; pt3d<pcloud.size(); pt3d++) {
                            if (pcloud[pt3d].imgpt_for_img[view_] == submatches[ii].queryIdx)
                            {
                                pcloud[pt3d].imgpt_for_img[working_view] = matches[j].trainIdx;
                                pcloud[pt3d].imgpt_for_img[older_view] = matches[j].queryIdx;
                                found_in_other_view = true;
                                add_to_cloud[j] = 0;

                            }
                        }
                    }
                }
            }
        }
        if (found_in_other_view) {
            found_other_views_count++;
        } else {
            add_to_cloud[j] = 1;
        }

    }
    std::cout << found_other_views_count << "/" << new_triangulated.size() << " points were found in other views, adding " << cv::countNonZero(add_to_cloud) << " new\n";
    return true;
}


int reconstruct3d::FindHomographyInliers2Views(int vi, int vj)
{

    vector<cv::KeyPoint> ikpts,jkpts; vector<cv::Point2f> ipts,jpts;
    GetAlignedPointsFromMatch(imgpts[vi],imgpts[vj],matches_matrix[make_pair(vi,vj)],ikpts,jkpts);

    // change keypoints to points
    for(unsigned int i = 0; i<ikpts.size(); i++)
        ipts.push_back(ikpts[i].pt);
    for(unsigned int i = 0; i<jkpts.size(); i++)
        jpts.push_back(jkpts[i].pt);

    double minVal,maxVal; cv::minMaxIdx(ipts,&minVal,&maxVal); //TODO flatten point2d?? or it takes max of width and height

    vector<uchar> status;
    cv::Mat H = cv::findHomography(ipts,jpts,status,CV_RANSAC, 0.004 * maxVal); //threshold from Snavely07
    return cv::countNonZero(status); //number of inliers

}



void reconstruct3d::DepthComputation()
{
    // Constructor already calculates features
    ComputeFMatrixAndCorPoints();

    TriangulationOfPoints();


}
