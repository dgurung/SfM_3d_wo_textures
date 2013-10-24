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

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "utilitycode.h"
#include "Triangulation.h"

using namespace std;

bool CheckCoherentRotation(cv::Mat_<double>& R);
bool TestTriangulation(const std::vector<CloudPoint>& pcloud, const cv::Matx34d& P, std::vector<uchar>& status);

cv::Mat GetFundamentalMat(	const std::vector<cv::KeyPoint>& imgpts1,
							const std::vector<cv::KeyPoint>& imgpts2,
							std::vector<cv::KeyPoint>& imgpts1_good,
							std::vector<cv::KeyPoint>& imgpts2_good,
                            std::vector<cv::DMatch>& matches);

bool FindCameraMatrices(const cv::Mat& K, 
                        const vector<cv::KeyPoint> &imgpts1,
                        const vector<cv::KeyPoint> &imgpts2,
                        vector<cv::KeyPoint> &imgpts1_good,
                        vector<cv::KeyPoint> &imgpts2_good,
                        cv::Matx34d &P,
                        cv::Matx34d &P1,
                        vector<cv::DMatch> &matches,
                        vector<CloudPoint> &outCloud
						);
