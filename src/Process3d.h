#ifndef Process3d_H
#define Process3d_H

#include "pretriangulation.h"
#include "lineartriangulation.h"
#include "bundleadjustment.h"
#include "common.h"


#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <cstring>
#include <stdlib.h>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/contrib/contrib.hpp>

class Process3d{
public:
    Process3d();
    Process3d(std::vector<cv::Mat> &images, double *camera_intrinsic);
    void FindHomographyInliers2Views();

public:
    std::vector< cv::Mat > images_Process3d;
};

#endif // Process3d_H
