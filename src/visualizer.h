#pragma once

#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <opencv2/core/core.hpp>
#include <iostream>
#include <fstream>
#include <string>



#include <pcl/common/common_headers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
//#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

using namespace cv;
using namespace std;

void viewerOneOff (pcl::visualization::PCLVisualizer& viewer);
boost::shared_ptr<pcl::visualization::PCLVisualizer> customColourVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
void visualize(string &filename);

#endif
