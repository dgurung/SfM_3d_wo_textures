#include "visualizer.h"

using namespace cv;
using namespace std;

void visualize(){
    cout << endl << "Reading: " << endl;
    FileStorage fs("PointCloudFoun.xml ", FileStorage::READ ); //PointCloud_SURF_Foun_1K.xml

    if (fs.isOpened())
        cout<<"File is opened\n";
    else
        cout << "error in opening" << endl;

    FileNode n = fs["CloudPoint"];

    int row;
    row = (int) (n["rows"]);

    cout << "Row:" << row << endl;

    unsigned int count;

    FileNode nd = n["data"];

    vector< double > pointcloud;

    FileNodeIterator it = nd.begin(), it_end = nd.end(); // Go through the node
    for (; it != it_end; ++it)
    {
        pointcloud.push_back( (double)*it );
        // cout << (double)*it;
        count += 1;
    }

    cout << count/3 <<endl;
    fs.release();

    cout << "\n" << pointcloud.size()/3 <<endl;
    //--------------------
    // Visualization
    //--------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    std::cout << "Genarating point clouds.\n\n";

    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

    for(unsigned int i = 0; i < pointcloud.size() ; i = i+3)
    {

        if(isnan( pointcloud[i] ) )
            continue;

        pcl::PointXYZ p;

        p.x = pointcloud[i];
        p.y = pointcloud[i+1];
        p.z = pointcloud[i+2];

        cloud->push_back(p);
        //cloud->points.push_back(p);
    }

    cloud->width = (int) cloud->points.size();
    cloud->height = 1;

    for (size_t i = 0; i < cloud->points.size (); ++i)
        std::cerr << "    " << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << std::endl;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = customColourVis(cloud);
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

}



boost::shared_ptr<pcl::visualization::PCLVisualizer> customColourVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}




void viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor (1.0, 0.5, 1.0);
    pcl::PointXYZ o;
    o.x = 0;
    o.y = 0;
    o.z = 0;
    viewer.addSphere (o, 0.25, "sphere", 0);
    std::cout << "i only run once" << std::endl;
}
