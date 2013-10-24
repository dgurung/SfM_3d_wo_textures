#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


#include "ui_mainwindow.h"


#include "src/Process3d.h"
#include "src/visualizer.h"


namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    
public slots:
    void signalSlotsInit();
    void LoadCalibrationFile();
    void LoadCameraIntrinsicMatrix();
    void LoadImages();
    void sfm_compute();
    void visualizefrom2d();
    void visualizefromNd();

private:
    /// Field for image variables
    std::vector<cv::Mat_<cv::Vec3b> > imgs_orig;
    std::vector<cv::Mat> imgs;
    std::vector<std::string> imgs_names;
    double *camera_intrinsic;

private:
    Ui::MainWindow *ui;

};

#endif // MAINWINDOW_H
