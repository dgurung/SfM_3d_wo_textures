#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QFileDialog>

#include <string>
#include <QDebug>

#include "cameraintrinsicdialog.h"

MainWindow::MainWindow(QWidget *parent):
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // Connect Menu and button signals to slots
    signalSlotsInit();

    // initially disable all buttons for 3D viewing
    // These are set after loading images and calibration matrix
    ui->sfm_btn->setEnabled(false);
    ui->reconstructfrm2d_btn->setEnabled(false);
    ui->reconstructfromNd_btn->setEnabled(false);
    ui->set_camera_intrinsic_btn->setEnabled(false);
    ui->load_calibration_btn->setEnabled(false);
    ui->actionCreate_Calibration_Matrix->setEnabled(false);
    ui->actionLoad_Calibration_data->setEnabled(false);

    // For user guide
    ui->txtupdate->appendPlainText(QString("Load more than 2 images"));
}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::signalSlotsInit()
{
    // connect actions of all menu bar here
    connect(ui->actExit, SIGNAL(triggered()), this, SLOT(close()));
    connect(ui->actionLoad_Image, SIGNAL(triggered()), this, SLOT(LoadImages()));
    connect(ui->actionLoad_Calibration_data, SIGNAL(triggered()), this, SLOT(LoadCalibrationFile()));
    connect(ui->actionCreate_Calibration_Matrix, SIGNAL(triggered()), this, SLOT(LoadCameraIntrinsicMatrix()));


    // Connect actions of all buttons
    connect(ui->load_IMG_btn, SIGNAL(clicked()), this, SLOT(LoadImages()));
    connect(ui->load_calibration_btn, SIGNAL(clicked()), this, SLOT(LoadCalibrationFile()));
    connect(ui->set_camera_intrinsic_btn, SIGNAL(clicked()), this, SLOT(LoadCameraIntrinsicMatrix()));
    connect(ui->sfm_btn, SIGNAL(clicked()), this, SLOT(sfm_compute()));
    connect(ui->reconstructfrm2d_btn, SIGNAL(clicked()), this, SLOT(visualizefrom2d()));
    connect(ui->reconstructfromNd_btn, SIGNAL(clicked()), this, SLOT(visualizefromNd()));

} // signalSlotsInit()


void MainWindow::LoadCalibrationFile()
{
    QString fileName = QFileDialog::getOpenFileName(this,  tr("Open Calibration data"), ".", tr("XML files (*.xml)"));
    std::string pointcloudxml =  fileName.toUtf8().constData();

    if(pointcloudxml.empty())
        ui->txtupdate->appendPlainText(QString("Calibration data NOT loaded. Please Try Again"));

    qDebug() << fileName;

    //use opencv function to load fileName
    FileStorage fs(pointcloudxml, FileStorage::READ);
    Mat calibParam = Mat::eye(3,3,CV_32FC1);;
    if(!fs.isOpened())
        calibParam = Mat::eye(3,3,CV_32FC1);
    else
    {
        fs["Camera_Intra_Param"] >> calibParam;

        if(calibParam.rows==3 && calibParam.cols ==3)
        {
            camera_intrinsic = new double[9];
            // convert cv::Mat to 1D array.
            // This is because camera_intrinsic is a 1D array used in computation.
            int k=0;
            for(int i=0; i<3; i++ ){
                for(int j=0; j<3; j++){
                    camera_intrinsic[k] =  calibParam.at<double>(i,j);
                    qDebug() << "Load SEction:" << camera_intrinsic[k];
                    k++;
                }
            }
            //enable sfm computation
            ui->txtupdate->appendPlainText(QString("Calibration data loaded. Now you can compute SfM. SfM computation may take some time depending on number of images."));
            ui->sfm_btn->setEnabled(true);
        }
        else
            ui->txtupdate->appendPlainText(QString("Please Load xml that is recognizible"));
    }
}

void MainWindow::LoadCameraIntrinsicMatrix()
{
    CameraIntrinsicDialog *inp = new CameraIntrinsicDialog("Select Camera Intrinsic Parameters", "Please enter camera intrinsic", this->windowTitle());
    if(inp->exec() == QDialog::Accepted) {
        camera_intrinsic = new double[9];
        camera_intrinsic = inp->getCameraMatrix();
    }
    delete inp;

    for(int i=0; i<9; i++)
        qDebug() << i << '\t' << camera_intrinsic[i];

    ui->sfm_btn->setEnabled(true);
}


void MainWindow::LoadImages()
{
    QFileDialog dialog(this);
    dialog.setDirectory(QDir::homePath());
    dialog.setFileMode(QFileDialog::ExistingFiles);
    dialog.setNameFilter(trUtf8("Splits (*.JPG *.png)"));
    QStringList fileNames;
    if (dialog.exec())
        fileNames = dialog.selectedFiles();

    qDebug() << fileNames;

    float downscale_factor = 1.0;

    imgs_names.clear();
    imgs.clear();

    for (unsigned int i=0; i<fileNames.size(); i++) {
        // convert QString to string
        imgs_names.push_back( fileNames[i].toUtf8().constData() );
        cv::Mat m_ = cv::imread(std::string(imgs_names.back()));
        if(downscale_factor != 1.0)
            cv::resize(m_,m_,cv::Size(),downscale_factor,downscale_factor);
        imgs.push_back(m_);
    }
    if(!imgs.empty())
    {
        ui->txtupdate->appendPlainText(QString("Image Loaded. Now load calibration data..."));
        ui->load_calibration_btn->setEnabled(true);
        ui->actionCreate_Calibration_Matrix->setEnabled(true);
        ui->actionLoad_Calibration_data->setEnabled(true);
        ui->set_camera_intrinsic_btn->setEnabled(true);
    }
}


void MainWindow::sfm_compute()
{
    qDebug() << "Process start";

    ui->sfm_btn->setEnabled(false);
    ui->txtupdate->appendPlainText(QString("SfM Process Started..."));

    /************ Compute SfM *************/
    // use constructor method
//    for(int i=0; i<9; i++)
//        qDebug() << i << '\t' << camera_intrinsic[i];
    Process3d computation3D(imgs, camera_intrinsic);

    ui->txtupdate->appendPlainText(QString("SfM Process Completed..."));
    ui->sfm_btn->setEnabled(true);
    ui->reconstructfrm2d_btn->setEnabled(true);
    ui->reconstructfromNd_btn->setEnabled(true);
}


void MainWindow::visualizefrom2d()
{
    std::string fileName = "CloudPoints0000.xml";
    visualize(fileName);
    //ui->reconstructfrm2d_btn->setEnabled(false);
    //ui->reconstructfromNd_btn->setEnabled(false);
}

void MainWindow::visualizefromNd()
{
    std::string fileName = "CloudPoints0100.xml";
    visualize(fileName);
    //ui->reconstructfromNd_btn->setEnabled(false);
    //ui->reconstructfrm2d_btn->setEnabled(false);

}

