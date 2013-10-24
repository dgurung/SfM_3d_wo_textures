#include "cameraintrinsicdialog.h"
#include "ui_cameraintrinsicdialog.h"

#include <QDebug>

CameraIntrinsicDialog::CameraIntrinsicDialog(QString title, QString desc, QString defaultValue):

    ui(new Ui::CameraIntrinsicDialog)
{
    ui->setupUi(this);
    this->setWindowTitle(title);

    connect(ui->fx_in,SIGNAL(textChanged(QString)), this, SLOT(updateCameraMatrix()));
    connect(ui->skewn_in,SIGNAL(textChanged(QString)), this, SLOT(updateCameraMatrix()));
    connect(ui->u0_in,SIGNAL(textChanged(QString)), this, SLOT(updateCameraMatrix()));
    connect(ui->no_in21,SIGNAL(textChanged(QString)), this, SLOT(updateCameraMatrix()));
    connect(ui->fy_in,SIGNAL(textChanged(QString)), this, SLOT(updateCameraMatrix()));
    connect(ui->v0_in,SIGNAL(textChanged(QString)), this, SLOT(updateCameraMatrix()));
    connect(ui->no_in31,SIGNAL(textChanged(QString)), this, SLOT(updateCameraMatrix()));
    connect(ui->no_in32,SIGNAL(textChanged(QString)), this, SLOT(updateCameraMatrix()));
    connect(ui->no_in33,SIGNAL(textChanged(QString)), this, SLOT(updateCameraMatrix()));
}

CameraIntrinsicDialog::~CameraIntrinsicDialog()
{
    delete ui;
}

void CameraIntrinsicDialog::updateCameraMatrix()
{
    camera_matrix = new double[9];
    camera_matrix[0] = ((ui->fx_in->text())).toDouble();
    camera_matrix[1] = ((ui->skewn_in->text())).toDouble();
    camera_matrix[2] = ((ui->u0_in->text())).toDouble();
    camera_matrix[3] = ((ui->no_in21->text())).toDouble();
    camera_matrix[4] = ((ui->fy_in->text())).toDouble();
    camera_matrix[5] = ((ui->v0_in->text())).toDouble();
    camera_matrix[6] = ((ui->no_in31->text())).toDouble();
    camera_matrix[7] = ((ui->no_in32->text())).toDouble();
    camera_matrix[8] = ((ui->no_in33->text())).toDouble();

//    qDebug()<< "from dialog box: " << ui->fx_in->text();
//    for(int i =0; i<9; i++)
//    {
//    qDebug( "%f", camera_matrix[i] );
//    }
}
