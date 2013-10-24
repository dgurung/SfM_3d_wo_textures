#pragma once

#ifndef CAMERAINTRINSICDIALOG_H
#define CAMERAINTRINSICDIALOG_H

#include <QDialog>
#include <QString>

namespace Ui {
class CameraIntrinsicDialog;
}

class CameraIntrinsicDialog : public QDialog
{
    Q_OBJECT
    
public:
    explicit CameraIntrinsicDialog(QString title, QString desc, QString defaultValue);
    ~CameraIntrinsicDialog();
    
private:
    double *camera_matrix;

public:
    double* getCameraMatrix()
    { return camera_matrix; }

public slots:
    void updateCameraMatrix();

private:
    Ui::CameraIntrinsicDialog *ui;
};

#endif // CAMERAINTRINSICDIALOG_H
