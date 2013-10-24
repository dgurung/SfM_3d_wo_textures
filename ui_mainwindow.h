/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created: Thu Oct 10 21:09:48 2013
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QPlainTextEdit>
#include <QtGui/QPushButton>
#include <QtGui/QStatusBar>
#include <QtGui/QToolBar>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actExit;
    QAction *actionLoad_Image;
    QAction *actionLoad_Calibration_data;
    QAction *actionCreate_Calibration_Matrix;
    QWidget *centralWidget;
    QPlainTextEdit *txtupdate;
    QWidget *layoutWidget;
    QVBoxLayout *verticalLayout_2;
    QLabel *label_2;
    QPushButton *reconstructfrm2d_btn;
    QPushButton *reconstructfromNd_btn;
    QWidget *widget;
    QVBoxLayout *verticalLayout_3;
    QLabel *label_3;
    QPushButton *sfm_btn;
    QWidget *widget1;
    QVBoxLayout *verticalLayout_5;
    QVBoxLayout *verticalLayout;
    QLabel *label;
    QPushButton *load_IMG_btn;
    QWidget *widget2;
    QVBoxLayout *verticalLayout_4;
    QLabel *label_4;
    QPushButton *load_calibration_btn;
    QPushButton *set_camera_intrinsic_btn;
    QMenuBar *menuBar;
    QMenu *menuMenu;
    QMenu *menuCalibration;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(532, 568);
        actExit = new QAction(MainWindow);
        actExit->setObjectName(QString::fromUtf8("actExit"));
        actionLoad_Image = new QAction(MainWindow);
        actionLoad_Image->setObjectName(QString::fromUtf8("actionLoad_Image"));
        actionLoad_Calibration_data = new QAction(MainWindow);
        actionLoad_Calibration_data->setObjectName(QString::fromUtf8("actionLoad_Calibration_data"));
        actionCreate_Calibration_Matrix = new QAction(MainWindow);
        actionCreate_Calibration_Matrix->setObjectName(QString::fromUtf8("actionCreate_Calibration_Matrix"));
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        txtupdate = new QPlainTextEdit(centralWidget);
        txtupdate->setObjectName(QString::fromUtf8("txtupdate"));
        txtupdate->setGeometry(QRect(32, 280, 445, 217));
        layoutWidget = new QWidget(centralWidget);
        layoutWidget->setObjectName(QString::fromUtf8("layoutWidget"));
        layoutWidget->setGeometry(QRect(340, 104, 171, 85));
        verticalLayout_2 = new QVBoxLayout(layoutWidget);
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setContentsMargins(11, 11, 11, 11);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        verticalLayout_2->setContentsMargins(0, 0, 0, 0);
        label_2 = new QLabel(layoutWidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        verticalLayout_2->addWidget(label_2);

        reconstructfrm2d_btn = new QPushButton(layoutWidget);
        reconstructfrm2d_btn->setObjectName(QString::fromUtf8("reconstructfrm2d_btn"));

        verticalLayout_2->addWidget(reconstructfrm2d_btn);

        reconstructfromNd_btn = new QPushButton(layoutWidget);
        reconstructfromNd_btn->setObjectName(QString::fromUtf8("reconstructfromNd_btn"));

        verticalLayout_2->addWidget(reconstructfromNd_btn);

        widget = new QWidget(centralWidget);
        widget->setObjectName(QString::fromUtf8("widget"));
        widget->setGeometry(QRect(140, 212, 253, 52));
        verticalLayout_3 = new QVBoxLayout(widget);
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setContentsMargins(11, 11, 11, 11);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        verticalLayout_3->setContentsMargins(0, 0, 0, 0);
        label_3 = new QLabel(widget);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        verticalLayout_3->addWidget(label_3);

        sfm_btn = new QPushButton(widget);
        sfm_btn->setObjectName(QString::fromUtf8("sfm_btn"));

        verticalLayout_3->addWidget(sfm_btn);

        widget1 = new QWidget(centralWidget);
        widget1->setObjectName(QString::fromUtf8("widget1"));
        widget1->setGeometry(QRect(16, 12, 225, 61));
        verticalLayout_5 = new QVBoxLayout(widget1);
        verticalLayout_5->setSpacing(6);
        verticalLayout_5->setContentsMargins(11, 11, 11, 11);
        verticalLayout_5->setObjectName(QString::fromUtf8("verticalLayout_5"));
        verticalLayout_5->setContentsMargins(0, 0, 0, 0);
        verticalLayout = new QVBoxLayout();
        verticalLayout->setSpacing(6);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        label = new QLabel(widget1);
        label->setObjectName(QString::fromUtf8("label"));

        verticalLayout->addWidget(label);

        load_IMG_btn = new QPushButton(widget1);
        load_IMG_btn->setObjectName(QString::fromUtf8("load_IMG_btn"));

        verticalLayout->addWidget(load_IMG_btn);


        verticalLayout_5->addLayout(verticalLayout);

        widget2 = new QWidget(centralWidget);
        widget2->setObjectName(QString::fromUtf8("widget2"));
        widget2->setGeometry(QRect(16, 100, 285, 89));
        verticalLayout_4 = new QVBoxLayout(widget2);
        verticalLayout_4->setSpacing(6);
        verticalLayout_4->setContentsMargins(11, 11, 11, 11);
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        verticalLayout_4->setContentsMargins(0, 0, 0, 0);
        label_4 = new QLabel(widget2);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        verticalLayout_4->addWidget(label_4);

        load_calibration_btn = new QPushButton(widget2);
        load_calibration_btn->setObjectName(QString::fromUtf8("load_calibration_btn"));

        verticalLayout_4->addWidget(load_calibration_btn);

        set_camera_intrinsic_btn = new QPushButton(widget2);
        set_camera_intrinsic_btn->setObjectName(QString::fromUtf8("set_camera_intrinsic_btn"));

        verticalLayout_4->addWidget(set_camera_intrinsic_btn);

        MainWindow->setCentralWidget(centralWidget);
        layoutWidget->raise();
        layoutWidget->raise();
        sfm_btn->raise();
        txtupdate->raise();
        label->raise();
        label_2->raise();
        label_3->raise();
        layoutWidget->raise();
        layoutWidget->raise();
        set_camera_intrinsic_btn->raise();
        load_calibration_btn->raise();
        label_4->raise();
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 532, 25));
        menuMenu = new QMenu(menuBar);
        menuMenu->setObjectName(QString::fromUtf8("menuMenu"));
        menuCalibration = new QMenu(menuBar);
        menuCalibration->setObjectName(QString::fromUtf8("menuCalibration"));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow->setStatusBar(statusBar);

        menuBar->addAction(menuMenu->menuAction());
        menuBar->addAction(menuCalibration->menuAction());
        menuMenu->addAction(actionLoad_Image);
        menuMenu->addAction(actionLoad_Calibration_data);
        menuMenu->addAction(actExit);
        menuCalibration->addAction(actionCreate_Calibration_Matrix);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0, QApplication::UnicodeUTF8));
        actExit->setText(QApplication::translate("MainWindow", "Exit", 0, QApplication::UnicodeUTF8));
        actionLoad_Image->setText(QApplication::translate("MainWindow", "Load Images ...", 0, QApplication::UnicodeUTF8));
        actionLoad_Calibration_data->setText(QApplication::translate("MainWindow", "Load Camera Parameter Matrix", 0, QApplication::UnicodeUTF8));
        actionCreate_Calibration_Matrix->setText(QApplication::translate("MainWindow", "Create Camera Matrix ...", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("MainWindow", "Visualise", 0, QApplication::UnicodeUTF8));
        reconstructfrm2d_btn->setText(QApplication::translate("MainWindow", "3D from Two views", 0, QApplication::UnicodeUTF8));
        reconstructfromNd_btn->setText(QApplication::translate("MainWindow", "3D from Multiple View", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("MainWindow", "Compute", 0, QApplication::UnicodeUTF8));
        sfm_btn->setText(QApplication::translate("MainWindow", "Compute SfM", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("MainWindow", "Load Data", 0, QApplication::UnicodeUTF8));
        load_IMG_btn->setText(QApplication::translate("MainWindow", "Load Images ", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("MainWindow", "Load Calibration Matrix", 0, QApplication::UnicodeUTF8));
        load_calibration_btn->setText(QApplication::translate("MainWindow", "Load Camera intrinsic parameters (xml)", 0, QApplication::UnicodeUTF8));
        set_camera_intrinsic_btn->setText(QApplication::translate("MainWindow", "Set Camera Intrinsic", 0, QApplication::UnicodeUTF8));
        menuMenu->setTitle(QApplication::translate("MainWindow", "Menu", 0, QApplication::UnicodeUTF8));
        menuCalibration->setTitle(QApplication::translate("MainWindow", "Set Camera Intrinsic", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
