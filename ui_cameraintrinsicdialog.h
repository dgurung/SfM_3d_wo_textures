/********************************************************************************
** Form generated from reading UI file 'cameraintrinsicdialog.ui'
**
** Created: Thu Oct 10 21:09:48 2013
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CAMERAINTRINSICDIALOG_H
#define UI_CAMERAINTRINSICDIALOG_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDialog>
#include <QtGui/QDialogButtonBox>
#include <QtGui/QGridLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_CameraIntrinsicDialog
{
public:
    QDialogButtonBox *buttonBox;
    QLabel *label_10;
    QWidget *layoutWidget;
    QGridLayout *gridLayout_2;
    QVBoxLayout *verticalLayout;
    QLabel *label;
    QLabel *label_7;
    QLabel *label_4;
    QVBoxLayout *verticalLayout_2;
    QLabel *label_3;
    QLabel *label_2;
    QLabel *label_8;
    QVBoxLayout *verticalLayout_3;
    QLabel *label_5;
    QLabel *label_6;
    QLabel *label_9;
    QWidget *layoutWidget_2;
    QGridLayout *gridLayout;
    QLineEdit *fx_in;
    QLineEdit *skewn_in;
    QLineEdit *u0_in;
    QLineEdit *fy_in;
    QLineEdit *v0_in;
    QLineEdit *no_in31;
    QLineEdit *no_in32;
    QLineEdit *no_in33;
    QLineEdit *no_in21;

    void setupUi(QDialog *CameraIntrinsicDialog)
    {
        if (CameraIntrinsicDialog->objectName().isEmpty())
            CameraIntrinsicDialog->setObjectName(QString::fromUtf8("CameraIntrinsicDialog"));
        CameraIntrinsicDialog->resize(426, 218);
        buttonBox = new QDialogButtonBox(CameraIntrinsicDialog);
        buttonBox->setObjectName(QString::fromUtf8("buttonBox"));
        buttonBox->setGeometry(QRect(72, 168, 341, 32));
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);
        label_10 = new QLabel(CameraIntrinsicDialog);
        label_10->setObjectName(QString::fromUtf8("label_10"));
        label_10->setGeometry(QRect(16, 20, 277, 17));
        layoutWidget = new QWidget(CameraIntrinsicDialog);
        layoutWidget->setObjectName(QString::fromUtf8("layoutWidget"));
        layoutWidget->setGeometry(QRect(20, 52, 137, 101));
        gridLayout_2 = new QGridLayout(layoutWidget);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        gridLayout_2->setContentsMargins(0, 0, 0, 0);
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        label = new QLabel(layoutWidget);
        label->setObjectName(QString::fromUtf8("label"));

        verticalLayout->addWidget(label);

        label_7 = new QLabel(layoutWidget);
        label_7->setObjectName(QString::fromUtf8("label_7"));

        verticalLayout->addWidget(label_7);

        label_4 = new QLabel(layoutWidget);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        verticalLayout->addWidget(label_4);


        gridLayout_2->addLayout(verticalLayout, 0, 0, 1, 1);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        label_3 = new QLabel(layoutWidget);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        verticalLayout_2->addWidget(label_3);

        label_2 = new QLabel(layoutWidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        verticalLayout_2->addWidget(label_2);

        label_8 = new QLabel(layoutWidget);
        label_8->setObjectName(QString::fromUtf8("label_8"));

        verticalLayout_2->addWidget(label_8);


        gridLayout_2->addLayout(verticalLayout_2, 0, 1, 1, 1);

        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        label_5 = new QLabel(layoutWidget);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        verticalLayout_3->addWidget(label_5);

        label_6 = new QLabel(layoutWidget);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        verticalLayout_3->addWidget(label_6);

        label_9 = new QLabel(layoutWidget);
        label_9->setObjectName(QString::fromUtf8("label_9"));

        verticalLayout_3->addWidget(label_9);


        gridLayout_2->addLayout(verticalLayout_3, 0, 2, 1, 1);

        layoutWidget_2 = new QWidget(CameraIntrinsicDialog);
        layoutWidget_2->setObjectName(QString::fromUtf8("layoutWidget_2"));
        layoutWidget_2->setGeometry(QRect(168, 52, 245, 101));
        gridLayout = new QGridLayout(layoutWidget_2);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        gridLayout->setContentsMargins(0, 0, 0, 0);
        fx_in = new QLineEdit(layoutWidget_2);
        fx_in->setObjectName(QString::fromUtf8("fx_in"));

        gridLayout->addWidget(fx_in, 0, 0, 1, 1);

        skewn_in = new QLineEdit(layoutWidget_2);
        skewn_in->setObjectName(QString::fromUtf8("skewn_in"));

        gridLayout->addWidget(skewn_in, 0, 1, 1, 1);

        u0_in = new QLineEdit(layoutWidget_2);
        u0_in->setObjectName(QString::fromUtf8("u0_in"));

        gridLayout->addWidget(u0_in, 0, 2, 1, 1);

        fy_in = new QLineEdit(layoutWidget_2);
        fy_in->setObjectName(QString::fromUtf8("fy_in"));

        gridLayout->addWidget(fy_in, 1, 1, 1, 1);

        v0_in = new QLineEdit(layoutWidget_2);
        v0_in->setObjectName(QString::fromUtf8("v0_in"));

        gridLayout->addWidget(v0_in, 1, 2, 1, 1);

        no_in31 = new QLineEdit(layoutWidget_2);
        no_in31->setObjectName(QString::fromUtf8("no_in31"));

        gridLayout->addWidget(no_in31, 2, 0, 1, 1);

        no_in32 = new QLineEdit(layoutWidget_2);
        no_in32->setObjectName(QString::fromUtf8("no_in32"));

        gridLayout->addWidget(no_in32, 2, 1, 1, 1);

        no_in33 = new QLineEdit(layoutWidget_2);
        no_in33->setObjectName(QString::fromUtf8("no_in33"));

        gridLayout->addWidget(no_in33, 2, 2, 1, 1);

        no_in21 = new QLineEdit(layoutWidget_2);
        no_in21->setObjectName(QString::fromUtf8("no_in21"));

        gridLayout->addWidget(no_in21, 1, 0, 1, 1);


        retranslateUi(CameraIntrinsicDialog);
        QObject::connect(buttonBox, SIGNAL(accepted()), CameraIntrinsicDialog, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), CameraIntrinsicDialog, SLOT(reject()));

        QMetaObject::connectSlotsByName(CameraIntrinsicDialog);
    } // setupUi

    void retranslateUi(QDialog *CameraIntrinsicDialog)
    {
        CameraIntrinsicDialog->setWindowTitle(QApplication::translate("CameraIntrinsicDialog", "Dialog", 0, QApplication::UnicodeUTF8));
        label_10->setText(QApplication::translate("CameraIntrinsicDialog", "Set Camera Intrinsic Parameters", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("CameraIntrinsicDialog", "fx", 0, QApplication::UnicodeUTF8));
        label_7->setText(QApplication::translate("CameraIntrinsicDialog", "0", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("CameraIntrinsicDialog", "0", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("CameraIntrinsicDialog", "Skew", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("CameraIntrinsicDialog", "fy", 0, QApplication::UnicodeUTF8));
        label_8->setText(QApplication::translate("CameraIntrinsicDialog", "0", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("CameraIntrinsicDialog", "u0", 0, QApplication::UnicodeUTF8));
        label_6->setText(QApplication::translate("CameraIntrinsicDialog", "v0", 0, QApplication::UnicodeUTF8));
        label_9->setText(QApplication::translate("CameraIntrinsicDialog", "1", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class CameraIntrinsicDialog: public Ui_CameraIntrinsicDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CAMERAINTRINSICDIALOG_H
