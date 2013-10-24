/********************************************************************************
** Form generated from reading UI file 'CalibrationDataLoaddialog.ui'
**
** Created: Fri May 31 18:29:38 2013
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CALIBRATIONDATALOADDIALOG_H
#define UI_CALIBRATIONDATALOADDIALOG_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDialog>
#include <QtGui/QDialogButtonBox>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Dialog
{
public:
    QDialogButtonBox *buttonBox;
    QWidget *widget;
    QVBoxLayout *verticalLayout;
    QLabel *label;
    QLabel *label_7;
    QLabel *label_4;
    QWidget *widget1;
    QVBoxLayout *verticalLayout_2;
    QLabel *label_3;
    QLabel *label_2;
    QLabel *label_8;
    QWidget *widget2;
    QVBoxLayout *verticalLayout_3;
    QLabel *label_5;
    QLabel *label_6;
    QLabel *label_9;

    void setupUi(QDialog *Dialog)
    {
        if (Dialog->objectName().isEmpty())
            Dialog->setObjectName(QString::fromUtf8("Dialog"));
        Dialog->resize(400, 300);
        buttonBox = new QDialogButtonBox(Dialog);
        buttonBox->setObjectName(QString::fromUtf8("buttonBox"));
        buttonBox->setGeometry(QRect(30, 240, 341, 32));
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);
        widget = new QWidget(Dialog);
        widget->setObjectName(QString::fromUtf8("widget"));
        widget->setGeometry(QRect(16, 36, 37, 69));
        verticalLayout = new QVBoxLayout(widget);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        label = new QLabel(widget);
        label->setObjectName(QString::fromUtf8("label"));

        verticalLayout->addWidget(label);

        label_7 = new QLabel(widget);
        label_7->setObjectName(QString::fromUtf8("label_7"));

        verticalLayout->addWidget(label_7);

        label_4 = new QLabel(widget);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        verticalLayout->addWidget(label_4);

        widget1 = new QWidget(Dialog);
        widget1->setObjectName(QString::fromUtf8("widget1"));
        widget1->setGeometry(QRect(52, 36, 49, 69));
        verticalLayout_2 = new QVBoxLayout(widget1);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        verticalLayout_2->setContentsMargins(0, 0, 0, 0);
        label_3 = new QLabel(widget1);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        verticalLayout_2->addWidget(label_3);

        label_2 = new QLabel(widget1);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        verticalLayout_2->addWidget(label_2);

        label_8 = new QLabel(widget1);
        label_8->setObjectName(QString::fromUtf8("label_8"));

        verticalLayout_2->addWidget(label_8);

        widget2 = new QWidget(Dialog);
        widget2->setObjectName(QString::fromUtf8("widget2"));
        widget2->setGeometry(QRect(100, 36, 25, 69));
        verticalLayout_3 = new QVBoxLayout(widget2);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        verticalLayout_3->setContentsMargins(0, 0, 0, 0);
        label_5 = new QLabel(widget2);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        verticalLayout_3->addWidget(label_5);

        label_6 = new QLabel(widget2);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        verticalLayout_3->addWidget(label_6);

        label_9 = new QLabel(widget2);
        label_9->setObjectName(QString::fromUtf8("label_9"));

        verticalLayout_3->addWidget(label_9);


        retranslateUi(Dialog);
        QObject::connect(buttonBox, SIGNAL(accepted()), Dialog, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), Dialog, SLOT(reject()));

        QMetaObject::connectSlotsByName(Dialog);
    } // setupUi

    void retranslateUi(QDialog *Dialog)
    {
        Dialog->setWindowTitle(QApplication::translate("Dialog", "Dialog", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("Dialog", "fx", 0, QApplication::UnicodeUTF8));
        label_7->setText(QApplication::translate("Dialog", "0", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("Dialog", "0", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("Dialog", "Skew", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("Dialog", "fy", 0, QApplication::UnicodeUTF8));
        label_8->setText(QApplication::translate("Dialog", "0", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("Dialog", "u0", 0, QApplication::UnicodeUTF8));
        label_6->setText(QApplication::translate("Dialog", "v0", 0, QApplication::UnicodeUTF8));
        label_9->setText(QApplication::translate("Dialog", "1", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class Dialog: public Ui_Dialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CALIBRATIONDATALOADDIALOG_H
