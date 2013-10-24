#-------------------------------------------------
#
# Project created by QtCreator 2013-05-25T20:06:38
#
#-------------------------------------------------

QT       += core gui

TARGET = gui
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    src/common.cpp \
    src/pretriangulation.cpp \
    src/lineartriangulation.cpp \
    src/bundleadjustment.cpp \
    src/Process3d.cpp \
    src/visualizer.cpp \
    cameraintrinsicdialog.cpp

HEADERS  += mainwindow.h \
    src/common.h \
    src/pretriangulation.h \
    src/lineartriangulation.h \
    src/bundleadjustment.h \
    src/Process3d.h \
    src/visualizer.h \
    cameraintrinsicdialog.h

FORMS    += mainwindow.ui \
    cameraintrinsicdialog.ui


LIBS += `pkg-config opencv --libs`


INCLUDEPATH +=   /usr/include/pcl-1.7/ \
                /usr/include/flann/ \
                /usr/include/eigen3/ \
                /usr/include/openni/ \
                /usr/include/boost/ \
                /usr/include/vtk-5.8/

LIBS += -lQtGui -lQtCore -lQtOpenGL \
        -lpcl_registration -lpcl_sample_consensus -lpcl_features -lpcl_filters -lpcl_surface -lpcl_segmentation \
        -lpcl_search -lpcl_kdtree -lpcl_octree -lflann_cpp -lpcl_common -lpcl_io \
        -lpcl_visualization \
        -L/usr/lib -lvtkCommon -lvtksys -lvtkViews -lvtkWidgets -lvtkRendering -lvtkGraphics -lvtkImaging -lvtkIO -lvtkFiltering -lvtkDICOMParser -lvtkmetaio -lvtkexoIIc -lvtkftgl -lvtkHybrid \
        -L/usr/lib -lboost_thread \
        -lboost_system
