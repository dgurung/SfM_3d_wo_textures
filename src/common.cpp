#pragma once

#include "common.h"


#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <cstring>
#include <stdlib.h>
#include <fstream>

using namespace std;

/* Function to write point cloud in .txt file */
void WritePointClouds(std::vector<Point3d> pointCloud,int ImgNo){

    char NAME[200];
    sprintf(NAME,"CloudPoints%.4d.txt",ImgNo);
    FILE * my3dPoints;
    long int i;
    my3dPoints = fopen(NAME,"w+");
    for(i=0; i<pointCloud.size(); i++)
            fprintf(my3dPoints,"%f %f %f\n",(pointCloud)[i].x ,(pointCloud)[i].y ,(pointCloud)[i].z );
    fclose(my3dPoints);
}
/* Function to write point cloud in .XML file */
void WritePointCloudsXML(std::vector<Point3d> pointCloud,int ImgNo){

    char NAME[200];
    sprintf(NAME,"CloudPoints%.4d.xml",ImgNo);
    FileStorage fs(NAME, FileStorage::WRITE); //declare xml output file
    Mat new_cloudPoint(pointCloud); //convert vector to Mat
    fs << "CloudPoint" << new_cloudPoint; //save Mat
    fs.release(); //release output file
}
/* Function to write matching points in .txt file */
void WriteMatches(std::vector<Point2f> FeaturePts1,std::vector<Point2f> FeaturePts2,int ImgNo){

    char NAME1[200];
    sprintf(NAME1,"ImgPtsLeft%.4d.txt",ImgNo);
    ofstream File1;
    File1.open (NAME1);
    File1 << FeaturePts1;
    File1.close();

    char NAME2[200];
    sprintf(NAME2,"ImgPtsRight%.4d.txt",ImgNo);
    ofstream File2;
    File2.open (NAME2);
    File2 << FeaturePts2;
    File2.close();
}
