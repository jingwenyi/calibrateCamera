#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <dirent.h>
#include <string.h>



using namespace std;
using namespace cv;

#define INTRINSICS_FILE     "../share_file/calibration_result.txt"
#define SOURCE_FLIE_DIR     "calibrate_need"
#define TARGET_FLIE_DIR     "calibrate_result"



int main()
{
/*
**1.读取相机的内参
**
*/
    Mat cameraMatrix, distCoeffs;
    FileStorage fs;
    fs.open(INTRINSICS_FILE, FileStorage::READ);

    if(!fs.isOpened())
    {
        std::cout<<"failed to open file "<<INTRINSICS_FILE<<"\n"<<endl;
        return -1;
    }

    fs["M"]>>cameraMatrix;
    fs["D"]>>distCoeffs;

    std::cout<<"read camera intrinsics is:\n"<<endl;
    std::cout<<"M:"<<cameraMatrix<<"\n"<<endl;;
    std::cout<<"D:"<<distCoeffs<<"\n"<<endl;

/*
**2.遍历源文件下的所有相片，进行校正
**
*/
    DIR *dir;
    struct dirent * ptr;
    string imageFileName;
    
    dir = opendir(SOURCE_FLIE_DIR);
    if(dir == NULL)
    {
           std::cout<<"faild to open "<<SOURCE_FLIE_DIR<<"\n"<<endl;
           return -1;
    }

    while((ptr = readdir(dir)) != NULL)
    {
        if(strcmp(ptr->d_name, ".") == 0
            || strcmp(ptr->d_name, "..") == 0
            || strcmp(ptr->d_name, "readme.txt") == 0)
        {
            continue;
        }

        std::cout<<"Calibration photo: "<<ptr->d_name<<"...\n"<<endl;
        imageFileName.clear();
        imageFileName += SOURCE_FLIE_DIR;
        imageFileName += "/";
        imageFileName += ptr->d_name;
        
        Mat t = imread(imageFileName);
        Mat newimage = t.clone();
        Size image_size = t.size();
        Mat mapx = Mat(image_size, CV_32FC1);
        Mat mapy = Mat(image_size, CV_32FC1);
        Mat R = Mat::eye(3,3,CV_32F);

        Mat newCameraMatrix = Mat(3,3,CV_32FC1, Scalar::all(0));
        initUndistortRectifyMap(cameraMatrix, distCoeffs, R, cameraMatrix, image_size, CV_32FC1, mapx, mapy);
        
        remap(t, newimage, mapx, mapy, INTER_LINEAR);
        
        imageFileName.clear();
        imageFileName += TARGET_FLIE_DIR;
        imageFileName += "/";
        imageFileName += ptr->d_name;
        
        imwrite(imageFileName,newimage);
    }
}

