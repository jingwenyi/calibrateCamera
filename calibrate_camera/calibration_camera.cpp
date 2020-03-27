#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>


using namespace std;
using namespace cv;


#define INTRINSICS_FILE   "../share_file/calibration_result.txt"
#define RESOURCE_FLIE_DIR   "resource/"
#define RESULT_FLIE_DIR     "result/"

int main()
{
    //校准使用的图片数
    int image_count = 30;
    Mat frame, ResizeImage;
    Size image_size;

    //校准使用的棋盘
    Size board_size = Size(8,5);

    //由于图片太大，需要缩放的比例
    //黑卡相机像素太高，校准太近会导致
    //角点无法识别( 原因是角点占的像素太多 )， 所有需要缩小
    double fscale = 0.2;
    //棋盘的大小 单位:mm
    double fSize_w = 150;
    double fSize_h = 150;
    Size outSize;
    bool use_SubPix = false;


    vector<Point2f> corners;
    vector<vector<Point2f> > corners_Seq;


/*
**  1.获取每个校准图片的角点坐标
**
*/

    std::cout <<"found corners...\n"<<endl;
    //读取每张图片
    int count = 0,n = 0;
    stringstream tempname;
	string filename;
    string temp_filename;

    while(n < image_count)
    {
        frame.setTo(0);
        filename += RESOURCE_FLIE_DIR;
        n++;
        tempname << n;
        tempname >> temp_filename;
        filename += temp_filename;
        filename += ".JPG";

        frame = imread(filename, IMREAD_COLOR);
        temp_filename.clear();
        tempname.clear();
        filename.clear();

        //缩放图片
        outSize.width = frame.cols * fscale;
        outSize.height = frame.rows * fscale;
        resize(frame, ResizeImage, outSize, 0, 0, INTER_AREA);

        
        image_size = ResizeImage.size();

        Mat imageGray;
        cvtColor(ResizeImage, imageGray, CV_RGB2GRAY);
        bool patternfound = findChessboardCorners(ResizeImage, board_size, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
        //找到图片的角点，如果有一个没有找到无法继续，可以调小缩放值
        if(patternfound == false){
            std::cout<<"the: "<<n<<" photo corners is error\r\n"<<endl;
            return -1;
        }else{
            std::cout<<"the: "<<n<<" photo corners is ok\r\n"<<endl;
        }

        

        
        //亚像素精确化,效果不好
        if(use_SubPix)
        {
            cornerSubPix(imageGray, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
            count += corners.size();
            corners_Seq.push_back(corners);
        }

        //把角点放大到正常的图片中去
        if(patternfound == true){
            for(int i=0; i<board_size.width; i++){
                for(int j=0; j <board_size.height; j++){
                    corners[j * board_size.width + i].x = corners[j * board_size.width + i].x / fscale;
                    corners[j * board_size.width + i].y = corners[j * board_size.width + i].y / fscale;
                }
            }
        }

        corners_Seq.push_back(corners);
    }



/*
**2.用获取到的角点，标定相机，输出标定文件
**
*/

    std::cout<<"Angular point extraction is complete, start calibration\n"<<endl;


    //恢复size 的大小
    image_size = frame.size();
    //每个棋盘格的实际大小
    Size square_size = Size(fSize_w, fSize_h);
    //保存定标板上角点的三维坐标
    vector<vector<Point3f> >     object_Points;
    //每幅图像中的角点数量
    vector<int> point_counts;
    //摄像机的内参矩阵
    Mat intrinsic_matrix = Mat(3, 3, CV_32FC1, Scalar::all(0));
    // 摄像机的5个畸变系数：k1,k2,p1,p2,k3 
    Mat distortion_coeffs = Mat(1, 5, CV_32FC1, Scalar::all(0));
    //每幅图像的旋转向量
    vector<Mat> rotation_vectors;
    //每幅图像的平移向量
    vector<Mat> translation_vectors;

    //把角点放到z=0的3位坐标中去
    for(int t=0; t<image_count; t++)
    {
        vector<Point3f> tempPointSet;
        for(int i=0; i<board_size.height; i++)
        {
            for(int j=0; j<board_size.width; j++)
            {
                Point3f tempPoint;
                tempPoint.x = i * square_size.width;
                tempPoint.y = j * square_size.height;
                tempPoint.z = 0;
                tempPointSet.push_back(tempPoint);
            }
        }

        object_Points.push_back(tempPointSet);
        
    }


    //每幅图像都能考到所有的角点
    for(int i=0; i<image_count; i++)
    {
        point_counts.push_back(board_size.width * board_size.height);
    }


    //执行标定函数
    calibrateCamera(object_Points, corners_Seq, image_size, intrinsic_matrix, distortion_coeffs, rotation_vectors, translation_vectors);

    std::cout <<"save calibrate result....\n"<<endl;

    FileStorage fs(INTRINSICS_FILE, FileStorage::WRITE);
    if(fs.isOpened())
    {
        fs <<"M"<<intrinsic_matrix<<"D"<<distortion_coeffs;
        fs.release();

    }
    else
    {
        std::cout<<"error: share_file/calibration_result.txt can not open\n"<<endl;
        return -1;
    }




/*
**3.测试标定误差
**
*/
    std::cout <<"The result evaluation\n"<<endl;
    //所有图像的平均误差的总和
    double total_err = 0.0;
    //每幅图像的平均误差
    double err = 0.0;
    //保存重新计算得到的投影点
    vector<Point2f> image_points2;

    for (int i=0; i<image_count; i++)
    {
        err = 0.0;
        
        vector<Point3f> tempPointSet = object_Points[i];
        //重新投影计算
        projectPoints(tempPointSet, rotation_vectors[i], translation_vectors[i], intrinsic_matrix, distortion_coeffs, image_points2);

        //计算新的投影点和旧的投影点之间的误差
        vector<Point2f> tempImagePoint = corners_Seq[i];
        //申请空间
        Mat tempImagePointMat = Mat(1, tempImagePoint.size(), CV_32FC2);
        Mat image_points2Mat = Mat(1, image_points2.size(), CV_32FC2);

        for (int j=0; j<tempImagePoint.size(); j++)
        {
            //保存数据
            image_points2Mat.at<Vec2f>(0,j) = Vec2f(image_points2[j].x, image_points2[j].y);
            tempImagePointMat.at<Vec2f>(0,j) = Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
        }

        err = norm(image_points2Mat, tempImagePointMat, NORM_L2);
        total_err += err/= point_counts[i];

        std::cout <<"The photo "<<i+1<<" err is "<<err<<"\n"<<endl;
        
    }

    std::cout <<"total_err is "<<total_err<<"\n"<<endl;
    std::cout <<"avg_err is "<<total_err /image_count <<"\n"<<endl;



/*
**4.测试标定效果
**
*/
    Mat mapx = Mat(image_size, CV_32FC1);
    Mat mapy = Mat(image_size, CV_32FC1);

    Mat R = Mat::eye(3,3,CV_32F);

    string imageFileName;
    stringstream StrStm;

    std::cout<<"Calibration photo.....\n"<<endl;

    for(int i=0; i < image_count; i++)
    {
        Mat newCameraMatrix = Mat(3,3,CV_32FC1, Scalar::all(0));
        initUndistortRectifyMap(intrinsic_matrix, distortion_coeffs, R, intrinsic_matrix, image_size, CV_32FC1, mapx, mapy);
        StrStm.clear();
        imageFileName.clear();
        temp_filename.clear();

        imageFileName += RESOURCE_FLIE_DIR;
        
        StrStm<<i+1;
        StrStm >> temp_filename;
        imageFileName += temp_filename;
        imageFileName += ".JPG";

        std::cout<<"Calibration the "<<imageFileName<<" photo...\n"<<endl;

        Mat t = imread(imageFileName);

        Mat newimage = t.clone();
        remap(t, newimage, mapx, mapy, INTER_LINEAR);

        StrStm.clear();
        imageFileName.clear();
        temp_filename.clear();
        imageFileName += RESULT_FLIE_DIR;
        StrStm<<i+1;
        StrStm >> temp_filename;
        imageFileName += temp_filename;
        imageFileName += "_d.JPG";

        imwrite(imageFileName,newimage);
        
    }
}
