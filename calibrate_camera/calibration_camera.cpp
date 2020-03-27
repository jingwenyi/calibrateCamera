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
    //У׼ʹ�õ�ͼƬ��
    int image_count = 30;
    Mat frame, ResizeImage;
    Size image_size;

    //У׼ʹ�õ�����
    Size board_size = Size(8,5);

    //����ͼƬ̫����Ҫ���ŵı���
    //�ڿ��������̫�ߣ�У׼̫���ᵼ��
    //�ǵ��޷�ʶ��( ԭ���ǽǵ�ռ������̫�� )�� ������Ҫ��С
    double fscale = 0.2;
    //���̵Ĵ�С ��λ:mm
    double fSize_w = 150;
    double fSize_h = 150;
    Size outSize;
    bool use_SubPix = false;


    vector<Point2f> corners;
    vector<vector<Point2f> > corners_Seq;


/*
**  1.��ȡÿ��У׼ͼƬ�Ľǵ�����
**
*/

    std::cout <<"found corners...\n"<<endl;
    //��ȡÿ��ͼƬ
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

        //����ͼƬ
        outSize.width = frame.cols * fscale;
        outSize.height = frame.rows * fscale;
        resize(frame, ResizeImage, outSize, 0, 0, INTER_AREA);

        
        image_size = ResizeImage.size();

        Mat imageGray;
        cvtColor(ResizeImage, imageGray, CV_RGB2GRAY);
        bool patternfound = findChessboardCorners(ResizeImage, board_size, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
        //�ҵ�ͼƬ�Ľǵ㣬�����һ��û���ҵ��޷����������Ե�С����ֵ
        if(patternfound == false){
            std::cout<<"the: "<<n<<" photo corners is error\r\n"<<endl;
            return -1;
        }else{
            std::cout<<"the: "<<n<<" photo corners is ok\r\n"<<endl;
        }

        

        
        //�����ؾ�ȷ��,Ч������
        if(use_SubPix)
        {
            cornerSubPix(imageGray, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
            count += corners.size();
            corners_Seq.push_back(corners);
        }

        //�ѽǵ�Ŵ�������ͼƬ��ȥ
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
**2.�û�ȡ���Ľǵ㣬�궨���������궨�ļ�
**
*/

    std::cout<<"Angular point extraction is complete, start calibration\n"<<endl;


    //�ָ�size �Ĵ�С
    image_size = frame.size();
    //ÿ�����̸��ʵ�ʴ�С
    Size square_size = Size(fSize_w, fSize_h);
    //���涨����Ͻǵ����ά����
    vector<vector<Point3f> >     object_Points;
    //ÿ��ͼ���еĽǵ�����
    vector<int> point_counts;
    //��������ڲξ���
    Mat intrinsic_matrix = Mat(3, 3, CV_32FC1, Scalar::all(0));
    // �������5������ϵ����k1,k2,p1,p2,k3 
    Mat distortion_coeffs = Mat(1, 5, CV_32FC1, Scalar::all(0));
    //ÿ��ͼ�����ת����
    vector<Mat> rotation_vectors;
    //ÿ��ͼ���ƽ������
    vector<Mat> translation_vectors;

    //�ѽǵ�ŵ�z=0��3λ������ȥ
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


    //ÿ��ͼ���ܿ������еĽǵ�
    for(int i=0; i<image_count; i++)
    {
        point_counts.push_back(board_size.width * board_size.height);
    }


    //ִ�б궨����
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
**3.���Ա궨���
**
*/
    std::cout <<"The result evaluation\n"<<endl;
    //����ͼ���ƽ�������ܺ�
    double total_err = 0.0;
    //ÿ��ͼ���ƽ�����
    double err = 0.0;
    //�������¼���õ���ͶӰ��
    vector<Point2f> image_points2;

    for (int i=0; i<image_count; i++)
    {
        err = 0.0;
        
        vector<Point3f> tempPointSet = object_Points[i];
        //����ͶӰ����
        projectPoints(tempPointSet, rotation_vectors[i], translation_vectors[i], intrinsic_matrix, distortion_coeffs, image_points2);

        //�����µ�ͶӰ��;ɵ�ͶӰ��֮������
        vector<Point2f> tempImagePoint = corners_Seq[i];
        //����ռ�
        Mat tempImagePointMat = Mat(1, tempImagePoint.size(), CV_32FC2);
        Mat image_points2Mat = Mat(1, image_points2.size(), CV_32FC2);

        for (int j=0; j<tempImagePoint.size(); j++)
        {
            //��������
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
**4.���Ա궨Ч��
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
