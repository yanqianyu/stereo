//
//  main.cpp
//  calibration_1
//
//  Created by 严倩羽 on 2018/5/23.
//  Copyright © 2018 apple. All rights reserved.
//

//zhang zhengyou
#include <iostream>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <vector>
using namespace std;
using namespace cv;
int  main(){
    //标定所用图像文件的路径
    ifstream fin("/Users/apple/Desktop/stereo/calibdataLeft.txt");
    //保存标定结果的文件
    ofstream fout("/Users/apple/Desktop/stereo/calibration_1_result/calibration_result.txt");
    //读取图片 从中提取角点
    int imageCount=0;//图片数量
    Size imageSize;//图片尺寸
    Size boardSize=Size(6,9);//标定板上每行、列角点数
    vector<Mat> InImage;
    
    vector<Point2f> imagePointsBuf;
    vector<vector<Point2f>> imagePointsSeq;//角点二维坐标
    string filename;
    while(getline(fin,filename)){
        imageCount++;
        // 用于观察检验输出
        cout<<"image_count = "<<imageCount<<endl;
        //输出检验
        Mat imageInput=imread(filename);
        InImage.push_back(imageInput);
        if (imageCount == 1)  //读入第一张图片时获取图像宽高信息
        {
            imageSize.width = imageInput.cols;
            imageSize.height =imageInput.rows;
            cout<<"image_size.width = "<<imageSize.width<<endl;
            cout<<"image_size.height = "<<imageSize.height<<endl;
        }
        
        //提取角点
        if (0 == findChessboardCorners(imageInput,boardSize,imagePointsBuf))
        {
            cout<<"can not find chessboard corners!\n"; //找不到角点
            exit(1);
        }
        else
        {
            Mat view_gray;
            cvtColor(imageInput,view_gray,CV_RGB2GRAY);
            find4QuadCornerSubpix(view_gray,imagePointsBuf,Size(5,5)); //对粗提取的角点进行精确化
            imagePointsSeq.push_back(imagePointsBuf);
            //在图像上显示角点位置
            drawChessboardCorners(view_gray,boardSize,imagePointsBuf,true); //用于在图片中标记角点
            imshow("Camera Calibration",view_gray);//显示图片
            waitKey(1500);//暂停1.5S
        }
    }
    cvDestroyWindow("Camera Calibration");
    //标定前的初始化
    cout<<"开始标定"<<endl;
    Size square_size = Size(10,10);
    vector<vector<Point3f>> object_points;//保存标定板上角点的三维坐标
    //内外参数
    Mat cameraMatrix=Mat(3,3,CV_32FC1,Scalar::all(0)); // 摄像机内参数矩阵
    vector<int> point_counts;  // 每幅图像中角点的数量
    Mat distCoeffs=Mat(1,5,CV_32FC1,Scalar::all(0)); //摄像机的5个畸变系数：k1,k2,p1,p2,k3
    vector<Mat> tvecsMat;  // 每幅图像的旋转向量
    vector<Mat> rvecsMat; // 每幅图像的平移向量
    //初始化标定板上角点的三维坐标
    int i,j,t;
    for (t=0;t<imageCount;t++)
    {
        vector<Point3f> tempPointSet;
        for (i=0;i<boardSize.height;i++)
        {
            for (j=0;j<boardSize.width;j++)
            {
                Point3f realPoint;
                //假设标定板放在世界坐标系中z=0的平面上
                realPoint.x = i*square_size.width;
                realPoint.y = j*square_size.height;
                realPoint.z = 0;
                tempPointSet.push_back(realPoint);
            }
        }
        object_points.push_back(tempPointSet);
    }
    //初始化每幅图像中的角点数量，假定每幅图像中都可以看到完整的标定板
    for (i=0;i<imageCount;i++)
    {
        point_counts.push_back(boardSize.width*boardSize.height);
    }
    
    //开始标定
    calibrateCamera(object_points,imagePointsSeq,imageSize,cameraMatrix,distCoeffs,rvecsMat,tvecsMat,0);
    cout<<"标定完成！"<<endl;
    
    //保存标定的结果
    cout<<"开始保存标定的结果"<<endl;
    //每幅图像的旋转矩阵
    Mat rotationMatrix=Mat(3,3,CV_32FC1,Scalar::all(0));
    fout<<"相机内参数矩阵："<<endl;
    fout<<cameraMatrix<<endl<<endl;
    fout<<"畸变系数"<<endl;
    fout<<distCoeffs<<endl;
    for(int i=0;i<imageCount;i++){
        fout<<"第"<<i+1<<"幅图像的旋转向量："<<endl;
        fout<<tvecsMat[i]<<endl;
        //将旋转向量转换为相对应的旋转矩阵
        Rodrigues(tvecsMat[i],rotationMatrix);
        fout<<"第"<<i+1<<"幅图像的旋转矩阵："<<endl;
        fout<<rotationMatrix<<endl;
        fout<<"第"<<i+1<<"幅图像的平移向量："<<endl;
        fout<<rvecsMat[i]<<endl<<endl;
    }
    cout<<"完成保存"<<endl;
    fout<<endl;
    
    
    //矫正图像
    cout<<"开始矫正"<<endl;
    //方法一
    /*
    for(int i=0;i<imageCount;i++){
        Mat OutImage=InImage[i].clone();
        undistort(InImage[i], OutImage, cameraMatrix, distCoeffs);
        imshow("view1",InImage[i]);
        imshow("result1",OutImage);
        waitKey(1500);//暂停1.5S
    }
     */
    //方法二
    Mat mapx = Mat(imageSize,CV_32FC1);
    Mat mapy = Mat(imageSize,CV_32FC1);
    Mat R = Mat::eye(3,3,CV_32F);
    for(int i=0;i<imageCount;i++){
        initUndistortRectifyMap(cameraMatrix,distCoeffs,R,cameraMatrix,imageSize,CV_32FC1,mapx,mapy);
        Mat OutImage=InImage[i].clone();
        remap(InImage[i],OutImage,mapx, mapy, INTER_LINEAR);
        imshow("view2",InImage[i]);
        imshow("result2",OutImage);
        waitKey(1500);//暂停1.5S
    }
    cout<<"矫正完成"<<endl;
}

