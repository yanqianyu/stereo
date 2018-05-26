//
//  main.cpp
//  stereoCalibration
//
//  Created by 严倩羽 on 2018/5/23.
//  Copyright © 2018 apple. All rights reserved.
//

//opencv版本3.3
#include <iostream>
#include <fstream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <vector>
using namespace std;
using namespace cv;
Mat cameraMatrixes[2],DistCoeffs[2];
vector<Mat> rvecs[2],tvecs[2];
vector<vector<Point2f>> imagePointsSeqs[2];
vector<vector<Point3f>> objectPoints;
vector<Mat> InImageL;
vector<Mat> InImageR;
Size imSize;
int imCount=0;
//标定左边的单目相机
void left(){
    //标定所用图像文件的路径
    ifstream fin("/Users/apple/Desktop/stereo/calibdataLeft.txt");
    //保存标定结果的文件
    ofstream fout("/Users/apple/Desktop/stereo/stereoCalibration_result/calibration_result_left.txt");
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
            imshow("Camera Calibration Left",view_gray);//显示图片
            waitKey(1500);//暂停1.5S
        }
    }
    InImageL=InImage;
    imCount=imageCount;
    cvDestroyWindow("Camera Calibration Left");
    //标定前的初始化
    cout<<"开始标定左边单目相机"<<endl;
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
    objectPoints=object_points;
    //初始化每幅图像中的角点数量，假定每幅图像中都可以看到完整的标定板
    for (i=0;i<imageCount;i++)
    {
        point_counts.push_back(boardSize.width*boardSize.height);
    }
    
    //开始标定
    calibrateCamera(object_points,imagePointsSeq,imageSize,cameraMatrix,distCoeffs,rvecsMat,tvecsMat,0);
    cout<<"左边单目相机标定完成！"<<endl;
    
    cameraMatrixes[0]=cameraMatrix;
    DistCoeffs[0]=distCoeffs;
    rvecs[0]=rvecsMat;
    tvecs[0]=tvecsMat;
    imagePointsSeqs[0]=imagePointsSeq;
    imSize=imageSize;
    //保存标定的结果
    cout<<"开始保存左边单目相机标定的结果"<<endl;
    //每幅图像的旋转矩阵
    Mat rotationMatrix=Mat(3,3,CV_32FC1,Scalar::all(0));
    fout<<"相机内参数矩阵："<<endl;
    fout<<cameraMatrix<<endl<<endl;
    fout<<"畸变系数"<<endl;
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
    cout<<"完成左边单目相机标定信息保存"<<endl;
    fout<<endl;
    
    /*
    //矫正图像
    cout<<"开始矫正"<<endl;
    //方法二
    Mat mapx = Mat(imageSize,CV_32FC1);
    Mat mapy = Mat(imageSize,CV_32FC1);
    Mat R = Mat::eye(3,3,CV_32F);
    for(int i=0;i<imageCount;i++){
        initUndistortRectifyMap(cameraMatrix,distCoeffs,R,cameraMatrix,imageSize,CV_32FC1,mapx,mapy);
        Mat OutImage=InImage[i].clone();
        remap(InImage[i],OutImage,mapx, mapy, INTER_LINEAR);
        imshow("view_left",InImage[i]);
        imshow("result_left",OutImage);
        waitKey(1500);//暂停1.5S
    }
    cvDestroyWindow("view_left");
    cvDestroyWindow("result_left");
    cout<<"右边单目相机矫正完成"<<endl;
     */
}
//标定右边的单目相机
void right(){
    //标定所用图像文件的路径
    ifstream fin("/Users/apple/Desktop/stereo/calibdataRight.txt");
    //保存标定结果的文件
    ofstream fout("/Users/apple/Desktop/stereo/stereoCalibration_result/calibration_result_right.txt");
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
            imshow("Camera Calibration Right",view_gray);//显示图片
            waitKey(1500);//暂停1.5S
        }
    }
    InImageR=InImage;
    cvDestroyWindow("Camera Calibration Right");
    //标定前的初始化
    cout<<"开始标定右边单目相机"<<endl;
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
    
    cameraMatrixes[1]=cameraMatrix;
    DistCoeffs[1]=distCoeffs;
    rvecs[1]=rvecsMat;
    tvecs[1]=tvecsMat;
    imagePointsSeqs[1]=imagePointsSeq;
    
    cout<<"右边单目相机标定完成！"<<endl;
    
    //保存标定的结果
    cout<<"开始保存右边单目相机标定的结果"<<endl;
    //每幅图像的旋转矩阵
    Mat rotationMatrix=Mat(3,3,CV_32FC1,Scalar::all(0));
    fout<<"相机内参数矩阵："<<endl;
    fout<<cameraMatrix<<endl<<endl;
    fout<<"畸变系数"<<endl;
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
    cout<<"完成右边单目相机标定信息保存"<<endl;
    fout<<endl;
    
    /*
    //矫正图像
    cout<<"开始矫正"<<endl;
    Mat mapx = Mat(imageSize,CV_32FC1);
    Mat mapy = Mat(imageSize,CV_32FC1);
    Mat R = Mat::eye(3,3,CV_32F);
    for(int i=0;i<imageCount;i++){
        initUndistortRectifyMap(cameraMatrix,distCoeffs,R,cameraMatrix,imageSize,CV_32FC1,mapx,mapy);
        Mat OutImage=InImage[i].clone();
        remap(InImage[i],OutImage,mapx, mapy, INTER_LINEAR);
        imshow("view_right",InImage[i]);
        imshow("result_right",OutImage);
        waitKey(1500);//暂停1.5S
    }
    cvDestroyWindow("view_right");
    cvDestroyWindow("result_right");
    cout<<"右边单目相机矫正完成"<<endl;
     */
}
int  main(){
    Mat R,T,E,F;
    //E essential matrix;
    //F fundamental matrix;
    cameraMatrixes[0]=Mat(3,3,CV_32FC1,Scalar::all(0));
    cameraMatrixes[1]=Mat(3,3,CV_32FC1,Scalar::all(0));
    left();
    right();
    //双目立体标定
    //不改变之前标定所得到的内参
    double rms=stereoCalibrate(objectPoints, imagePointsSeqs[0], imagePointsSeqs[1], cameraMatrixes[0], DistCoeffs[0], cameraMatrixes[1], DistCoeffs[1], imSize, R, T, E, F);
    cout<<"done with rms error "<<rms<<endl;
    
    Mat R1,R2,P1,P2,Q;
    Rect validRoi[2];
    stereoRectify(cameraMatrixes[0], DistCoeffs[0], cameraMatrixes[1], DistCoeffs[1], imSize, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, imSize, &validRoi[0], &validRoi[1]);
    //记录两相机的旋转矩阵R和平移矩阵T
    cout<<"开始保存两相机的校正矩阵"<<endl;
    ofstream fout("/Users/apple/Desktop/stereo/stereoCalibration_result/calibration_result_stereo.txt");
    fout<<"R :"<<endl;
    fout<<R<<endl;
    
    fout<<"T :"<<endl;
    fout<<T<<endl;
    
    fout<<"R1 :"<<endl;
    fout<<R1<<endl;
    
    fout<<"R2 :"<<endl;
    fout<<R2<<endl;
    
    fout<<"P1 :"<<endl;
    fout<<P1<<endl;
    
    fout<<"P2 :"<<endl;
    fout<<P2<<endl;
    cout<<"保存两相机的校正矩阵结束"<<endl;
    
    Mat canvas;
    //由R P计算图像的映射表mapx，mapy
    Mat mapLx ;//= Mat(imSize,CV_32FC1);
    Mat mapLy ;//= Mat(imSize,CV_32FC1);
    initUndistortRectifyMap(cameraMatrixes[0], DistCoeffs[0], R1, P1, imSize, CV_32FC1, mapLx, mapLy);
    
    Mat mapRx ;
    Mat mapRy ;
    initUndistortRectifyMap(cameraMatrixes[1], DistCoeffs[1], R2, P2, imSize, CV_32FC1, mapRx, mapRy);
    
    Mat rectifyImageL,rectifyImageR;
    for(int i=0;i<imCount;i++){
        remap(InImageL[i],rectifyImageL,mapLx, mapLy, INTER_LINEAR);
        remap(InImageR[i],rectifyImageR,mapRx, mapRy, INTER_LINEAR);
        /*
        imshow("rectifyL",rectifyImageL);
        waitKey(1500);
        imshow("rectifyR", rectifyImageR);
        waitKey(1500);
        */
        double sf;
        int w,h;
        sf=600./MAX(imSize.width, imSize.height);
        w=cvRound(imSize.width*sf);
        h=cvRound(imSize.height*sf);
        canvas.create(h, w*2, CV_8UC3);
        
        //left
        Mat canvasPart=canvas(Rect(0,0,w,h));
        
        resize(rectifyImageL, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);
        Rect vroiL(cvRound(validRoi[0].x*sf),cvRound(validRoi[0].y*sf),cvRound(validRoi[0].width*sf),cvRound(validRoi[0].height*sf));
        rectangle(canvasPart, vroiL, Scalar(0, 0, 255), 3, 8);
        
        //right
        canvasPart=canvas(Rect(w,0,w,h));
        resize(rectifyImageR, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);
        Rect vroiR(cvRound(validRoi[1].x*sf),cvRound(validRoi[1].y*sf),cvRound(validRoi[1].width*sf),cvRound(validRoi[1].height*sf));
        rectangle(canvasPart, vroiR, Scalar(0, 0, 255), 3, 8);
        
        //画上对应线条
        for(int i=0;i<canvas.rows;i+=16)
            line(canvas, Point(0, i), Point(canvas.cols, i), Scalar(0, 255, 0), 1, 8);
        imshow("rectified", canvas);
        waitKey(1500);
        
        //求baseline
        
        //视差图
        //opencv版本3.3
        //stereo_match.cpp
        int numberOfDisparities=((imSize.width/8)+15)&-16;
        cv::Ptr<cv::StereoSGBM> sgbm=cv::StereoSGBM::create(0, 16, 3);
        sgbm->setPreFilterCap(63);
        int SADWindowSize = 9;
        int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
        sgbm->setBlockSize(sgbmWinSize);
        int cn = rectifyImageL.channels();
        sgbm->setP1(8 * cn*sgbmWinSize*sgbmWinSize);
        sgbm->setP2(32 * cn*sgbmWinSize*sgbmWinSize);
        sgbm->setMinDisparity(0);
        sgbm->setNumDisparities(numberOfDisparities);
        sgbm->setUniquenessRatio(10);
        sgbm->setSpeckleWindowSize(100);
        sgbm->setSpeckleRange(32);
        sgbm->setDisp12MaxDiff(1);
        sgbm->setMode(cv::StereoSGBM::MODE_SGBM);
        
        // Compute disparity
        Mat disp;
        sgbm->compute(rectifyImageL, rectifyImageR, disp);
        disp.convertTo(disp, CV_8U, 255/(numberOfDisparities*16.));
        imshow("disp",disp);
        waitKey(1500);
    }
}
