/*
 * CCarCamCal.h
 *
 *  Created on: Jan 22, 2017
 *      Author: yanzhangwu
 */
#ifndef _CCALIBRATION_H
#define _CCALIBRATION_H
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/opencv.hpp"
//#include <glog/logging.h>
#ifdef WIN32
#include <io.h>
#else
#include <unistd.h>
#endif

// using namespace std;

class CalibrationIntri {
 public:
  CalibrationIntri();
  ~CalibrationIntri();
  cv::Mat m_IntriMatrix;
  cv::Mat m_distCoeffs;
  double m_totalAvgErr;
  // board para
  cv::Size m_boardSize;
  float m_squareSize;
  float m_aspectRatio;

 private:
  std::vector<cv::Mat> m_rvecs;
  std::vector<cv::Mat> m_tvecs;
  std::vector<float> m_reprojErrs;

  // Rectification
  cv::Mat m_cameraMatrixForRTF;
  cv::Mat m_distCoeffsForRTF;

 public:
  void SetBoardPara(cv::Size _boardSize, float _squareSize, float _aspectRatio);
  bool FindBoardCorners(cv::Mat& image, std::vector<cv::Point2f>& pointBuff,
                        bool draw);
  bool RunCalibration(std::vector<std::vector<cv::Point2f> > imagePoints,
                      cv::Size imageSize, int flags);
  bool SaveCamIntrinsic(const char* _filename);

 private:
  double ComputeReprojectionErrors(
      const std::vector<std::vector<cv::Point3f> >& objectPoints,
      const std::vector<std::vector<cv::Point2f> >& imagePoints);
  void CalcChessboardCorners(std::vector<cv::Point3f>& corners);

  bool ImageRectification(char* undistortXmlName, cv::Mat _inImage,
                          cv::Mat& outImage_);
};

class CalibrationExtri {
 public:
  CalibrationExtri();
  ~CalibrationExtri();

  cv::Mat m_rotationMatrix;
  cv::Mat m_offsetMatrix;
  cv::Mat m_cameraLocation;

 private:
  cv::Mat m_IntriMatrix;
  cv::Mat m_distCoeffs;
  // 2017-09-22
  float m_fcx;  // focal length x
  float m_fcy;  // focal length y
  float m_cx;
  float m_cy;
  float m_alpha_c;

  // 3 line parameters
  // 2017-09-22
  cv::Point3f m_worldPointA;
  cv::Point3f m_worldPointB;

  cv::Point2f m_auxPointA;  // Point A (Auxiliary Coordinate)
  cv::Point2f m_auxPointB;  // Point B (Auxiliary Coordinate)
  cv::Point2f m_auxPointV;  // Point V (Auxiliary Coordinate)

  // 3 line parameters camera extrinsic
  float m_q1;
  float m_q2;
  float m_q3;

 public:
  //通过xml文件加载内参
  bool LoadCamIntrinsic(char* _filename);
  //内参没有标定的时候,直接设置参数
  void SetCamIntrinsic(float _fcx, float _fcy, float _cx, float _cy,
                       float _alpha_c);
  //设置参考点
  void SetLocationInWorld(cv::Point3f _worldPointA, cv::Point3f _worldPointB);
  //计算外参矩阵
  bool RunCalibration(cv::Point2f _pointA, cv::Point2f _pointB,
                      cv::Point2f _pointV);
  bool GetWorldPointInImg(cv::Point2f _imgPoint, cv::Point3f& wordPoint_);
  //保存外参数xml
  bool SaveCamExtrinsic(char* _filename, cv::Point2f _deltaXY);
};

class VisualDistance {
 public:
  VisualDistance();
  ~VisualDistance();
  cv::Point2f m_headLocation;

 private:
  cv::Mat m_IntriMatrix;
  cv::Mat m_distCoeffs;
  cv::Mat m_rotationMatrix;
  cv::Mat m_offsetMatrix;

 public:
  //加载内参和外参
  bool LoadCamPara(const char* _intrinsicXmlName,
                   const char* _extrinsicXmlName);
  //通过内参,外参计算距离
  bool GetWorldPointInImg(cv::Point2f imgPoint_, cv::Point3f& _wordPoint);
  //计算世界坐标在图像中的位置
  bool GetImgPointInWorld(cv::Point3f _wordPoint, cv::Point2f& imgPoint_);
};

#endif  //_CCALIBRATION_H
