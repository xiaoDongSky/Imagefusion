/*
 * CCarCamCal.cpp
 *
 *  Created on: Jan 22, 2017
 *      Author: yanzhangwu
 */
#include "CCarCamCal.h"

// calibration intrinsic
CalibrationIntri::CalibrationIntri() {}

CalibrationIntri::~CalibrationIntri() {}

void CalibrationIntri::SetBoardPara(cv::Size _boardSize, float _squareSize,
                                    float _aspectRatio) {
  m_boardSize = _boardSize;
  m_squareSize = _squareSize;
  m_aspectRatio = _aspectRatio;
}

bool CalibrationIntri::FindBoardCorners(cv::Mat &image,
                                        std::vector<cv::Point2f> &pointBuff,
                                        bool draw) {
  if (m_boardSize.width < 3 || m_boardSize.height < 3) {
    printf("boardSize error.\n");
    return false;
  }
  cv::Mat viewGray;
  cv::cvtColor(image, viewGray, cv::COLOR_BGR2GRAY);

  bool found = cv::findChessboardCorners(viewGray, m_boardSize, pointBuff,
                                         cv::CALIB_CB_ADAPTIVE_THRESH |
                                             cv::CALIB_CB_FAST_CHECK |
                                             cv::CALIB_CB_NORMALIZE_IMAGE);
  if (found) {
    cv::cornerSubPix(
        viewGray, pointBuff, cv::Size(11, 11), cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30,
                         0.1));
    if (draw)
      cv::drawChessboardCorners(image, m_boardSize, cv::Mat(pointBuff), found);
  } else {
    printf("Corners are not found.\n");
    return false;
  }
  return true;
}

bool CalibrationIntri::RunCalibration(
    std::vector<std::vector<cv::Point2f> > imagePoints, cv::Size imageSize,
    int flags) {
  m_IntriMatrix = cv::Mat::eye(3, 3, CV_64F);
  if (flags & cv::CALIB_FIX_ASPECT_RATIO)
    m_IntriMatrix.at<double>(0, 0) = m_aspectRatio;

  m_distCoeffs = cv::Mat::zeros(8, 1, CV_64F);

  std::vector<std::vector<cv::Point3f> > objectPoints(1);

  CalcChessboardCorners(objectPoints[0]);

  objectPoints.resize(imagePoints.size(), objectPoints[0]);

  double rms = cv::calibrateCamera(
      objectPoints, imagePoints, imageSize, m_IntriMatrix, m_distCoeffs,
      m_rvecs, m_tvecs, flags | cv::CALIB_FIX_K4 | cv::CALIB_FIX_K5);
  ///*|CALIB_FIX_K3*/|CALIB_FIX_K4|CALIB_FIX_K5);
  printf("RMS error reported by calibrateCamera: %g\n", rms);

  bool ok = checkRange(m_IntriMatrix) && checkRange(m_distCoeffs);

  m_totalAvgErr = ComputeReprojectionErrors(objectPoints, imagePoints);

  return ok;
}

bool CalibrationIntri::SaveCamIntrinsic(const char *_filename) {
  cv::FileStorage fs(_filename, cv::FileStorage::WRITE);
  if (!fs.isOpened()) {
    printf("save intrinsic matrix error.\n");
    return false;
  }
  fs << "camIntrinsicMat" << m_IntriMatrix;
  fs << "distortion_coefficients" << m_distCoeffs;

  cv::Mat cameraIntriMatrix = cv::Mat::eye(3, 4, CV_64F);

  m_IntriMatrix.copyTo(cameraIntriMatrix(cv::Range(0, m_IntriMatrix.rows),
                                         cv::Range(0, m_IntriMatrix.cols)));

  fs << "camIntriMatrix" << cameraIntriMatrix;

  fs.release();
  return true;
}

double CalibrationIntri::ComputeReprojectionErrors(
    const std::vector<std::vector<cv::Point3f> > &objectPoints,
    const std::vector<std::vector<cv::Point2f> > &imagePoints) {
  std::vector<cv::Point2f> imagePoints2;
  int i, totalPoints = 0;
  double totalErr = 0, err;
  m_reprojErrs.resize(objectPoints.size());

  for (i = 0; i < (int)objectPoints.size(); i++) {
    cv::projectPoints(cv::Mat(objectPoints[i]), m_rvecs[i], m_tvecs[i],
                      m_IntriMatrix, m_distCoeffs, imagePoints2);
    err = cv::norm(cv::Mat(imagePoints[i]), cv::Mat(imagePoints2), cv::NORM_L2);
    int n = (int)objectPoints[i].size();
    m_reprojErrs[i] = (float)std::sqrt(err * err / n);
    totalErr += err * err;
    totalPoints += n;
  }

  return std::sqrt(totalErr / totalPoints);
}

void CalibrationIntri::CalcChessboardCorners(
    std::vector<cv::Point3f> &corners) {
  corners.resize(0);
  for (int i = 0; i < m_boardSize.height; i++)
    for (int j = 0; j < m_boardSize.width; j++) {
      corners.push_back(
          cv::Point3f(float(j * m_squareSize), float(i * m_squareSize), 0));
    }
}

bool CalibrationIntri::ImageRectification(char *undistortXmlName,
                                          cv::Mat _inImage,
                                          cv::Mat &outImage_) {
  if (access(undistortXmlName, F_OK) != 0) {
    printf("%s is not existed.\n", undistortXmlName);
    return false;
  }
  cv::FileStorage fs(undistortXmlName, cv::FileStorage::READ);

  if (!fs.isOpened()) {
    printf("open %s error.\n", undistortXmlName);
    return false;
  }
  fs["camIntrinsicMat"] >> m_cameraMatrixForRTF;
  fs["distortion_coefficients"] >> m_distCoeffsForRTF;
  fs.release();

  if (m_cameraMatrixForRTF.data && m_distCoeffsForRTF.data) {
    cv::undistort(_inImage.clone(), outImage_, m_cameraMatrixForRTF,
                  m_distCoeffsForRTF);

    return true;
  } else {
    return false;
  }
}

// calibration extrinsic
CalibrationExtri::CalibrationExtri() {}

CalibrationExtri::~CalibrationExtri() {}

bool CalibrationExtri::LoadCamIntrinsic(char *_filename) {
  if (access(_filename, F_OK) != 0) {
    printf("%s is not existed.\n", _filename);
    return false;
  }
  cv::FileStorage fs(_filename, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    printf("open %s error.\n", _filename);
    return false;
  }
  fs["camIntrinsicMat"] >> m_IntriMatrix;
  fs["distortion_coefficients"] >> m_distCoeffs;
  fs.release();

  SetCamIntrinsic(
      m_IntriMatrix.at<double>(0, 0), m_IntriMatrix.at<double>(1, 1),
      m_IntriMatrix.at<double>(0, 2), m_IntriMatrix.at<double>(1, 2),
      m_IntriMatrix.at<double>(0, 0));
  return true;
}

void CalibrationExtri::SetCamIntrinsic(float _fcx, float _fcy, float _cx,
                                       float _cy, float _alpha_c) {
  m_fcx = _fcx;
  m_fcy = _fcy;
  m_cx = _cx;
  m_cy = _cy;
  m_alpha_c = _alpha_c;
}

void CalibrationExtri::SetLocationInWorld(cv::Point3f _worldPointA,
                                          cv::Point3f _worldPointB) {
  m_worldPointA = _worldPointA;
  m_worldPointB = _worldPointB;
}

bool CalibrationExtri::RunCalibration(cv::Point2f _pointA, cv::Point2f _pointB,
                                      cv::Point2f _pointV) {
#ifdef _UNDISTORT_
  std::cout << _pointA << "," << _pointB << "," << _pointV << endl;
  if (m_distCoeffs.data) {
    double k1 = m_distCoeffs.at<double>(0, 0);
    double k2 = m_distCoeffs.at<double>(1, 0);
    double p1 = m_distCoeffs.at<double>(2, 0);
    double p2 = m_distCoeffs.at<double>(3, 0);
    double k3 = m_distCoeffs.at<double>(4, 0);  // fish camera
    float pointAR = _pointA.x * _pointA.x + _pointA.y * _pointB.y;
    float pointBR = _pointB.x * _pointB.x + _pointB.y * _pointB.y;
    float pointVR = _pointV.x * _pointV.x + _pointV.y * _pointV.y;

    _pointA.x = _pointA.x * (1 + k1 * pointAR + k2 * pointAR * pointAR) +
                2 * p1 * _pointA.x * _pointA.y +
                p2 * (pointAR + 2 * _pointA.x * _pointA.x);
    _pointA.y = _pointA.y * (1 + k1 * pointAR + k2 * pointAR * pointAR) +
                2 * p2 * _pointA.x * _pointA.y +
                p1 * (pointAR + 2 * _pointA.y * _pointA.y);

    _pointA.x = _pointA.x * m_fcx + m_cx;
    _pointA.y = _pointA.y * m_fcy + m_cy;

    _pointB.x = _pointB.x * (1 + k1 * pointBR + k2 * pointBR * pointBR) +
                2 * p1 * _pointB.x * _pointB.y +
                p2 * (pointBR + 2 * _pointB.x * _pointB.x);
    _pointB.y = _pointB.y * (1 + k1 * pointBR + k2 * pointBR * pointBR) +
                2 * p2 * _pointB.x * _pointB.y +
                p1 * (pointBR + 2 * _pointB.y * _pointB.y);

    _pointB.x = _pointB.x * m_fcx + m_cx;
    _pointB.y = _pointB.y * m_fcy + m_cy;

    _pointV.x = _pointV.x * (1 + k1 * pointVR + k2 * pointVR * pointVR) +
                2 * p1 * _pointV.x * _pointV.y +
                p2 * (pointVR + 2 * _pointV.x * _pointV.x);
    _pointV.y = _pointV.y * (1 + k1 * pointVR + k2 * pointVR * pointVR) +
                2 * p2 * _pointV.x * _pointV.y +
                p1 * (pointVR + 2 * _pointV.y * _pointV.y);

    _pointV.x = _pointV.x * m_fcx + m_cx;
    _pointV.y = _pointV.y * m_fcy + m_cy;

    std::cout << _pointA << "," << _pointB << "," << _pointV << endl;
  }

#endif
  //_pointA,_pointB,_pointV for 3 line calibrate
  if (m_fcx == 0 || m_fcy == 0) {
    printf("The camera intrinsic parameters are error.\n");
    return false;
  } else {
    m_auxPointA.x = (_pointA.x - m_cx) / m_fcx;
    m_auxPointA.y = (_pointA.y - m_cy) / m_fcy;

    m_auxPointB.x = (_pointB.x - m_cx) / m_fcx;
    m_auxPointB.y = (_pointB.y - m_cy) / m_fcy;

    m_auxPointV.x = (_pointV.x - m_cx) / m_fcx;
    m_auxPointV.y = (_pointV.y - m_cy) / m_fcy;
  }

  // calc q1, q2, q3
  float Pa = 0, Pb = 0, Pc = 0, Pd = 0, Pe = 0;

  Pa = m_auxPointA.y - m_auxPointB.y;
  Pb = m_auxPointB.x - m_auxPointA.x;
  Pc = m_auxPointA.x * m_auxPointB.y - m_auxPointB.x * m_auxPointA.y;
  Pd = Pa * m_auxPointV.x + Pb * m_auxPointV.y + Pc;
  Pe = 1 + m_auxPointV.x * m_auxPointV.x + m_auxPointV.y * m_auxPointV.y;

  m_q3 = atan2((Pd * m_auxPointV.x - Pa * Pe), (Pd * m_auxPointV.y - Pb * Pe));
  m_q2 = atan2(1, -(cos(m_q3) * m_auxPointV.y + sin(m_q3) * m_auxPointV.x));
  m_q1 = atan2(
      -(sin(m_q3) * m_auxPointV.y - cos(m_q3) * m_auxPointV.x) * sin(m_q2), -1);

  // calc Rotation matrix
  m_rotationMatrix = cv::Mat::zeros(3, 3, CV_32F);

  m_rotationMatrix.at<float>(0, 0) =
      (cos(m_q3) * cos(m_q1) - sin(m_q3) * cos(m_q2) * sin(m_q1));
  m_rotationMatrix.at<float>(1, 0) =
      -sin(m_q3) * cos(m_q1) - cos(m_q3) * cos(m_q2) * sin(m_q1);
  m_rotationMatrix.at<float>(2, 0) = (sin(m_q2) * sin(m_q1));
  m_rotationMatrix.at<float>(0, 1) =
      (cos(m_q3) * sin(m_q1) + sin(m_q3) * cos(m_q2) * cos(m_q1));
  m_rotationMatrix.at<float>(1, 1) =
      -sin(m_q3) * sin(m_q1) + cos(m_q3) * cos(m_q2) * cos(m_q1);
  m_rotationMatrix.at<float>(2, 1) = (-sin(m_q2) * cos(m_q1));
  m_rotationMatrix.at<float>(0, 2) = (sin(m_q3) * sin(m_q2));
  m_rotationMatrix.at<float>(1, 2) = cos(m_q3) * sin(m_q2);
  m_rotationMatrix.at<float>(2, 2) = (cos(m_q2));

  // calc offsets
  if (Pb == 0) {
    printf("The scene does not meet the requirements.\n");
    return false;
  } else {
    m_offsetMatrix = cv::Mat::zeros(3, 1, CV_32F);

    m_offsetMatrix.at<float>(0, 0) =
        ((m_auxPointA.x * m_worldPointB.x - m_auxPointB.x * m_worldPointA.x) *
             m_rotationMatrix.at<float>(0, 0) +
         (m_auxPointA.x - m_auxPointB.x) * m_worldPointA.y *
             m_rotationMatrix.at<float>(0, 1) +
         m_auxPointA.x * m_auxPointB.x * (m_worldPointA.x - m_worldPointB.x) *
             m_rotationMatrix.at<float>(2, 0)) /
        Pb;

    m_offsetMatrix.at<float>(2, 0) =
        ((m_worldPointB.x - m_worldPointA.x) *
             m_rotationMatrix.at<float>(0, 0) +
         (m_auxPointA.x * m_worldPointA.x - m_auxPointB.x * m_worldPointB.x) *
             m_rotationMatrix.at<float>(2, 0) +
         (m_auxPointA.x - m_auxPointB.x) * m_worldPointA.y *
             m_rotationMatrix.at<float>(2, 1)) /
        Pb;

    m_offsetMatrix.at<float>(1, 0) =
        m_auxPointA.y * (m_rotationMatrix.at<float>(2, 0) * m_worldPointA.x +
                         m_rotationMatrix.at<float>(2, 1) * m_worldPointA.y +
                         m_offsetMatrix.at<float>(2, 0)) -
        m_rotationMatrix.at<float>(1, 0) * m_worldPointA.x -
        m_rotationMatrix.at<float>(1, 1) * m_worldPointA.y;
  }

  m_cameraLocation = -m_rotationMatrix.inv() * m_offsetMatrix;
  if (!m_cameraLocation.data) {
    printf("can not get camera location.\n");
    return false;
  }
  return true;
}

bool CalibrationExtri::GetWorldPointInImg(cv::Point2f _imgPoint,
                                          cv::Point3f &wordPoint_) {
  // input point
  cv::Mat camPoint = cv::Mat::zeros(3, 1, CV_32F);
  camPoint.at<float>(0, 0) = (_imgPoint.x - m_IntriMatrix.at<double>(0, 2)) /
                             m_IntriMatrix.at<double>(0, 0);
  camPoint.at<float>(1, 0) = (_imgPoint.y - m_IntriMatrix.at<double>(1, 2)) /
                             m_IntriMatrix.at<double>(1, 1);

  camPoint.at<float>(2, 0) = 1;

  float scale = 0.0, scaleDeno = 0.0;
  cv::Mat invRotaMatrix = m_rotationMatrix.inv();
  cv::Mat scaleDenoMat = invRotaMatrix.row(2) * camPoint;

  if (!scaleDenoMat.data) return false;
  scaleDeno = scaleDenoMat.at<float>(0, 0);

  if (scaleDeno == 0) return false;
  cv::Mat scaleMat = (invRotaMatrix.row(2) * m_offsetMatrix) / scaleDeno;

  if (!scaleMat.data) return false;
  scale = scaleMat.at<float>(0, 0);

  cv::Mat worldPoint = invRotaMatrix * (scale * camPoint - m_offsetMatrix);
  if (!worldPoint.data) return false;
  wordPoint_ =
      cv::Point3f(worldPoint.at<float>(0, 0), worldPoint.at<float>(1, 0),
                  worldPoint.at<float>(2, 0));
  return true;
}

bool CalibrationExtri::SaveCamExtrinsic(char *_filename, cv::Point2f _deltaXY) {
  cv::FileStorage fs(_filename, cv::FileStorage::WRITE);

  if (!fs.isOpened()) {
    printf("open %s error.\n", _filename);
    return false;
  } else {
    fs << "rotationMatrix" << m_rotationMatrix;
    fs << "offsetMatrix" << m_offsetMatrix;
    cv::Point3f test;
    test.x = m_cameraLocation.at<float>(0, 0);
    test.y = m_cameraLocation.at<float>(1, 0);
    test.z = m_cameraLocation.at<float>(2, 0);
    fs << "cameraLocation" << test;

    cv::Mat camExtriMatrix = cv::Mat::eye(4, 4, CV_64F);

    m_rotationMatrix.copyTo(
        camExtriMatrix(cv::Range(0, m_rotationMatrix.rows),
                       cv::Range(0, m_rotationMatrix.cols)));
    m_offsetMatrix.copyTo(
        camExtriMatrix(cv::Range(0, m_offsetMatrix.rows), cv::Range(3, 3 + 1)));

    fs << "camExtriMatrix" << camExtriMatrix;
    fs << "deltaXY" << _deltaXY;
    fs << "headLocation" << _deltaXY;
    fs.release();
  }

  return true;
}

// visaul distance
VisualDistance::VisualDistance() {}

VisualDistance::~VisualDistance() {}

bool VisualDistance::LoadCamPara(const char *_intrinsicXmlName,
                                 const char *_extrinsicXmlName) {
  if (access(_intrinsicXmlName, F_OK) != 0 ||
      access(_extrinsicXmlName, F_OK) != 0) {
    // LOG(ERROR)<<"Load xml file error.";
    printf("Load %s or %s error.\n", _intrinsicXmlName, _extrinsicXmlName);
    return false;
  }

  cv::FileStorage fs(_intrinsicXmlName, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    // LOG(ERROR)<<"Open xml file error.";
    printf("Open %s error.\n", _intrinsicXmlName);
    return false;
  }
  fs["camIntrinsicMat"] >> m_IntriMatrix;
  fs["distortion_coefficients"] >> m_distCoeffs;
  fs.release();
  if (!m_IntriMatrix.data) {
    // LOG(ERROR)<<"Intrinsic Matrix error.";
    return false;
  }

  fs.open(_extrinsicXmlName, cv::FileStorage::READ);

  if (!fs.isOpened()) {
    // LOG(ERROR)<<"Open xml file error.";
    printf("Open %s error.\n", _extrinsicXmlName);
    return false;
  }

  fs["rotationMatrix"] >> m_rotationMatrix;
  fs["offsetMatrix"] >> m_offsetMatrix;
  fs["headLocation"] >> m_headLocation;
  fs.release();
  return true;
}

bool VisualDistance::GetWorldPointInImg(cv::Point2f _imgPoint,
                                        cv::Point3f &wordPoint_) {
  // input point
  cv::Mat camPoint = cv::Mat::zeros(3, 1, CV_32F);
  camPoint.at<float>(0, 0) = (_imgPoint.x - m_IntriMatrix.at<double>(0, 2)) /
                             m_IntriMatrix.at<double>(0, 0);
  camPoint.at<float>(1, 0) = (_imgPoint.y - m_IntriMatrix.at<double>(1, 2)) /
                             m_IntriMatrix.at<double>(1, 1);

  camPoint.at<float>(2, 0) = 1.0;

  float scale = 0.0, scaleDeno = 0.0;
  cv::Mat invRotaMatrix = m_rotationMatrix.inv();
  cv::Mat scaleDenoMat = invRotaMatrix.row(2) * camPoint;

  if (!scaleDenoMat.data) return false;
  scaleDeno = scaleDenoMat.at<float>(0, 0);

  if (scaleDeno == 0) return false;
  cv::Mat scaleMat = (invRotaMatrix.row(2) * m_offsetMatrix) / scaleDeno;

  if (!scaleMat.data) return false;
  scale = scaleMat.at<float>(0, 0);

  cv::Mat worldPoint = invRotaMatrix * (scale * camPoint - m_offsetMatrix);
  if (!worldPoint.data) return false;
  wordPoint_ =
      cv::Point3f(worldPoint.at<float>(0, 0), worldPoint.at<float>(1, 0),
                  worldPoint.at<float>(2, 0));
  return true;
}

bool VisualDistance::GetImgPointInWorld(cv::Point3f _wordPoint,
                                        cv::Point2f &imgPoint_) {
  cv::Mat wordPoint = cv::Mat::zeros(3, 1, CV_32F);
  wordPoint.at<float>(0, 0) = _wordPoint.x;
  wordPoint.at<float>(1, 0) = _wordPoint.y;
  wordPoint.at<float>(2, 0) = 0;

  cv::Mat camMatrix = m_rotationMatrix * wordPoint + m_offsetMatrix;

  if (!camMatrix.data) return false;

  camMatrix.col(0) = camMatrix.col(0) / camMatrix.at<float>(2, 0);
  //
  // std::cout<<camMatrix<<std::endl;
  camMatrix.convertTo(camMatrix, CV_64F);
  // std::cout<<camMatrix<<std::endl;

  cv::Mat imgPoint = m_IntriMatrix * camMatrix;

  if (!imgPoint.data) return false;
  imgPoint_ = cv::Point2f(imgPoint.at<double>(0, 0), imgPoint.at<double>(1, 0));
  return true;
}
