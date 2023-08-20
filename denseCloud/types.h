#ifndef _TYPES_H_
#define _TYPES_H_
#include <iostream>
#include <set>
#include <vector>
#include <fstream>
#include <string> 
#include <sstream> 
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "opencv2/opencv.hpp"
#include "glog/logging.h"
// constants
#define TWO_PI			6.283185307179586476925286766559
#define PI				3.1415926535897932384626433832795
#define HALF_PI			1.5707963267948966192313216916398
#define SQRT_2PI		2.506628274631000502415765284811
#define INV_TWO_PI		0.15915494309189533576888376337251
#define INV_PI			0.31830988618379067153776752674503
#define INV_HALF_PI		0.63661977236758134307553505349006
#define INV_SQRT_2PI	0.39894228040143267793994605993439
#define D2R(d)			((d)*(PI/180.0)) // degree to radian
#define R2D(r)			((r)*(180.0/PI)) // radian to degree
#define SQRT_2			1.4142135623730950488016887242097
#define SQRT_3			1.7320508075688772935274463415059
#define LOG_2			0.30102999566398119521373889472449
#define LN_2			0.69314718055994530941723212145818
#define ZERO_TOLERANCE	(1e-7)
#define INV_ZERO		(1e+14)

// float constants
#define FTWO_PI			((float)TWO_PI)
#define FPI				((float)PI)
#define FHALF_PI		((float)HALF_PI)
#define FSQRT_2PI		((float)SQRT_2PI)
#define FINV_TWO_PI		((float)INV_TWO_PI)
#define FINV_PI			((float)INV_PI)
#define FINV_HALF_PI	((float)INV_HALF_PI)
#define FINV_SQRT_2PI	((float)INV_SQRT_2PI)
#define FD2R(d)			((d)*(FPI/180.f)) // degree to radian
#define FR2D(r)			((r)*(180.f/FPI)) // radian to degree
#define FSQRT_2			((float)SQRT_2)
#define FSQRT_3			((float)SQRT_3)
#define FLOG_2			((float)LOG_2)
#define FLN_2			((float)LN_2)
#define FZERO_TOLERANCE	0.0001f
#define FINV_ZERO		1000000.f


using dType = double;


enum CameraModel
{
	NONE,
	SIMPLE_RADIAL,
};

struct Camera
{
	int CAMERA_ID;
	CameraModel model;
	int height;
	int width;
	float focalLength;
	float cx;
	float cy;
	Camera();
	Camera(const std::string& cameraStr);;
	cv::Point_<dType> pixel2cam(const cv::Point_<dType>& p);
	static std::vector<Camera> readCameras(const std::string& cameraTXT);
	static CameraModel stringToCameraModel(const std::string& str);
};

struct ImageData
{
	int imageId;
	Eigen::Quaternion<dType>  q;
	Eigen::Matrix<dType, 3, 1>  t;
	Eigen::Matrix<dType, 3, 1>  camera_t;
	cv::Mat worldToCamera;
	int cameraId;
	std::string imgPath;
	std::vector<cv::Point_<dType>>thisImgPts;
	std::vector<int>thisObjPtsIdx;
	dType worldPtDepth(const cv::Point3_<dType>& X)const;
	ImageData();
	ImageData(const std::string& posStr, const std::string& ptsStr);
	static std::map<int, ImageData> readImageData(const std::string& imagesTXT);
};

struct Point3dData
{
	int objPtId;
	cv::Point3_<dType> objPt;
	cv::Point3_<uchar> objPtRgb;
	dType error;
	std::map<int,int>tracks_imgId_imgPtId;
	Point3dData();
	Point3dData(const std::string& posStr);
	static std::map<int, Point3dData> readPoint3dData(const std::string& points3DTXT);
};

#endif // !_TYPES_H_