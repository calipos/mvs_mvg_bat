#include <iostream>
#include <set>
#include <vector>
#include <fstream>
#include <string> 
#include <sstream> 
#include "glog/logging.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "opencv2/opencv.hpp"
std::vector<std::string> splitString(const std::string& src, const std::string& symbols, bool repeat)
{
	std::vector<std::string> result;
	int startIdx = 0;
	for (int i = 0; i < src.length(); i++)
	{
		bool isMatch = false;
		for (int j = 0; j < symbols.length(); j++)
		{
			if (src[i] == symbols[j])
			{
				isMatch = true;
				break;
			}
			if (!repeat)
			{
				break;
			}
		}
		if (isMatch)
		{
			std::string sub = src.substr(startIdx, i - startIdx);
			startIdx = i + 1;
			if (sub.length() > 0)
			{
				result.push_back(sub);
			}
		}
		if (i + 1 == src.length())
		{
			std::string sub = src.substr(startIdx, src.length() - startIdx);
			startIdx = i + 1;
			if (sub.length() > 0)
			{
				result.push_back(sub);
			}
		}
	}
	return std::move(result);
}
template<class Dtype>
Dtype str2number(const std::string& str)
{
	std::stringstream ss(str);
	Dtype ret;
	ss >> ret;
	return   ret;
}
enum CameraModel
{
	NONE,
	SIMPLE_RADIAL,
};

using dType = double;
struct Camera
{
	int CAMERA_ID;
	CameraModel model;
	int height;
	int width;
	float focalLength;
	float cx;
	float cy;
	Camera() {}
	Camera(const std::string& cameraStr)
	{
		std::vector<std::string> params = splitString(cameraStr," ",true);
		CHECK(params.size()>5);
		CAMERA_ID = str2number<int>(params[0]);
		model = stringToCameraModel(params[1]);
		width = str2number<int>(params[2]);
		height = str2number<int>(params[3]);
		focalLength = str2number<float>(params[4]);
		cx = str2number<float>(params[5]);
		cy = str2number<float>(params[6]);
	}
	static std::vector<Camera> readCameras(const std::string& cameraTXT)
	{
		std::vector<Camera> ret;
		std::fstream fin(cameraTXT, std::ios::in);
		std::string aline;
		while (std::getline(fin, aline))
		{
			if (aline.length() <1)continue;
			if ( aline[0] == '#')continue;
			else
			{
				ret.emplace_back(aline);
			}
		}
		std::set<int>cameraIds;
		for (auto&d: ret)
		{
			cameraIds.insert(d.CAMERA_ID);
		}
		CHECK(cameraIds.size()==ret.size());
		return ret;
	}
	static CameraModel stringToCameraModel(const std::string&str)
	{
		if (str.compare("SIMPLE_RADIAL") == 0)
		{
			return CameraModel::SIMPLE_RADIAL;
		}
		else
		{
			CHECK(false);
			return CameraModel::NONE;
		}
	}
};
struct ImageData
{
	int imageId;
	Eigen::Quaternion<dType>  q;
	Eigen::Matrix<dType, 3, 1>  t;
	int cameraId;
	std::string imgPath;
	std::vector<cv::Point_<dType>>thisImgPts;
	std::vector<int>thisObjPtsIdx;
	ImageData() {}
	ImageData(const std::string& posStr, const std::string& ptsStr)
	{
		std::vector<std::string> params = splitString(posStr, " ", true);
		CHECK(params.size() ==10);
		imageId = str2number<int>(params[0]);
		q.w() = str2number<dType>(params[1]);
		q.x() = str2number<dType>(params[2]);
		q.y() = str2number<dType>(params[3]);
		q.z() = str2number<dType>(params[4]);
		t[0] = str2number<dType>(params[5]);
		t[1] = str2number<dType>(params[6]);
		t[2] = str2number<dType>(params[7]);
		cameraId = str2number<int>(params[8]);
		imgPath = params[9];

		std::vector<std::string> ptsData = splitString(ptsStr, " ", true);
		CHECK(ptsData.size() % 3 == 0);
		int ptsCnt = ptsData.size() /3;
		thisImgPts.resize(ptsCnt);
		thisObjPtsIdx.resize(ptsCnt);
		for (size_t i = 0; i < ptsCnt; i++)
		{
			thisImgPts[i].x = str2number<dType>(ptsData[i * 3 + 0]);
			thisImgPts[i].y = str2number<dType>(ptsData[i * 3 + 1]);
			thisObjPtsIdx[i] = str2number<int>(ptsData[i * 3 + 2]);
		}
		int minObjPtIdx = *std::min_element(thisObjPtsIdx.begin(), thisObjPtsIdx.end());
		int maxObjPtIdx = *std::max_element(thisObjPtsIdx.begin(), thisObjPtsIdx.end());
		LOG(INFO) << "minObjPtIdx = " << minObjPtIdx << "; maxObjPtIdx = " << maxObjPtIdx; 
	}
	static std::vector<ImageData> readImageData(const std::string& imagesTXT)
	{
		std::vector<ImageData> ret;
		std::fstream fin(imagesTXT, std::ios::in);
		std::string aline;
		std::string aline2;
		while (std::getline(fin, aline))
		{
			if (aline.length() < 1)continue;
			if (aline[0] == '#')continue;
			else
			{
				std::getline(fin, aline2);
				ret.emplace_back(aline, aline2);
			}
		} 
		return ret;
	}
};
struct Point3dData
{
	int objPtId;
	cv::Point3_<dType> objPt;
	cv::Point3_<uchar> objPtRgb;
	dType error;
	std::vector<int>tracks;
	Point3dData() {}
	Point3dData(const std::string& posStr)
	{
		std::vector<std::string> params = splitString(posStr, " ", true);
		CHECK(params.size() >8);
		objPtId = str2number<int>(params[0]);
		objPt.x = str2number<dType>(params[1]);
		objPt.y = str2number<dType>(params[2]);
		objPt.z = str2number<dType>(params[3]);
		objPtRgb.x = str2number<uchar>(params[4]);
		objPtRgb.y = str2number<uchar>(params[5]);
		objPtRgb.z = str2number<uchar>(params[6]);
		error = str2number<dType>(params[7]);
		tracks.reserve(params.size());
		for (size_t i = 8; i < params.size(); i++)
		{
			tracks.emplace_back(str2number<int>(params[i]));
		} 
	}
	static std::map<int,Point3dData> readPoint3dData(const std::string& points3DTXT)
	{
		std::map<int, Point3dData> ret;
		std::fstream fin(points3DTXT, std::ios::in);
		std::string aline;
		while (std::getline(fin, aline))
		{
			if (aline.length() < 1)continue;
			if (aline[0] == '#')continue;
			else
			{
				auto theObjPt = Point3dData(aline);
				ret[theObjPt.objPtId] = std::move(theObjPt);
			}
		}
		return ret;
	}
};
cv::Point_<dType> project(const Camera& camera, cv::Point3_<dType>& pt, const Eigen::Quaternion<dType>& q, const Eigen::Matrix<dType, 3, 1>& t)
{ 
	Eigen::Matrix<dType, 4, 4>Rt = Eigen::Matrix<dType, 4, 4>::Identity();
	Rt.topLeftCorner<3, 3>() = q.matrix();
	Rt.topRightCorner<3, 1>() = t; 
	Eigen::Matrix<dType, 4, 1> objPt(pt.x, pt.y, pt.z,1);
	auto cameraPt = Rt* objPt;
	dType u = cameraPt[0] / cameraPt[2];
	dType v = cameraPt[1] / cameraPt[2]; 
	return cv::Point_<dType>(u * camera.focalLength + camera.cx, v * camera.focalLength + camera.cy);
}
void testQt()
{
	std::string colmapDir = "D:/repo/mvs_mvg_bat/viewerout/colmap";
	if (colmapDir[colmapDir.length() - 1] == '\\' || colmapDir[colmapDir.length() - 1] == '/')
	{

	}
	else
	{
		colmapDir += "/";
	}
	std::string cameraTXT = colmapDir + "cameras.txt";
	std::string imagesTXT = colmapDir + "images.txt";
	std::string points3DTXT = colmapDir + "points3D.txt";
	std::vector<Camera>cameras = Camera::readCameras(cameraTXT);
	std::vector<ImageData>imgs = ImageData::readImageData(imagesTXT);
	std::map<int,Point3dData>objPts = Point3dData::readPoint3dData(points3DTXT);
	int a = 0;
	int b = 10;
	const std::vector<int>& thisObjPtsIdxA = imgs[a].thisObjPtsIdx;
	const std::vector<int>& thisObjPtsIdxB = imgs[b].thisObjPtsIdx;
	int pt_a=0;//选择这张图象的第一个  obj 点,并得到该点的3d 索引
	int objPtsIdx = thisObjPtsIdxA[pt_a];
	auto iter = std::find(thisObjPtsIdxB.begin(), thisObjPtsIdxB.end(), objPtsIdx);
	CHECK(thisObjPtsIdxB.end()!= iter);
	int pt_b = iter -thisObjPtsIdxB.begin();
	cv::Point_<dType> imgPt_a = imgs[a].thisImgPts[pt_a];
	cv::Point_<dType> imgPt_b = imgs[b].thisImgPts[pt_b];
	LOG(INFO) << "objPt = " << objPts[objPtsIdx].objPt;
	LOG(INFO) << "a = " << imgs[a].imgPath << "  " << imgPt_a << " " << project(cameras[0], objPts[objPtsIdx].objPt, imgs[a].q, imgs[a].t);
	LOG(INFO) << "b = " << imgs[b].imgPath << "  " << imgPt_b << " " << project(cameras[0], objPts[objPtsIdx].objPt, imgs[b].q, imgs[b].t); 

	return;
}
int main(int argc, char** argv)
{
	if (argc != 2)
	{
		std::cout << "cmd colmapDir" << std::endl;
		return -1;
	}
	else
	{
		std::cout << " You called:" << std::endl;
		std::cout << "  ---" << argv[0] << std::endl;
		std::cout << "  ---" << argv[1] << std::endl;
	}
	std::string colmapDir(argv[1]);
	if (colmapDir[colmapDir.length() - 1] == '\\' || colmapDir[colmapDir.length() - 1] == '/' )
	{

	}
	else
	{
		colmapDir += "/";
	}
	std::string cameraTXT = colmapDir + "cameras.txt";
	std::string imagesTXT = colmapDir + "images.txt";
	std::string points3DTXT = colmapDir + "points3D.txt";
	std::vector<Camera>cameras = Camera::readCameras(cameraTXT);
	std::vector<ImageData>imgs = ImageData::readImageData(imagesTXT);
	std::map<int, Point3dData>objPts = Point3dData::readPoint3dData(points3DTXT);
	return 0;
}