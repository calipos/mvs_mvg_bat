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
	using dType = double;
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
		std::vector<std::string> ptsData = splitString(ptsStr, " ", true);
		CHECK(params.size() ==10);
		CHECK(ptsData.size() % 3 == 0);
		imageId = str2number<int>(params[0]);
		q.w() = str2number<dType>(params[1]);
		q.x() = str2number<dType>(params[2]);
		q.y() = str2number<dType>(params[3]);
		q.z() = str2number<dType>(params[4]);
		t[0] = str2number<dType>(params[5]);
		t[1] = str2number<dType>(params[6]);
		t[2] = str2number<dType>(params[7]);
		//cameraId = str2number<int>(params[8]);
		imgPath = params[9];
		int ptsCnt = ptsData.size() /3;
		thisImgPts.resize(ptsCnt);
		thisObjPtsIdx.resize(ptsCnt);
		for (size_t i = 0; i < ptsCnt; i++)
		{
			thisImgPts[i].x = str2number<dType>(ptsData[i * 3 + 0]);
			thisImgPts[i].y = str2number<dType>(ptsData[i * 3 + 1]);
			thisObjPtsIdx[i] = str2number<int>(ptsData[i * 3 + 2]);
		}
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
		//std::set<int>cameraIds;
		//for (auto& d : ret)
		//{
		//	cameraIds.insert(d.CAMERA_ID);
		//}
		//CHECK(cameraIds.size() == ret.size());
		return ret;
	}
};
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
	return 0;
}