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
#include "types.h"
#include "SelectViews.h" 
#include "omp.h"

 
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
	std::map<int, ImageData>imgs = ImageData::readImageData(imagesTXT);
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
std::vector<cv::Point_<dType>> getStandardImgPts(const cv::Size& size)
{
	std::vector<cv::Point_<dType>> ret(size.width* size.height);
#pragma omp parallel for
	for (int i = 0; i < ret.size(); i++)
	{
		ret[i].x = i % size.width;
		ret[i].y = i / size.width;
	}
	return ret;
}
std::vector<cv::Point_<dType>> getFlowImgPts(const std::vector<cv::Point_<dType>>&standardImgPts,const std::string&flowPath)
{
	std::vector<cv::Point_<dType>> flowImgPts(standardImgPts.size());
	size_t size = 0;
	{
		std::ifstream in(flowPath);
		in.seekg(0, std::ios::end);
		size = in.tellg();
		in.close(); 
	}
	CHECK(standardImgPts.size() * 2 * sizeof(float) == size);
	std::vector<float> flowData(standardImgPts.size() * 2);
	{
		std::ifstream in(flowPath,std::ios::binary);
		in.read((char*)&flowData[0],size);
		in.close();
	}
#pragma omp parallel for
	for (int i = 0; i < standardImgPts.size(); i++)
	{
		flowImgPts[i].x = standardImgPts[i].x + flowData[2 * i];
		flowImgPts[i].y = standardImgPts[i].y + flowData[2 * i+1];;
	}
	return flowImgPts;
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
	std::map<int, ImageData>imgs = ImageData::readImageData(imagesTXT);
	std::map<int, Point3dData>objPts = Point3dData::readPoint3dData(points3DTXT);


	SelectNeighborViews(cameras, imgs, objPts);





	std::vector<int>imgKeys;
	imgKeys.reserve(imgs.size());
	for (auto&d: imgs)
	{
		imgKeys.emplace_back(d.first);
	}
	std::sort(imgKeys.begin(), imgKeys.end());
	LOG(INFO) << "img size = " << imgKeys.size();
	cv::Mat imgFirst = cv::imread("viewer/"+imgs[imgKeys[0]].imgPath);
	CHECK(!imgFirst.empty());
	std::vector<cv::Point_<dType>>standardImgPts = getStandardImgPts(imgFirst.size());
	std::vector<cv::Point_<dType>>standardCameraPts(standardImgPts.size());
#pragma omp parallel for
	for (int p = 0; p < standardImgPts.size(); p++)
	{
		standardCameraPts[p] = cameras[0].pixel2cam(standardImgPts[p]);
	}
	for (size_t i = 0; i < imgKeys.size()-1; i++)
	{
		std::string imgAPath = imgs[imgKeys[i]].imgPath;
		std::string imgBPath = imgs[imgKeys[i + 1]].imgPath;
		std::string imgBFlowPath ="RAFT/"+ imgAPath + "-" + imgBPath + ".bin";
		std::vector<cv::Point_<dType>>flowImgPts = getFlowImgPts(standardImgPts, imgBFlowPath);
		std::vector<cv::Point_<dType>>flowCameraPts(flowImgPts.size());
#pragma omp parallel for
		for (int p = 0; p < standardImgPts.size(); p++)
		{
			flowCameraPts[p] = cameras[0].pixel2cam(flowImgPts[p]);
		}
		cv::Mat_<dType> pts;
		cv::triangulatePoints(imgs[imgKeys[i]].worldToCamera, imgs[imgKeys[i + 1]].worldToCamera, standardCameraPts, flowCameraPts, pts);
		writePts("RAFT/" + imgAPath + ".pts", pts);
	}
	//
	return 0;
}