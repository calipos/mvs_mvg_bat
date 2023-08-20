#include "types.h"
#include "utils.h"
Camera::Camera() {}
CameraModel Camera::stringToCameraModel(const std::string& str)
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
Camera::Camera(const std::string& cameraStr)
{
	std::vector<std::string> params = splitString(cameraStr, " ", true);
	CHECK(params.size() > 5);
	CAMERA_ID = str2number<int>(params[0]);
	model = stringToCameraModel(params[1]);
	width = str2number<int>(params[2]);
	height = str2number<int>(params[3]);
	focalLength = str2number<float>(params[4]);
	cx = str2number<float>(params[5]);
	cy = str2number<float>(params[6]);
}
cv::Point_<dType> Camera::pixel2cam(const cv::Point_<dType>& p)
{
	return cv::Point_<dType>
		(
			(p.x - cx) / focalLength,
			(p.y - cy) / focalLength
			);
}

std::vector<Camera> Camera::readCameras(const std::string& cameraTXT)
{
	std::vector<Camera> ret;
	std::fstream fin(cameraTXT, std::ios::in);
	std::string aline;
	while (std::getline(fin, aline))
	{
		if (aline.length() < 1)continue;
		if (aline[0] == '#')continue;
		else
		{
			ret.emplace_back(aline);
		}
	}
	std::set<int>cameraIds;
	for (auto& d : ret)
	{
		cameraIds.insert(d.CAMERA_ID);
	}
	CHECK(cameraIds.size() == ret.size());
	return ret;
}


ImageData::ImageData() {}
ImageData::ImageData(const std::string& posStr, const std::string& ptsStr)
{
	std::vector<std::string> params = splitString(posStr, " ", true);
	CHECK(params.size() == 10);
	imageId = str2number<int>(params[0]);
	q.w() = str2number<dType>(params[1]);
	q.x() = str2number<dType>(params[2]);
	q.y() = str2number<dType>(params[3]);
	q.z() = str2number<dType>(params[4]);
	t[0] = str2number<dType>(params[5]);
	t[1] = str2number<dType>(params[6]);
	t[2] = str2number<dType>(params[7]);
	Eigen::Matrix<dType, 4, 4>Rt = Eigen::Matrix<dType, 4, 4>::Identity();
	camera_t = -q.matrix().transpose() * t;
	Rt.topLeftCorner<3, 3>() = q.matrix();
	Rt.topRightCorner<3, 1>() = t;
	worldToCamera = cv::Mat::eye(3, 4, cv::DataType<dType>::type);
	for (size_t i = 0; i < 3; i++)
		for (size_t j = 0; j < 4; j++)
			worldToCamera.ptr<dType>(i)[j] = Rt(i, j);
	cameraId = str2number<int>(params[8]);
	imgPath = params[9];

	std::vector<std::string> ptsData = splitString(ptsStr, " ", true);
	CHECK(ptsData.size() % 3 == 0);
	int ptsCnt = ptsData.size() / 3;
	thisImgPts.resize(ptsCnt);
	thisObjPtsIdx.resize(ptsCnt);
	for (size_t i = 0; i < ptsCnt; i++)
	{
		thisImgPts[i].x = str2number<dType>(ptsData[i * 3 + 0]);
		thisImgPts[i].y = str2number<dType>(ptsData[i * 3 + 1]);
		thisObjPtsIdx[i] = str2number<int>(ptsData[i * 3 + 2]);
	}
	//int minObjPtIdx = *std::min_element(thisObjPtsIdx.begin(), thisObjPtsIdx.end());
	//int maxObjPtIdx = *std::max_element(thisObjPtsIdx.begin(), thisObjPtsIdx.end());
	//LOG(INFO) << "minObjPtIdx = " << minObjPtIdx << "; maxObjPtIdx = " << maxObjPtIdx; 
}
dType ImageData::worldPtDepth(const cv::Point3_<dType>& X)const
{
	return worldToCamera.ptr<dType>(2)[0] * X.x + worldToCamera.ptr<dType>(2)[1] * X.y + worldToCamera.ptr<dType>(2)[2] * X.z + worldToCamera.ptr<dType>(2)[3];
}
std::map<int, ImageData> ImageData::readImageData(const std::string& imagesTXT)
{
	std::map<int, ImageData> ret;
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
			ImageData temp(aline, aline2);
			ret[temp.imageId] = std::move(temp);
		}
	}
	return ret;
}

Point3dData::Point3dData() {}
Point3dData::Point3dData(const std::string& posStr)
{
	std::vector<std::string> params = splitString(posStr, " ", true);
	CHECK(params.size() > 8);
	objPtId = str2number<int>(params[0]);
	objPt.x = str2number<dType>(params[1]);
	objPt.y = str2number<dType>(params[2]);
	objPt.z = str2number<dType>(params[3]);
	objPtRgb.x = str2number<uchar>(params[4]);
	objPtRgb.y = str2number<uchar>(params[5]);
	objPtRgb.z = str2number<uchar>(params[6]);
	error = str2number<dType>(params[7]);
	CHECK(params.size()%2==0);
	for (size_t i = 8; i < params.size(); i+=2)
	{
		int imgId = str2number<int>(params[i]);
		int imgPtId = str2number<int>(params[i+1]);
		tracks_imgId_imgPtId[imgId] = imgPtId;
	}
}
std::map<int, Point3dData> Point3dData::readPoint3dData(const std::string& points3DTXT)
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