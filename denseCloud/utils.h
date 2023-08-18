#ifndef _UTILS_H_
#define _UTILS_H_

#include <iostream>
#include <set>
#include <vector>
#include <fstream>
#include <string> 
#include <sstream> 
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "opencv2/opencv.hpp"




template<typename Dtype>
static inline int writePts2d(const std::string& path, const std::vector<cv::Point_<Dtype>>& pts)
{
	std::string aline;
	std::fstream fin1(path, std::ios::out);
	for (int i = 0; i < pts.size(); i++)
	{
		fin1 << pts[i].x << " " << pts[i].y << std::endl;
	}
	fin1.close();
	return 0;
}
template<typename Dtype>
int writePts(const std::string& path, const std::vector<cv::Point3_<Dtype>>& pts)
{
	std::string aline;
	std::fstream fin1(path, std::ios::out);
	for (int i = 0; i < pts.size(); i++)
	{
		fin1 << pts[i].x << " " << pts[i].y << " " << pts[i].z << std::endl;
	}
	fin1.close();
	return 0;
}
template<typename Dtype>
int writePts(const std::string& path, const cv::Mat_<Dtype>& pts)
{
	std::string aline;
	std::fstream fin1(path, std::ios::out);
	for (int i = 0; i < pts.cols; i++)
	{
		cv::Mat x = pts.col(i);
		x /= x.at<Dtype>(3, 0); // πÈ“ªªØ
		fin1 << x.at<Dtype>(0, 0) << " " << x.at<Dtype>(1, 0) << " " << x.at<Dtype>(2, 0) << std::endl;
	}
	fin1.close();
	return 0;
}
template<typename Dtype = int>
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

template<typename Dtype>
Dtype str2number(const std::string& str)
{
	std::stringstream ss(str);
	Dtype ret;
	ss >> ret;
	return   ret;
}

template<class Dtype>
cv::Mat eigen2mat(const Eigen::MatrixBase<Eigen::Matrix<Dtype, 4, 4, 0, 4, 4>>& Rt)
{
	cv::Mat moveMat(4, 4, cv::DataType<Dtype>::type);
	moveMat.ptr<Dtype>(0)[0] = Rt(0, 0);
	moveMat.ptr<Dtype>(0)[1] = Rt(0, 1);
	moveMat.ptr<Dtype>(0)[2] = Rt(0, 2);
	moveMat.ptr<Dtype>(0)[3] = Rt(0, 3);
	moveMat.ptr<Dtype>(1)[0] = Rt(1, 0);
	moveMat.ptr<Dtype>(1)[1] = Rt(1, 1);
	moveMat.ptr<Dtype>(1)[2] = Rt(1, 2);
	moveMat.ptr<Dtype>(1)[3] = Rt(1, 3);
	moveMat.ptr<Dtype>(2)[0] = Rt(2, 0);
	moveMat.ptr<Dtype>(2)[1] = Rt(2, 1);
	moveMat.ptr<Dtype>(2)[2] = Rt(2, 2);
	moveMat.ptr<Dtype>(2)[3] = Rt(2, 3);
	moveMat.ptr<Dtype>(3)[0] = Rt(3, 0);
	moveMat.ptr<Dtype>(3)[1] = Rt(3, 1);
	moveMat.ptr<Dtype>(3)[2] = Rt(3, 2);
	moveMat.ptr<Dtype>(3)[3] = Rt(3, 3);
	return moveMat;
};
template<typename Dtype, int _Rows, int _Cols, int _Flag>
inline cv::Mat eigenToMat(const Eigen::Matrix<Dtype, _Rows, _Cols, _Flag, _Rows, _Cols>& m)
{
	cv::Mat ret(m.rows(), m.cols(), cv::DataType<Dtype>::type);
	for (int r = 0; r < m.rows(); r++)
	{
		for (int c = 0; c < m.cols(); c++)
		{
			ret.ptr<Dtype>(r)[c] = m(r, c);
		}
	}
	return ret;
}
template<class Dtype>
cv::Point3_<Dtype> eigen2pt(const Eigen::Matrix<Dtype, 3, 1>& t)
{
	return cv::Point3_<Dtype>(t[0],t[1],t[2]);
};
#endif // !_UTILS_H_