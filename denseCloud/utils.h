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




#ifdef _FAST_FLOAT2INT
// fast float to int conversion
// (xs routines at stereopsis: http://www.stereopsis.com/sree/fpu2006.html by Sree Kotay)
const double _float2int_doublemagic = 6755399441055744.0; //2^52 * 1.5, uses limited precision to floor
const double _float2int_doublemagicdelta = (1.5e-8);
const double _float2int_doublemagicroundeps = (.5f - _float2int_doublemagicdelta); //almost .5f = .5f - 1e^(number of exp bit)
FORCEINLINE int CRound2Int(const double& x) {
	const CastD2I c(x + _float2int_doublemagic);
	ASSERT(int32_t(floor(x + .5)) == c.i);
	return c.i;
}
#endif
template <typename INTTYPE = int>
inline INTTYPE Floor2Int(float x) {
#ifdef _FAST_FLOAT2INT
	return CRound2Int(double(x) - _float2int_doublemagicroundeps);
#else
	return static_cast<INTTYPE>(floor(x));
#endif
}
template <typename INTTYPE = int>
inline INTTYPE Floor2Int(double x) {
#ifdef _FAST_FLOAT2INT
	return CRound2Int(x - _float2int_doublemagicroundeps);
#else
	return static_cast<INTTYPE>(floor(x));
#endif
}
template <typename INTTYPE = int>
inline INTTYPE Ceil2Int(float x) {
#ifdef _FAST_FLOAT2INT
	return CRound2Int(double(x) + _float2int_doublemagicroundeps);
#else
	return static_cast<INTTYPE>(ceil(x));
#endif
}
template <typename INTTYPE = int>
inline INTTYPE Ceil2Int(double x) {
#ifdef _FAST_FLOAT2INT
	return CRound2Int(x + _float2int_doublemagicroundeps);
#else
	return static_cast<INTTYPE>(ceil(x));
#endif
}
template <typename INTTYPE = int>
inline INTTYPE Round2Int(float x) {
#ifdef _FAST_FLOAT2INT
	return CRound2Int(double(x) + _float2int_doublemagicdelta);
#else
	return static_cast<INTTYPE>(floor(x + .5f));
#endif
}
template <typename INTTYPE = int>
inline INTTYPE Round2Int(double x) {
#ifdef _FAST_FLOAT2INT
	return CRound2Int(x + _float2int_doublemagicdelta);
#else
	return static_cast<INTTYPE>(floor(x + .5));
#endif
}
template<typename T>
constexpr T powi(T base, unsigned exp) {
	T result(1);
	while (exp) {
		if (exp & 1)
			result *= base;
		exp >>= 1;
		base *= base;
	}
	return result;
}
constexpr int log2i(unsigned val) {
	int ret = -1;
	while (val) {
		val >>= 1;
		++ret;
	}
	return ret;
}
template<typename T>
inline T EXP(const T& a) {
	return T(exp(a));
}
#define FLOOR			Floor2Int
#define FLOOR2INT		Floor2Int
#define CEIL			Ceil2Int
#define CEIL2INT		Ceil2Int
#define ROUND			Round2Int
#define ROUND2INT		Round2Int
#define SIN				std::sin
#define ASIN			std::asin
#define COS				std::cos
#define ACOS			std::acos
#define TAN				std::tan
#define ATAN			std::atan
#define ATAN2			std::atan2
#define POW				std::pow
#define POWI			powi
#define LOG2I			log2i


template<typename _Tp>
inline bool   ISINSIDE(_Tp v, _Tp l0, _Tp l1) { CHECK(l0 < l1); return l0 <= v && v < l1; }
template<typename _Tp>
inline _Tp    CLAMP(_Tp v, _Tp l0, _Tp l1) { CHECK(l0 <= l1); return std::min(std::max(v, l0), l1); }
template<typename DtypeIn, typename DtypeOut = DtypeIn>
inline DtypeOut ComputeAngle(const cv::Point3_<DtypeIn>& a, const cv::Point3_<DtypeIn>& b)
{
	DtypeIn dot = a.dot(b);
	DtypeIn n1 = cv::norm(a);
	DtypeIn n2 = cv::norm(b);
	return CLAMP(DtypeOut(dot/n1/n2), DtypeOut (-1), DtypeOut(1));
}

template<typename T>
constexpr T SQUARE(const T& a) {
	return a * a;
}

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
template <typename TYPE>
inline bool IsInside(const cv::Point_<TYPE>& pt, const cv::Point_<TYPE>& size) 
{
	return pt.x >= 0 && pt.y >= 0 && pt.x < size.x&& pt.y < size.y;
}
// given an array of values and their bound, approximate the area covered, in percentage
template<typename TYPE, int n, int s, bool bCentered>
inline TYPE ComputeCoveredArea(const std::vector<cv::Point_<TYPE>>values, const cv::Point2f bound, int stride = n) {
	CHECK(values.size() > 0);
	typedef Eigen::Matrix<TYPE, 1, n, Eigen::RowMajor> Vector;
	typedef Eigen::Map<const Vector, Eigen::Unaligned> MapVector;
	typedef Eigen::Matrix<TYPE, Eigen::Dynamic, n, Eigen::RowMajor> Matrix;
	typedef Eigen::Map<const Matrix, Eigen::Unaligned, Eigen::OuterStride<> > MapMatrix;
	typedef Eigen::Matrix<unsigned, s, s, Eigen::RowMajor> MatrixSurface;
	const MapMatrix points(&(values[0].x), values.size(), n, Eigen::OuterStride<>(stride));
	const Vector norm(bound.x, bound.y);
	const Vector offset(Vector::Constant(bCentered ? TYPE(0.5) : TYPE(0)));
	MatrixSurface surface;
	surface.setZero();
	for (size_t i = 0; i < values.size(); ++i) {
		const Vector point((points.row(i).cwiseQuotient(norm) + offset) * TYPE(s));
		CHECK((point(0) >= 0 && point(0) < s) && (point(1) >= 0 && point(1) < s));
		surface(FLOOR2INT(point(0)), FLOOR2INT(point(1))) = 1;
	}
	return TYPE(surface.sum()) / (s * s);
} // ComputeCoveredArea
#endif // !_UTILS_H_