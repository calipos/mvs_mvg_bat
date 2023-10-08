#ifndef _SERIALIZATION_H_
#define _SERIALIZATION_H_ 

#include <vector>
#include <fstream>
#include <string>
#include <sstream>
#include "opencv2/opencv.hpp"
#include <sstream>
#include <boost/serialization/serialization.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/map.hpp> 
#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <boost/serialization/split_free.hpp>
#include "utils.h"
#include "glog/logging.h" 
#ifdef _MSC_VER
#ifndef GLOG_NO_ABBREVIATED_SEVERITIES
#define GLOG_NO_ABBREVIATED_SEVERITIES
#endif // !GLOG_NO_ABBREVIATED_SEVERITIES
#endif 
namespace boost {
	namespace serialization {
		template<class Archive>
		void serialize(Archive& ar, cv::Mat& mat, const unsigned int)
		{
			if (mat.empty())
			{
				mat = cv::Mat(0, 0, CV_8UC1);
			}
			int cols, rows, type;
			bool continuous;

			if (Archive::is_saving::value) {
				cols = mat.cols; rows = mat.rows; type = mat.type();
				continuous = mat.isContinuous();
			}

			ar& cols& rows& type& continuous;

			if (Archive::is_loading::value)
				mat.create(rows, cols, type);

			if (continuous) {
				const unsigned int data_size = rows * cols * mat.elemSize();
				ar& boost::serialization::make_array(mat.ptr(), data_size);
			}
			else {
				const unsigned int row_size = cols * mat.elemSize();
				for (int i = 0; i < rows; i++) {
					ar& boost::serialization::make_array(mat.ptr(i), row_size);
				}
			}
		}
		template<class Archive>
		void serialize(Archive& ar, cv::Point& pt, const unsigned int)
		{
			ar& pt.x& pt.y;
		}
		template<class Archive>
		void serialize(Archive& ar, cv::Point2f& pt, const unsigned int)
		{
			ar& pt.x& pt.y;
		}
		template<class Archive>
		void serialize(Archive& ar, cv::Point2d& pt, const unsigned int)
		{
			ar& pt.x& pt.y;
		}
		template<class Archive>
		void serialize(Archive& ar, cv::KeyPoint& pt, const unsigned int)
		{
			ar& pt.pt& pt.size& pt.angle& pt.response& pt.octave& pt.class_id;
		}
		template<class Archive>
		void serialize(Archive& ar, cv::Point3f& pt, const unsigned int)
		{
			ar& pt.x& pt.y& pt.z;
		}
		template<class Archive>
		void serialize(Archive& ar, cv::Point3d& pt, const unsigned int)
		{
			ar& pt.x& pt.y& pt.z;
		}
		template<class Archive>
		void serialize(Archive& ar, cv::Point3i& pt, const unsigned int)
		{
			ar& pt.x& pt.y& pt.z;
		}
		template<class Archive>
		void serialize(Archive& ar, std::tuple<std::vector<cv::KeyPoint>, cv::Mat, cv::Mat, cv::Mat>& wildFeature, const unsigned int)
		{
			ar& std::get<0>(wildFeature)& std::get<1>(wildFeature)& std::get<2>(wildFeature)& std::get<3>(wildFeature);
		}

		template <class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
		void save(Archive& ar, const Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& m, const unsigned int version) {
			Eigen::Index rows = m.rows(), cols = m.cols();
			ar& rows;
			ar& cols;
			ar& boost::serialization::make_array(m.data(), rows * cols);
		}
		template <class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
		void load(Archive& ar, Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& m, const unsigned int version) {
			Eigen::Index rows, cols;
			ar& rows;
			ar& cols;
			m.resize(rows, cols);
			ar& boost::serialization::make_array(m.data(), rows * cols);
		}
		template <class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
		void serialize(Archive& ar, Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& m, const unsigned int version) {
			split_free(ar, m, version);
		}


		template <class Archive, typename _Scalar>
		void save(Archive& ar, const Eigen::Triplet<_Scalar>& m, const unsigned int version) {
			ar& m.row();
			ar& m.col();
			ar& m.value();
		}
		template <class Archive, typename _Scalar>
		void load(Archive& ar, Eigen::Triplet<_Scalar>& m, const unsigned int version) {
			Eigen::Index row, col;
			_Scalar value;
			ar& row;
			ar& col;
			ar& value;
			m = Eigen::Triplet<_Scalar>(row, col, value);
		}
		template <class Archive, typename _Scalar>
		void serialize(Archive& ar, Eigen::Triplet<_Scalar>& m, const unsigned int version) {
			split_free(ar, m, version);
		}
		template <class Archive, typename _Scalar, int _Options, typename _Index>
		void save(Archive& ar, const Eigen::SparseMatrix<_Scalar, _Options, _Index>& m, const unsigned int version) {
			Eigen::Index innerSize = m.innerSize();
			Eigen::Index outerSize = m.outerSize();
			typedef typename Eigen::Triplet<_Scalar> Triplet;
			std::vector<Triplet> triplets;
			for (Eigen::Index i = 0; i < outerSize; ++i) {
				for (typename Eigen::SparseMatrix<_Scalar, _Options, _Index>::InnerIterator it(m, i); it; ++it) {
					triplets.push_back(Triplet(it.row(), it.col(), it.value()));
				}
			}
			ar& innerSize;
			ar& outerSize;
			ar& triplets;
		}
		template <class Archive, typename _Scalar, int _Options, typename _Index>
		void load(Archive& ar, Eigen::SparseMatrix<_Scalar, _Options, _Index>& m, const unsigned int version) {
			Eigen::Index innerSize;
			Eigen::Index outerSize;
			ar& innerSize;
			ar& outerSize;
			Eigen::Index rows = m.IsRowMajor ? outerSize : innerSize;
			Eigen::Index cols = m.IsRowMajor ? innerSize : outerSize;
			m.resize(rows, cols);
			typedef typename Eigen::Triplet<_Scalar> Triplet;
			std::vector<Triplet> triplets;
			ar& triplets;
			m.setFromTriplets(triplets.begin(), triplets.end());

		}
		template <class Archive, typename _Scalar, int _Options, typename _Index>
		void serialize(Archive& ar, Eigen::SparseMatrix<_Scalar, _Options, _Index>& m, const unsigned int version) {
			split_free(ar, m, version);
		}
	}
}

template<class Dtype>
int save_bin_structure(const Dtype& s, const std::string filename) 
{
	try
	{
		std::ofstream ofs(filename, std::ios::out | std::ios::binary);
		boost::archive::binary_oarchive oa(ofs);
		oa << s;
	}
	catch (const std::exception&)
	{
		LOG(INFO) << "save_bin_structure ERROR : " << filename;
		return -1;
	}
	return 0;
}
template<class Dtype>
int load_bin_structure(Dtype& s, const std::string filename) 
{
	API_START(0);
	try
	{
		std::ifstream ifs(filename, std::ios::in | std::ios::binary);
		boost::archive::binary_iarchive ia(ifs);
		ia >> s;
	}
	catch (const std::exception&)
	{
		LOG(INFO) << "load_bin_structure ERROR : " << filename;
		return -1;
	} 
	API_END(0, "load_bin_structure");
	return 0;
}

#if TEST_SERIALIZATION>0
//https://blog.csdn.net/liulong1567/article/details/23845155
class SlefData
{
public:
	SlefData() {};
	~SlefData() {};
	std::vector<cv::Mat>a1;
	std::vector<cv::Point>a2;
	cv::Point2f a3;
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive& ar, const unsigned int version)
	{
		ar& a1;
		ar& a2;
		if (version > 0)
			ar& a3;
	}

};
BOOST_CLASS_VERSION(SlefData, 1)  // 通过该宏指示当前类的版本更新为1版本
inline void test_serialization()
{
	SlefData d4;
	load_bin_structure(d4, "a.dat");


	SlefData d1;
	d1.a1.resize(2);
	d1.a1[0] = cv::Mat::zeros(3, 3, CV_32FC1);
	d1.a1[1] = cv::Mat::ones(3, 3, CV_64FC1);
	d1.a2.resize(2);
	d1.a2[0] = cv::Point(3, 3);
	std::stringstream ss;

	boost::archive::binary_oarchive oa(ss);
	oa << d1;

	SlefData d2;
	boost::archive::binary_iarchive ia(ss);
	ia >> d2;

	save_bin_structure(d1, "a.dat");
	SlefData d3;
	load_bin_structure(d3, "a.dat");
	return;
}
#endif // TEST_SERIALIZATION>0

#endif // !_SERIALIZATION_H_
