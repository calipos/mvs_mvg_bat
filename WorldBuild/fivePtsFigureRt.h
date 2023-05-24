#ifndef _FIVE_POINTS_FIGURE_RT_H_
#define _FIVE_POINTS_FIGURE_RT_H_
#include "opencv2/opencv.hpp"
#include "glog/logging.h"
#include "Eigen/Core"
#include <Eigen/Dense>


namespace rt
{
	enum
	{
		coef_xxx,
		coef_xxy,
		coef_xyy,
		coef_yyy,
		coef_xxz,
		coef_xyz,
		coef_yyz,
		coef_xzz,
		coef_yzz,
		coef_zzz,
		coef_xx,
		coef_xy,
		coef_yy,
		coef_xz,
		coef_yz,
		coef_zz,
		coef_x,
		coef_y,
		coef_z,
		coef_1
	};

	Eigen::VectorXd o1(const Eigen::VectorXd& a, const Eigen::VectorXd& b);

	Eigen::VectorXd o2(const Eigen::VectorXd& a, const Eigen::VectorXd& b);

	Eigen::MatrixXd FivePointsPolynomialConstraints(const Eigen::MatrixXd& E_basis);

	Eigen::Matrix<double, 9, 4>   FivePointsNullspaceBasis(const std::vector<cv::Point3d>& p1_, const std::vector<cv::Point3d>& p2_);
	void Errors(const Eigen::Matrix3d& model, const Eigen::Matrix3d& K1_, const Eigen::Matrix3d& K2_,
		const std::vector<cv::Point3d>& p1, const std::vector<cv::Point3d>& p2, std::vector<double>& vec_errors);

	int fivePtsFigureRt(const Eigen::MatrixXd& p1, const Eigen::MatrixXd& p2, const Eigen::Matrix3d& K1, const Eigen::Matrix3d& K2, Eigen::Matrix3d& model);
}
#endif // !_FIVE_POINTS_FIGURE_RT_H_
