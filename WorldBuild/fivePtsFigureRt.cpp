#include "fivePtsFigureRt.h"
#include "opencv2/opencv.hpp"
#include "glog/logging.h"
#include "Eigen/Core"
#include <Eigen/Dense>
#include <numeric>
#include "common.h"
#include "essentialDecomposeIntoRt.h"
namespace rt
{ 
	Eigen::VectorXd o1(const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
		Eigen::VectorXd res = Eigen::VectorXd::Zero(20);

		res(coef_xx) = a(coef_x) * b(coef_x);
		res(coef_xy) = a(coef_x) * b(coef_y)
			+ a(coef_y) * b(coef_x);
		res(coef_xz) = a(coef_x) * b(coef_z)
			+ a(coef_z) * b(coef_x);
		res(coef_yy) = a(coef_y) * b(coef_y);
		res(coef_yz) = a(coef_y) * b(coef_z)
			+ a(coef_z) * b(coef_y);
		res(coef_zz) = a(coef_z) * b(coef_z);
		res(coef_x) = a(coef_x) * b(coef_1)
			+ a(coef_1) * b(coef_x);
		res(coef_y) = a(coef_y) * b(coef_1)
			+ a(coef_1) * b(coef_y);
		res(coef_z) = a(coef_z) * b(coef_1)
			+ a(coef_1) * b(coef_z);
		res(coef_1) = a(coef_1) * b(coef_1);

		return res;
	}

	Eigen::VectorXd o2(const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
		Eigen::VectorXd res(20);

		res(coef_xxx) = a(coef_xx) * b(coef_x);
		res(coef_xxy) = a(coef_xx) * b(coef_y)
			+ a(coef_xy) * b(coef_x);
		res(coef_xxz) = a(coef_xx) * b(coef_z)
			+ a(coef_xz) * b(coef_x);
		res(coef_xyy) = a(coef_xy) * b(coef_y)
			+ a(coef_yy) * b(coef_x);
		res(coef_xyz) = a(coef_xy) * b(coef_z)
			+ a(coef_yz) * b(coef_x)
			+ a(coef_xz) * b(coef_y);
		res(coef_xzz) = a(coef_xz) * b(coef_z)
			+ a(coef_zz) * b(coef_x);
		res(coef_yyy) = a(coef_yy) * b(coef_y);
		res(coef_yyz) = a(coef_yy) * b(coef_z)
			+ a(coef_yz) * b(coef_y);
		res(coef_yzz) = a(coef_yz) * b(coef_z)
			+ a(coef_zz) * b(coef_y);
		res(coef_zzz) = a(coef_zz) * b(coef_z);
		res(coef_xx) = a(coef_xx) * b(coef_1)
			+ a(coef_x) * b(coef_x);
		res(coef_xy) = a(coef_xy) * b(coef_1)
			+ a(coef_x) * b(coef_y)
			+ a(coef_y) * b(coef_x);
		res(coef_xz) = a(coef_xz) * b(coef_1)
			+ a(coef_x) * b(coef_z)
			+ a(coef_z) * b(coef_x);
		res(coef_yy) = a(coef_yy) * b(coef_1)
			+ a(coef_y) * b(coef_y);
		res(coef_yz) = a(coef_yz) * b(coef_1)
			+ a(coef_y) * b(coef_z)
			+ a(coef_z) * b(coef_y);
		res(coef_zz) = a(coef_zz) * b(coef_1)
			+ a(coef_z) * b(coef_z);
		res(coef_x) = a(coef_x) * b(coef_1)
			+ a(coef_1) * b(coef_x);
		res(coef_y) = a(coef_y) * b(coef_1)
			+ a(coef_1) * b(coef_y);
		res(coef_z) = a(coef_z) * b(coef_1)
			+ a(coef_1) * b(coef_z);
		res(coef_1) = a(coef_1) * b(coef_1);

		return res;
	}

	Eigen::MatrixXd FivePointsPolynomialConstraints(const Eigen::MatrixXd& E_basis) {
		// Build the polynomial form of E (equation (8) in Stewenius et al. [1])
		Eigen::VectorXd E[3][3];
		for (int i = 0; i < 3; ++i) {
			for (int j = 0; j < 3; ++j) {
				E[i][j] = Eigen::VectorXd::Zero(20);
				E[i][j](coef_x) = E_basis(3 * i + j, 0);
				E[i][j](coef_y) = E_basis(3 * i + j, 1);
				E[i][j](coef_z) = E_basis(3 * i + j, 2);
				E[i][j](coef_1) = E_basis(3 * i + j, 3);
			}
		}

		// The constraint matrix.
		Eigen::MatrixXd M(10, 20);
		int mrow = 0;

		// Determinant constraint det(E) = 0; equation (19) of Nister [2].
		M.row(mrow++) = o2(o1(E[0][1], E[1][2]) - o1(E[0][2], E[1][1]), E[2][0]) +
			o2(o1(E[0][2], E[1][0]) - o1(E[0][0], E[1][2]), E[2][1]) +
			o2(o1(E[0][0], E[1][1]) - o1(E[0][1], E[1][0]), E[2][2]);

		// Cubic singular values constraint.
		// Equation (20).
		Eigen::VectorXd EET[3][3];
		for (int i = 0; i < 3; ++i) {    // Since EET is symmetric, we only compute
			for (int j = 0; j < 3; ++j) {  // its upper triangular part.
				if (i <= j) {
					EET[i][j] = o1(E[i][0], E[j][0])
						+ o1(E[i][1], E[j][1])
						+ o1(E[i][2], E[j][2]);
				}
				else {
					EET[i][j] = EET[j][i];
				}
			}
		}

		// Equation (21).
		Eigen::VectorXd(&L)[3][3] = EET;
		const Eigen::VectorXd trace = 0.5 * (EET[0][0] + EET[1][1] + EET[2][2]);
		for (const int i : {0, 1, 2}) {
			L[i][i] -= trace;
		}

		// Equation (23).
		for (const int i : {0, 1, 2}) {
			for (const int j : {0, 1, 2}) {
				Eigen::VectorXd LEij = o2(L[i][0], E[0][j])
					+ o2(L[i][1], E[1][j])
					+ o2(L[i][2], E[2][j]);
				M.row(mrow++) = LEij;
			}
		}

		return M;
	}

	Eigen::Matrix<double, 9, 4>   FivePointsNullspaceBasis(const Eigen::MatrixXd& p1_, const Eigen::MatrixXd& p2_)
	{
		CHECK(p1_.cols() == p2_.cols() && p1_.cols() >= 5 && p1_.cols() < 9);
		Eigen::Matrix<double, 9, 9> epipolar_constraint = Eigen::Matrix<double, 9, 9>::Zero();
		epipolar_constraint.resize(9, 9);
		for (int i = 0; i < p1_.cols(); i++)
		{
			epipolar_constraint(i, 0) = p2_(0,i) * p1_(0,i);
			epipolar_constraint(i, 1) = p2_(0,i) * p1_(1,i);
			epipolar_constraint(i, 2) = p2_(0,i) * p1_(2,i);
			epipolar_constraint(i, 3) = p2_(1,i) * p1_(0,i);
			epipolar_constraint(i, 4) = p2_(1,i) * p1_(1,i);
			epipolar_constraint(i, 5) = p2_(1,i) * p1_(2,i);
			epipolar_constraint(i, 6) = p1_(0,i) * p2_(2,i);
			epipolar_constraint(i, 7) = p1_(1,i) * p2_(2,i);
			epipolar_constraint(i, 8) = p1_(2,i) * p2_(2,i);
		}
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>> solver
		(epipolar_constraint.transpose() * epipolar_constraint);
		return solver.eigenvectors().leftCols<4>();
	}

	void Errors(const Eigen::Matrix3d& model, const Eigen::Matrix3d& K1_, const Eigen::Matrix3d& K2_,
		const Eigen::MatrixXd& p1, const Eigen::MatrixXd& p2, std::vector<double>& vec_errors)
	{
		vec_errors.resize(p1.cols());
		Eigen::Matrix3d F = K2_.inverse().transpose() * model * K1_.inverse();
		for (uint32_t sample = 0; sample < p1.cols(); ++sample)
		{			
			const Eigen::Vector3d F_x = F * p1.col(sample);
			double y_F_x = F_x.dot(p2.col(sample));
			//vec_errors[sample] = y_F_x * y_F_x / F_x.head<2>().squaredNorm();
			vec_errors[sample] = y_F_x * y_F_x;
		}
	}




	int fivePtsFigureRt(const Eigen::MatrixXd& p1, const Eigen::MatrixXd& p2, const Eigen::Matrix3d& K1, const Eigen::Matrix3d& K2, Eigen::Matrix3d&model)
	{
		// Step 1: Nullspace Extraction.
		Eigen::Matrix<double, 9, 4> E_basis = rt::FivePointsNullspaceBasis(p1, p2);
		// Step 2: Constraint Expansion.
		Eigen::MatrixXd E_constraints = rt::FivePointsPolynomialConstraints(E_basis);
		// Step 3: Gauss-Jordan Elimination (done thanks to a LU decomposition). 
		Eigen::FullPivLU<Eigen::Matrix<double, 10, 10>> c_lu(E_constraints.block<10, 10>(0, 0));
		const Eigen::Matrix<double, 10, 10> M = c_lu.solve(E_constraints.block<10, 10>(0, 10));

		// Build action matrix.

		const Eigen::Matrix<double, 10, 10>& B = M.topRightCorner<10, 10>();
		Eigen::Matrix<double, 10, 10> At = Eigen::Matrix<double, 10, 10>::Zero(10, 10);
		At.block<3, 10>(0, 0) = B.block<3, 10>(0, 0);
		At.row(3) = B.row(4);
		At.row(4) = B.row(5);
		At.row(5) = B.row(7);
		At(6, 0) = At(7, 1) = At(8, 3) = At(9, 6) = -1;
		Eigen::EigenSolver<Eigen::Matrix<double, 10, 10>> eigensolver(At);
		const auto& eigenvectors = eigensolver.eigenvectors();
		const auto& eigenvalues = eigensolver.eigenvalues();
		// Build essential matrices for the real solutions.
		std::vector<Eigen::Matrix3d>  vec_models;
		vec_models.reserve(10);
		for (int s = 0; s < 10; ++s) {
			// Only consider real solutions.
			if (eigenvalues(s).imag() != 0) {
				continue;
			}
			Eigen::Matrix3d E;
			Eigen::Map<Eigen::Matrix<double, 9, 1> >(E.data()) = E_basis * eigenvectors.col(s).tail<4>().real();
			vec_models.emplace_back(E.transpose());
		}
		std::vector<double>errs;
		errs.reserve(vec_models.size());
		for (const auto& model_it : vec_models)
		{
			// Compute residual values
			std::vector<double>  vec_errors;
			Errors(model_it, K1,K2,p1,p2, vec_errors);			
			errs.emplace_back(std::accumulate(vec_errors.begin(), vec_errors.end(), 0.) / vec_errors.size());
		}
		int minErrIter = std::min_element(errs.begin(), errs.end())- errs.begin();
		model = vec_models[minErrIter];


		std::vector<uint32_t>vec_inliers(p1.cols());
		for (size_t i = 0; i < p1.cols(); i++)
		{
			vec_inliers[i] = i;
		}
		Pose3 relative_pose;
		if (!RelativePoseFromEssential(
			p1,
			p2,
			model,
			vec_inliers, &relative_pose))
		{
			return false;
		}
		
		const Pose3& pose_I =  { Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero() };
		const Pose3& pose_J = relative_pose;


		std::cout << relative_pose.rotation() << std::endl;
		std::cout << relative_pose.translation() << std::endl;
		return 0;
	}

}