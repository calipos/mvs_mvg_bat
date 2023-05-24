#include "essentialDecomposeIntoRt.h"
#include <array>
namespace rt
{

	// HZ 9.7 page 259 (Result 9.19)
	void MotionFromEssential(const Eigen::Matrix3d& E,
		std::vector<Pose3>* relative_poses) {
		Eigen::JacobiSVD<Eigen::Matrix3d> USV(E, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Eigen::Matrix3d U = USV.matrixU();
		Eigen::Matrix3d Vt = USV.matrixV().transpose();

		// Last column of U is undetermined since d = (a a 0).
		if (U.determinant() < 0) {
			U.col(2) *= -1;
		}
		// Last row of Vt is undetermined since d = (a a 0).
		if (Vt.determinant() < 0) {
			Vt.row(2) *= -1;
		}

		Eigen::Matrix3d W;
		W << 0, -1, 0,
			1, 0, 0,
			0, 0, 1;

		const Eigen::Matrix3d U_W_Vt = U * W * Vt;
		const Eigen::Matrix3d U_Wt_Vt = U * W.transpose() * Vt;

		const std::array<Eigen::Matrix3d, 2> R{ {U_W_Vt, U_Wt_Vt} };
		const std::array<Eigen::Vector3d, 2> t{ {U.col(2), -U.col(2)} };
		if (relative_poses)
		{
			relative_poses->clear();
			relative_poses->reserve(4);
			relative_poses->emplace_back(R[0], -R[0].transpose() * t[0]);
			relative_poses->emplace_back(R[1], -R[1].transpose() * t[1]);
			relative_poses->emplace_back(R[0], -R[0].transpose() * t[1]);
			relative_poses->emplace_back(R[1], -R[1].transpose() * t[0]);
		}
	}

	// HZ 12.2 p.312
	void TriangulateDLT
	(
		const Eigen::Matrix<double, 3, 4>& P1,
		const Eigen::Vector3d& x1,
		const Eigen::Matrix<double, 3, 4>& P2,
		const Eigen::Vector3d& x2,
		Eigen::Vector4d* X_homogeneous
	)
	{
		// Solve:
		// [cross(x0,P0) X = 0]
		// [cross(x1,P1) X = 0]
		Eigen::Matrix4d design;
		design.row(0) = x1[0] * P1.row(2) - x1[2] * P1.row(0);
		design.row(1) = x1[1] * P1.row(2) - x1[2] * P1.row(1);
		design.row(2) = x2[0] * P2.row(2) - x2[2] * P2.row(0);
		design.row(3) = x2[1] * P2.row(2) - x2[2] * P2.row(1);

		const Eigen::JacobiSVD<Eigen::Matrix4d> svd(design, Eigen::ComputeFullV);
		(*X_homogeneous) = svd.matrixV().col(3);
	}

	void TriangulateDLT
	(
		const Eigen::Matrix<double, 3, 4>& P1,
		const Eigen::Vector3d& x1,
		const Eigen::Matrix<double, 3, 4>& P2,
		const Eigen::Vector3d& x2,
		Eigen::Vector3d* X_euclidean
	)
	{
		Eigen::Vector4d X_homogeneous;
		TriangulateDLT(P1, x1, P2, x2, &X_homogeneous);
		(*X_euclidean) = X_homogeneous.hnormalized();
	}

	bool TriangulateDLT
	(
		const Eigen::Matrix3d& R0,
		const Eigen::Vector3d& t0,
		const Eigen::Vector3d& x0,
		const Eigen::Matrix3d& R1,
		const Eigen::Vector3d& t1,
		const Eigen::Vector3d& x1,
		Eigen::Vector3d* X
	)
	{
		Eigen::Matrix<double, 3, 4> P0, P1;
		P0.block<3, 3>(0, 0) = R0;
		P1.block<3, 3>(0, 0) = R1;
		P0.block<3, 1>(0, 3) = t0;
		P1.block<3, 1>(0, 3) = t1;
		TriangulateDLT(P0, x0, P1, x1, X);
		return x0.dot(R0 * (*X + R0.transpose() * t0)) > 0.0 &&
			x1.dot(R1 * (*X + R1.transpose() * t1)) > 0.0;
	}
	inline void AbsoluteToRelative(
		const Eigen::Matrix3d& R0,
		const Eigen::Vector3d& t0,
		const Eigen::Matrix3d& R1,
		const Eigen::Vector3d& t1,
		const Eigen::Vector3d& x0,
		Eigen::Matrix3d& R,
		Eigen::Vector3d& t,
		Eigen::Vector3d& Rx0
	)
	{
		R = R1 * R0.transpose();
		t = t1 - R * t0;
		Rx0 = R * x0;
	}
	bool Compute3DPoint(
		const Eigen::Vector3d& mprime0,
		const Eigen::Vector3d& mprime1,
		const Eigen::Vector3d& t,
		const Eigen::Matrix3d& R1,
		const Eigen::Vector3d& t1,
		Eigen::Vector3d* X)
	{
		const Eigen::Vector3d z = mprime1.cross(mprime0);
		const double z_squared = z.squaredNorm();
		const double lambda0 = z.dot(t.cross(mprime1)) / z_squared;
		const double lambda1 = z.dot(t.cross(mprime0)) / z_squared;
		const Eigen::Vector3d xprime1 = t + lambda0 * mprime0;

		// x'1 is into the frame of camera1 convert it into the world frame in order to obtain the 3D point
		*X = R1.transpose() * (xprime1 - t1);

		// make and return the result of the cheirality test
		return lambda0 > 0.0 && lambda1 > 0.0;
	}
	bool TriangulateL1Angular
	(
		const Eigen::Matrix3d& R0,
		const Eigen::Vector3d& t0,
		const Eigen::Vector3d& x0,
		const Eigen::Matrix3d& R1,
		const Eigen::Vector3d& t1,
		const Eigen::Vector3d& x1,
		Eigen::Vector3d* X_euclidean
	)
	{
		// Table 1 - 1) we compute m0 (Rx0) and m1 (x1)
		Eigen::Matrix3d R;
		Eigen::Vector3d t, Rx0;
		AbsoluteToRelative(R0, t0, R1, t1, x0, R, t, Rx0);

		// Table 1 - 2) obtain m'0 and m'1
		if (Rx0.normalized().cross(t).squaredNorm() <= x1.normalized().cross(t).squaredNorm())
		{
			const Eigen::Vector3d n1 = x1.cross(t).normalized();
			// Eq. (12)
			const Eigen::Vector3d mprime0 = Rx0 - Rx0.dot(n1) * n1;
			return Compute3DPoint(mprime0, x1, t, R1, t1, X_euclidean);
		}
		else
		{
			const Eigen::Vector3d n0 = Rx0.cross(t).normalized();
			// Eq. (13)
			const Eigen::Vector3d mprime1 = x1 - x1.dot(n0) * n0;
			return Compute3DPoint(Rx0, mprime1, t, R1, t1, X_euclidean);
		}
	}
	bool TriangulateLInfinityAngular
	(
		const Eigen::Matrix3d& R0,
		const Eigen::Vector3d& t0,
		const Eigen::Vector3d& x0,
		const Eigen::Matrix3d& R1,
		const Eigen::Vector3d& t1,
		const Eigen::Vector3d& x1,
		Eigen::Vector3d* X_euclidean
	)
	{
		// Table 1 - 1) we compute m0 (Rx0) and m1 (x1)
		Eigen::Matrix3d R;
		Eigen::Vector3d t, Rx0;
		AbsoluteToRelative(R0, t0, R1, t1, x0, R, t, Rx0);

		// cf. 7. Lemma 2
		const Eigen::Vector3d Rx0_norm = Rx0.normalized();
		const Eigen::Vector3d x1_norm = x1.normalized();
		const Eigen::Vector3d na = (Rx0_norm + x1_norm).cross(t);
		const Eigen::Vector3d nb = (Rx0_norm - x1_norm).cross(t);

		const Eigen::Vector3d nprime = na.squaredNorm() >= nb.squaredNorm() ? na.normalized() : nb.normalized();

		const Eigen::Vector3d mprime0 = Rx0 - (Rx0.dot(nprime)) * nprime;
		const Eigen::Vector3d mprime1 = x1 - (x1.dot(nprime)) * nprime;

		return Compute3DPoint(mprime0, mprime1, t, R1, t1, X_euclidean);
	}
	bool TriangulateIDWMidpoint(
		const Eigen::Matrix3d& R0,
		const Eigen::Vector3d& t0,
		const Eigen::Vector3d& x0,
		const Eigen::Matrix3d& R1,
		const Eigen::Vector3d& t1,
		const Eigen::Vector3d& x1,
		Eigen::Vector3d* X_euclidean
	)
	{
		// absolute to relative
		Eigen::Matrix3d R;
		Eigen::Vector3d t, Rx0;
		AbsoluteToRelative(R0, t0, R1, t1, x0, R, t, Rx0);

		const double p_norm = Rx0.cross(x1).norm();
		const double q_norm = Rx0.cross(t).norm();
		const double r_norm = x1.cross(t).norm();

		// Eq. (10)
		const auto xprime1 = (q_norm / (q_norm + r_norm))
			* (t + (r_norm / p_norm) * (Rx0 + x1));

		// relative to absolute
		*X_euclidean = R1.transpose() * (xprime1 - t1);

		// Eq. (7)
		const Eigen::Vector3d lambda0_Rx0 = (r_norm / p_norm) * Rx0;
		const Eigen::Vector3d lambda1_x1 = (q_norm / p_norm) * x1;

		// Eq. (9) - test adequation
		return (t + lambda0_Rx0 - lambda1_x1).squaredNorm()
			<
			std::min(std::min(
				(t + lambda0_Rx0 + lambda1_x1).squaredNorm(),
				(t - lambda0_Rx0 - lambda1_x1).squaredNorm()),
				(t - lambda0_Rx0 + lambda1_x1).squaredNorm());
	}
	bool Triangulate2View
	(
		const Eigen::Matrix3d& R0,
		const Eigen::Vector3d& t0,
		const Eigen::Vector3d& bearing0,
		const Eigen::Matrix3d& R1,
		const Eigen::Vector3d& t1,
		const Eigen::Vector3d& bearing1,
		Eigen::Vector3d& X,
		ETriangulationMethod etri_method
	)
	{
		switch (etri_method)
		{
		case ETriangulationMethod::DIRECT_LINEAR_TRANSFORM:
			return TriangulateDLT(R0, t0, bearing0, R1, t1, bearing1, &X);
			break;
		case ETriangulationMethod::L1_ANGULAR:
			return TriangulateL1Angular(R0, t0, bearing0, R1, t1, bearing1, &X);
			break;
		case ETriangulationMethod::LINFINITY_ANGULAR:
			return TriangulateLInfinityAngular(R0, t0, bearing0, R1, t1, bearing1, &X);
			break;
		case ETriangulationMethod::INVERSE_DEPTH_WEIGHTED_MIDPOINT:
			return TriangulateIDWMidpoint(R0, t0, bearing0, R1, t1, bearing1, &X);
			break;
		default:
			return false;
		}
		return false;
	}

	bool RelativePoseFromEssential
	(
		const Eigen::Matrix<double, 3, Eigen::Dynamic>& x1,
		const Eigen::Matrix<double, 3, Eigen::Dynamic>& x2,
		const Eigen::Matrix3d& E,
		const std::vector<uint32_t>& bearing_vector_index_to_use,
		Pose3* relative_pose,
		std::vector<uint32_t>* vec_selected_points,
		std::vector<Eigen::Vector3d>* vec_points,
		const ETriangulationMethod triangulation_method
	)
	{
		// Recover plausible relative poses from E.
		std::vector<Pose3> relative_poses;
		MotionFromEssential(E, &relative_poses);

		// Accumulator to find the best solution
		std::vector<uint32_t> cheirality_accumulator(relative_poses.size(), 0);

		// Find which solution is the best:
		// - count how many triangulated observations are in front of the cameras
		std::vector<std::vector<uint32_t>> vec_newInliers(relative_poses.size());
		std::vector<std::vector<Eigen::Vector3d>> vec_3D(relative_poses.size());

		const Pose3 pose1(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());

		for (size_t i = 0; i < relative_poses.size(); ++i)
		{
			const Pose3& pose2 = relative_poses[i];
			Eigen::Vector3d X;
			for (const uint32_t inlier_idx : bearing_vector_index_to_use)
			{
				const auto
					f1 = x1.col(inlier_idx),
					f2 = x2.col(inlier_idx);
				if (Triangulate2View
				(
					pose1.rotation(), pose1.translation(), f1,
					pose2.rotation(), pose2.translation(), f2,
					X,
					triangulation_method
				))
				{
					++cheirality_accumulator[i];
					vec_newInliers[i].push_back(inlier_idx);
					vec_3D[i].push_back(X);
				}
			}
		}

		// Check if there is a valid solution:
		const auto iter = std::max_element(cheirality_accumulator.cbegin(), cheirality_accumulator.cend());
		if (*iter == 0)
		{
			// There is no right solution with points in front of the cameras
			return false;
		}

		// Export the best solution data
		const size_t index = std::distance(cheirality_accumulator.cbegin(), iter);
		if (relative_pose)
		{
			(*relative_pose) = relative_poses[index];
		}
		if (vec_selected_points)
			(*vec_selected_points) = vec_newInliers[index];
		if (vec_points)
			(*vec_points) = vec_3D[index];

		// Test if the best solution is good by using the ratio of the two best solution score
		std::sort(cheirality_accumulator.begin(), cheirality_accumulator.end());
		const double ratio = cheirality_accumulator.rbegin()[1]
			/ static_cast<double>(cheirality_accumulator.rbegin()[0]);
		return true;
	}
}
