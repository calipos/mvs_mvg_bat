#ifndef _ESSENTIAL_DECOMPOSE_INTO_RT_H_
#define _ESSENTIAL_DECOMPOSE_INTO_RT_H_
#include "Eigen/Core"
#include <Eigen/Dense>
#include <numeric>
#include <vector>
#include "common.h"
namespace rt
{
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
	);
	bool RelativePoseFromEssential
	(
		const Eigen::Matrix<double, 3, Eigen::Dynamic>& x1,
		const Eigen::Matrix<double, 3, Eigen::Dynamic>& x2,
		const Eigen::Matrix3d& E,
		const std::vector<uint32_t>& bearing_vector_index_to_use,
		Pose3* relative_pose = nullptr,
		std::vector<uint32_t>* vec_selected_points = nullptr,
		std::vector<Eigen::Vector3d>* vec_points = nullptr,
		const ETriangulationMethod triangulation_method = ETriangulationMethod::DEFAULT
	);
}
#endif // !_ESSENTIAL_DECOMPOSE_INTO_RT_H_
