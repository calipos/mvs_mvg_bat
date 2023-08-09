#ifndef _MESH_H_
#define _MESH_H_

#include "Eigen/Eigen"
template<typename Dtype, typename DtypeIn>
inline Eigen::Matrix<Dtype, Eigen::Dynamic, Eigen::Dynamic> vector3ToEigen(const std::vector<std::vector<DtypeIn>>& pts)
{
	Eigen::Matrix<Dtype, Eigen::Dynamic, Eigen::Dynamic> ret;
	ret.resize(pts.size(), pts[0].size());
#pragma omp parallel for
	for (int i = 0; i < pts.size(); i++)
	{
		for (int j = 0; j < pts[i].size(); j++)
		{
			ret(i, j) = pts[i][j];
		}
	}
	return ret;
}
namespace mesh
{
	int reorderFaces(Eigen::MatrixXi& faces, const int& baseFaceId);
	int reorderFaces(Eigen::MatrixXi& faces); 
	int inertFaces(Eigen::MatrixXi& faces);
	void save(const std::string& path, const Eigen::MatrixXf& pts, const Eigen::MatrixXi& faces);
}
#endif