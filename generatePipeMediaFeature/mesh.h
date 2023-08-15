#ifndef _MESH_H_
#define _MESH_H_

#include "Eigen/Eigen"
#define IGL_RAY_TRI_EPSILON 0.000000001
#define IGL_RAY_TRI_CROSS(dest,v1,v2) \
          dest[0]=v1[1]*v2[2]-v1[2]*v2[1]; \
          dest[1]=v1[2]*v2[0]-v1[0]*v2[2]; \
          dest[2]=v1[0]*v2[1]-v1[1]*v2[0];
#define IGL_RAY_TRI_DOT(v1,v2) (v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2])
#define IGL_RAY_TRI_SUB(dest,v1,v2) \
          dest[0]=v1[0]-v2[0]; \
          dest[1]=v1[1]-v2[1]; \
          dest[2]=v1[2]-v2[2]; 
template<class Dtype = float>
int intersect_triangle1(const Dtype* orig, const Dtype* dir,
	const Dtype* vert0, const Dtype* vert1, const Dtype* vert2,
	Dtype* t, Dtype* u, Dtype* v)
{
	Dtype edge1[3], edge2[3], tvec[3], pvec[3], qvec[3];
	Dtype det, inv_det;

	/* find vectors for two edges sharing vert0 */
	IGL_RAY_TRI_SUB(edge1, vert1, vert0);
	IGL_RAY_TRI_SUB(edge2, vert2, vert0);

	/* begin calculating determinant - also used to calculate U parameter */
	IGL_RAY_TRI_CROSS(pvec, dir, edge2);

	/* if determinant is near zero, ray lies in plane of triangle */
	det = IGL_RAY_TRI_DOT(edge1, pvec);

	if (det > IGL_RAY_TRI_EPSILON)
	{
		/* calculate distance from vert0 to ray origin */
		IGL_RAY_TRI_SUB(tvec, orig, vert0);

		/* calculate U parameter and test bounds */
		*u = IGL_RAY_TRI_DOT(tvec, pvec);
		if (*u < 0.0 || *u > det)
			return 0;

		/* prepare to test V parameter */
		IGL_RAY_TRI_CROSS(qvec, tvec, edge1);

		/* calculate V parameter and test bounds */
		*v = IGL_RAY_TRI_DOT(dir, qvec);
		if (*v < 0.0 || *u + *v > det)
			return 0;

	}
	else if (det < -IGL_RAY_TRI_EPSILON)
	{
		/* calculate distance from vert0 to ray origin */
		IGL_RAY_TRI_SUB(tvec, orig, vert0);

		/* calculate U parameter and test bounds */
		*u = IGL_RAY_TRI_DOT(tvec, pvec);
		/*      printf("*u=%f\n",(float)*u); */
		/*      printf("det=%f\n",det); */
		if (*u > 0.0 || *u < det)
			return 0;

		/* prepare to test V parameter */
		IGL_RAY_TRI_CROSS(qvec, tvec, edge1);

		/* calculate V parameter and test bounds */
		*v = IGL_RAY_TRI_DOT(dir, qvec);
		if (*v > 0.0 || *u + *v < det)
			return 0;
	}
	else return 0;  /* ray is parallel to the plane of the triangle */


	inv_det = 1.0 / det;

	/* calculate t, ray intersects triangle */
	*t = IGL_RAY_TRI_DOT(edge2, qvec) * inv_det;
	(*u) *= inv_det;
	(*v) *= inv_det;

	return 1;
}
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