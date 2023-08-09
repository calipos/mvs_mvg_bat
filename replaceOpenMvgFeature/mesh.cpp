#include "mesh.h"
#include <iostream>
#include<fstream>
#include <string>
#include <list>
#include <map>
#include <vector>
namespace mesh
{
	struct Edge
	{
		int a;
		int b; 
		Edge() {}
		Edge(const int& a_, const int& b_)
		{
			if (a_<b_)
			{
				a = a_;
				b = b_;
			}
			else
			{
				b = a_;
				a = b_;
			}
		}
		bool operator<(const Edge& other) const //注意这里的两个const
		{ 
			return (a < other.a) || (a == other.a&& b < other.b);
		}
	};
	int reorderFaces(Eigen::MatrixXi& faces, const int& baseFaceId)
	{
		if (baseFaceId >= faces.rows())return -1;
		if (baseFaceId <0)return -1;
		int max = faces.maxCoeff();
		std::map<Edge, std::list<int>>egde2face;
		std::map<int, std::list<int>>face2neighborFace;
		for (int i = 0; i < faces.rows(); i++)
		{
			Edge e1(faces(i, 0), faces(i, 1));
			Edge e2(faces(i, 2), faces(i, 1));
			Edge e3(faces(i, 0), faces(i, 2));			
			egde2face[e1].emplace_back(i); ;			
			egde2face[e2].emplace_back(i);;			
			egde2face[e3].emplace_back(i);;
			
		}
		for (auto&d: egde2face)
		{
			if (d.second.size()>2)
			{
				std::cout<<"manifold error" << std::endl;
				return -1;
			}
			if (d.second.size()==2)
			{
				int a = *d.second.begin();
				int b = *(++d.second.begin());
				if (face2neighborFace[a].end() == std::find(face2neighborFace[a].begin(), face2neighborFace[a].end(), b))
				{
					face2neighborFace[a].emplace_back(b);
				}
				if (face2neighborFace[b].end() == std::find(face2neighborFace[b].begin(), face2neighborFace[b].end(), a))
				{
					face2neighborFace[b].emplace_back(a);
				}
			}
		}
		std::map<int, int>reodered;
		std::list<int>reorderSet{ baseFaceId };
		while (reorderSet.size())
		{
			std::list<int>newReorderSet;
			for (auto&f: reorderSet)
			{
				const std::list<int>neighbors = face2neighborFace[f];
				std::string edge1 = std::to_string(faces(f, 0)) + std::to_string(faces(f, 1));
				std::string edge2 = std::to_string(faces(f, 1)) + std::to_string(faces(f, 2));
				std::string edge3 = std::to_string(faces(f, 2)) + std::to_string(faces(f, 0));
				for (auto&n:neighbors)
				{
					if (reodered.count(n)==0)
					{
						reodered[n] = 1;
						newReorderSet.emplace_back(n);
					}
					else
					{
						continue;
					}
					std::string edge1_n= std::to_string(faces(n, 0)) + std::to_string(faces(n, 1));
					std::string edge2_n= std::to_string(faces(n, 1)) + std::to_string(faces(n, 2));
					std::string edge3_n= std::to_string(faces(n, 2)) + std::to_string(faces(n, 0));
					if (edge1.compare(edge1_n) == 0)
					{
						std::swap(faces(n, 0), faces(n, 1));
						continue;
					}
					if (edge1.compare(edge2_n) == 0)
					{
						std::swap(faces(n, 0), faces(n, 1));
						continue;
					}
					if (edge1.compare(edge3_n) == 0)
					{
						std::swap(faces(n, 0), faces(n, 1));
						continue;
					}
					if (edge2.compare(edge1_n) == 0)
					{
						std::swap(faces(n, 0), faces(n, 1));
						continue;
					}
					if (edge2.compare(edge2_n) == 0)
					{
						std::swap(faces(n, 0), faces(n, 1));
						continue;
					}
					if (edge2.compare(edge3_n) == 0)
					{
						std::swap(faces(n, 0), faces(n, 1));
						continue;
					}
					if (edge3.compare(edge1_n) == 0)
					{
						std::swap(faces(n, 0), faces(n, 1));
						continue;
					}
					if (edge3.compare(edge2_n) == 0)
					{
						std::swap(faces(n, 0), faces(n, 1));
						continue;
					}
					if (edge3.compare(edge3_n) == 0)
					{
						std::swap(faces(n, 0), faces(n, 1));
						continue;
					}
				}
			}
			reorderSet = newReorderSet;
		}
		return 0;
	}
	int reorderFaces(Eigen::MatrixXi&faces)
	{
		return reorderFaces(faces,0);
	}
	int inertFaces(Eigen::MatrixXi& faces)
	{
		for (int i = 0; i < faces.rows(); i++)
		{
			std::swap(faces(i, 0), faces(i, 1));
		}
		return 0;
	}
	void save(const std::string& path, const Eigen::MatrixXf& pts, const Eigen::MatrixXi& faces)
	{
		std::fstream fout(path,std::ios::out);
		for (int i = 0; i < pts.rows(); i++)
		{
			fout << "v " << pts(i, 0) << " " << pts(i, 1) << " " << pts(i, 2) << std::endl;
		}
		for (int i = 0; i < faces.rows(); i++)
		{
			fout << "f " << faces(i, 0) + 1 << " " << faces(i, 1) + 1 << " " << faces(i, 2) + 1 << std::endl;
		}
		fout.close();
	}

}