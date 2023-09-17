
//D:\repo\openMVG\src\third_party\lemon; D:\repo\openMVG\src\build - 2019\third_party\lemon; D:\ucl360\libraries2019\Eigen\eigen3; D:\repo\openMVG\src; D:\repo\openMVG\src\dependencies\cereal\include; D:\repo\openMVG\src\third_party\stlplus3\filesystemSimplified; D:\repo\openMVG\src\build - 2019\third_party\ceres - solver\config; D:\repo\openMVG\src\third_party\ceres - solver\include; D:\repo\openMVG\src\third_party\ceres - solver\internal\ceres\miniglog
//NOMINMAX; WIN32; _WINDOWS; _USE_MATH_DEFINES; OPENMVG_USE_AVX2; OPENMVG_USE_AVX; __SSE2__; __SSE3__; __SSSE3__; __SSE4_1__; __SSE4_2__; __AVX__; __FMA__; __BMI2__; __AVX2__; OPENMVG_USE_OPENMP; EIGEN_MPL2_ONLY; USE_PATENTED_LIGT; LEMON_ONLY_TEMPLATES

//D:\repo\openMVG\src\build-2019\Windows-AMD64-Release\Debug\openMVG_features.lib;D:\repo\openMVG\src\build-2019\Windows-AMD64-Release\Debug\openMVG_matching_image_collection.lib;D:\repo\openMVG\src\build-2019\Windows-AMD64-Release\Debug\openMVG_multiview.lib;D:\repo\openMVG\src\build-2019\Windows-AMD64-Release\Debug\openMVG_sfm.lib;D:\repo\openMVG\src\build-2019\Windows-AMD64-Release\Debug\openMVG_system.lib;D:\repo\openMVG\src\build-2019\Windows-AMD64-Release\Debug\openMVG_stlplus.lib;D:\repo\openMVG\src\build-2019\Windows-AMD64-Release\Debug\openMVG_matching.lib;D:\repo\openMVG\src\build-2019\Windows-AMD64-Release\Debug\openMVG_fast.lib;D:\repo\openMVG\src\build-2019\Windows-AMD64-Release\Debug\openMVG_geometry.lib;D:\repo\openMVG\src\build-2019\Windows-AMD64-Release\Debug\openMVG_image.lib;D:\repo\openMVG\src\build-2019\Windows-AMD64-Release\Debug\vlsift.lib;D:\repo\openMVG\src\build-2019\Windows-AMD64-Release\Debug\openMVG_jpeg.lib;D:\repo\openMVG\src\build-2019\Windows-AMD64-Release\Debug\openMVG_png.lib;D:\repo\openMVG\src\build-2019\Windows-AMD64-Release\Debug\openMVG_tiff.lib;D:\repo\openMVG\src\build-2019\Windows-AMD64-Release\Debug\openMVG_zlib.lib;D:\repo\openMVG\src\build-2019\Windows-AMD64-Release\Debug\openMVG_lInftyComputerVision.lib;D:\repo\openMVG\src\build-2019\third_party\ceres-solver\lib\Debug\openMVG_ceres-debug.lib;D:\repo\openMVG\src\build-2019\Windows-AMD64-Release\Debug\openMVG_linearProgramming.lib;D:\repo\openMVG\src\build-2019\Windows-AMD64-Release\Debug\openMVG_numeric.lib;D:\repo\openMVG\src\build-2019\Windows-AMD64-Release\Debug\lib_clp.lib;D:\repo\openMVG\src\build-2019\Windows-AMD64-Release\Debug\lib_OsiClpSolver.lib;D:\repo\openMVG\src\build-2019\Windows-AMD64-Release\Debug\lib_CoinUtils.lib;D:\repo\openMVG\src\build-2019\Windows-AMD64-Release\Debug\lib_Osi.lib;kernel32.lib;user32.lib;gdi32.lib;winspool.lib;shell32.lib;ole32.lib;oleaut32.lib;uuid.lib;comdlg32.lib;advapi32.lib



//NOMINMAX;WIN32;_WINDOWS;_USE_MATH_DEFINES;OPENMVG_USE_AVX2;OPENMVG_USE_AVX;__SSE2__;__SSE3__;__SSSE3__;__SSE4_1__;__SSE4_2__;__AVX__;__FMA__;__BMI2__;__AVX2__;OPENMVG_USE_OPENMP;EIGEN_MPL2_ONLY;USE_PATENTED_LIGT;LEMON_ONLY_TEMPLATES


#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/sfm/sfm_report.hpp"
#include "openMVG/system/timer.hpp"
#include "openMVG/system/logger.hpp" 

#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"
#include "openMVG/stl/stl.hpp"
#include <sstream>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <locale>
#include <memory>
#include <map>
#include <array>
#include <string>
#include "omp.h"
#include "cereal/cereal.hpp"
#include "cereal/archives/binary.hpp"
#include "cereal/archives/json.hpp"
#include "cereal/types/string.hpp"
#include "cereal/types/utility.hpp"
#include "cereal/types/memory.hpp"
#include "cereal/types/complex.hpp"
#include "cereal/types/base_class.hpp"
#include "cereal/types/array.hpp"
#include "cereal/types/vector.hpp"
#include "cereal/types/map.hpp"
#include "cereal/types/tuple.hpp"

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
struct Pt3d_MeshId
{
	std::map<int, int>pt3d_MeshId;
	template <class Archive>
	void serialize(Archive& ar)
	{ 
		ar(cereal::make_nvp("pt3d_MeshId", pt3d_MeshId));
	}
};
struct LandmarkViz
{
	std::vector<std::string >path;
	std::vector<std::vector<int>>featId_landmarkId;
	template <class Archive>
	void serialize(Archive& ar)
	{
		ar(cereal::make_nvp("path", path));
		ar(cereal::make_nvp("featId_landmarkId", featId_landmarkId));
	}
};
int main(int argc, char** argv)
{
	if (argc != 2)
	{
		std::cout << "cmd sfmJsonPath" << std::endl;
		return -1;
	}
	else
	{
		std::cout << " You called:" << std::endl;
		std::cout << "  ---" << argv[0] << std::endl;
		std::cout << "  ---" << argv[1] << std::endl;
	}
	openMVG::sfm::SfM_Data sfm_data;
	std::string sfmJsonPath(argv[1]);
	LandmarkViz landmarkViz;
	const std::string featIdToLandmarkIdDir = stlplus::folder_down(stlplus::folder_part(sfmJsonPath),"matches");// +featIdToLandmarkId.json;
	const std::string featIdToLandmarkIdJson = stlplus::create_filespec(featIdToLandmarkIdDir, "featIdToLandmarkId", "json");
	if (!Load(sfm_data, sfmJsonPath, openMVG::sfm::ESfM_Data(openMVG::sfm::ALL))) {
		OPENMVG_LOG_ERROR
			<< "The input file \"" << sfmJsonPath << "\" cannot be read";
		return EXIT_FAILURE;
	}
	try
	{
		std::fstream fin(featIdToLandmarkIdJson, std::ios::in);
		std::stringstream ss;
		//ss << "{\"data\":";
		std::string aline;
		while (std::getline(fin, aline))
		{
			ss << aline;
		}
		//ss << "}";
		{
			cereal::JSONInputArchive archive(ss);
			archive(cereal::make_nvp("path", landmarkViz.path));
			archive(cereal::make_nvp("featId_landmarkId", landmarkViz.featId_landmarkId));
		}
		if (landmarkViz.path.size()!= landmarkViz.featId_landmarkId.size())
		{
			throw std::exception();
		}
	}
	catch (const std::exception&)
	{ 
		return EXIT_FAILURE;
	}
	Pt3d_MeshId pt3dMeshId;
	std::map<int, int>featId_landmarkId;
	for (const auto&d: sfm_data.structure)
	{
		const openMVG::sfm::Landmark& lk = d.second;
		const openMVG::sfm::Observations& obs = lk.obs;
		std::map<int, int>feat_id_cnt;
		int landmarkId = -1;
		int id_feat_ = -1;
		for (const auto&d2: obs)
		{
			const int& viewId = d2.first;
			const openMVG::IndexT& id_feat = d2.second.id_feat;
			const openMVG::IndexT& id_landmark = landmarkViz.featId_landmarkId[viewId][id_feat];
			if (feat_id_cnt.count(id_landmark)==0)
			{
				feat_id_cnt[id_landmark] = 1;
				landmarkId = id_landmark;
			}
			else
			{
				feat_id_cnt[id_landmark] += 1;
			}
		}
		if (feat_id_cnt.size()!=1)
		{
			for (const auto& d2 : feat_id_cnt)
			{
				std::cout << "  [" << d2.first << "] " << d2.second << std::endl;
			}
			return EXIT_FAILURE;
		} 
		if (landmarkId<0)
		{
			return EXIT_FAILURE;
		}
		pt3dMeshId.pt3d_MeshId[static_cast<int>(d.first)] = landmarkId;
		//featId_landmarkId[static_cast<int>(id_feat)] = landmarkId;
	}
	try
	{
		const std::string pt3dMeshIdJson = stlplus::create_filespec(stlplus::folder_part(sfmJsonPath), "pt3dMeshId", "json");
		std::stringstream ss;
		{
			cereal::JSONOutputArchive archive(ss); 
			archive(cereal::make_nvp("pt3dMeshId", pt3dMeshId.pt3d_MeshId));
		} 
		std::fstream fout(pt3dMeshIdJson, std::ios::out);
		fout << ss.str() << std::endl;
		fout.close();
	}
	catch (const std::exception&)
	{
		return EXIT_FAILURE;
	}
	return 0;
}