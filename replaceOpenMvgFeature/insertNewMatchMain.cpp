
//D:\repo\openMVG\src\third_party\lemon; D:\repo\openMVG\src\build - 2019\third_party\lemon; D:\ucl360\libraries2019\Eigen\eigen3; D:\repo\openMVG\src; D:\repo\openMVG\src\dependencies\cereal\include; D:\repo\openMVG\src\third_party\stlplus3\filesystemSimplified; D:\repo\openMVG\src\build - 2019\third_party\ceres - solver\config; D:\repo\openMVG\src\third_party\ceres - solver\include; D:\repo\openMVG\src\third_party\ceres - solver\internal\ceres\miniglog
//NOMINMAX; WIN32; _WINDOWS; _USE_MATH_DEFINES; OPENMVG_USE_AVX2; OPENMVG_USE_AVX; __SSE2__; __SSE3__; __SSSE3__; __SSE4_1__; __SSE4_2__; __AVX__; __FMA__; __BMI2__; __AVX2__; OPENMVG_USE_OPENMP; EIGEN_MPL2_ONLY; USE_PATENTED_LIGT; LEMON_ONLY_TEMPLATES

//D:\repo\openMVG\src\build-2019\Windows-AMD64-Release\Debug\openMVG_features.lib;D:\repo\openMVG\src\build-2019\Windows-AMD64-Release\Debug\openMVG_matching_image_collection.lib;D:\repo\openMVG\src\build-2019\Windows-AMD64-Release\Debug\openMVG_multiview.lib;D:\repo\openMVG\src\build-2019\Windows-AMD64-Release\Debug\openMVG_sfm.lib;D:\repo\openMVG\src\build-2019\Windows-AMD64-Release\Debug\openMVG_system.lib;D:\repo\openMVG\src\build-2019\Windows-AMD64-Release\Debug\openMVG_stlplus.lib;D:\repo\openMVG\src\build-2019\Windows-AMD64-Release\Debug\openMVG_matching.lib;D:\repo\openMVG\src\build-2019\Windows-AMD64-Release\Debug\openMVG_fast.lib;D:\repo\openMVG\src\build-2019\Windows-AMD64-Release\Debug\openMVG_geometry.lib;D:\repo\openMVG\src\build-2019\Windows-AMD64-Release\Debug\openMVG_image.lib;D:\repo\openMVG\src\build-2019\Windows-AMD64-Release\Debug\vlsift.lib;D:\repo\openMVG\src\build-2019\Windows-AMD64-Release\Debug\openMVG_jpeg.lib;D:\repo\openMVG\src\build-2019\Windows-AMD64-Release\Debug\openMVG_png.lib;D:\repo\openMVG\src\build-2019\Windows-AMD64-Release\Debug\openMVG_tiff.lib;D:\repo\openMVG\src\build-2019\Windows-AMD64-Release\Debug\openMVG_zlib.lib;D:\repo\openMVG\src\build-2019\Windows-AMD64-Release\Debug\openMVG_lInftyComputerVision.lib;D:\repo\openMVG\src\build-2019\third_party\ceres-solver\lib\Debug\openMVG_ceres-debug.lib;D:\repo\openMVG\src\build-2019\Windows-AMD64-Release\Debug\openMVG_linearProgramming.lib;D:\repo\openMVG\src\build-2019\Windows-AMD64-Release\Debug\openMVG_numeric.lib;D:\repo\openMVG\src\build-2019\Windows-AMD64-Release\Debug\lib_clp.lib;D:\repo\openMVG\src\build-2019\Windows-AMD64-Release\Debug\lib_OsiClpSolver.lib;D:\repo\openMVG\src\build-2019\Windows-AMD64-Release\Debug\lib_CoinUtils.lib;D:\repo\openMVG\src\build-2019\Windows-AMD64-Release\Debug\lib_Osi.lib;kernel32.lib;user32.lib;gdi32.lib;winspool.lib;shell32.lib;ole32.lib;oleaut32.lib;uuid.lib;comdlg32.lib;advapi32.lib

 

//NOMINMAX;WIN32;_WINDOWS;_USE_MATH_DEFINES;OPENMVG_USE_AVX2;OPENMVG_USE_AVX;__SSE2__;__SSE3__;__SSSE3__;__SSE4_1__;__SSE4_2__;__AVX__;__FMA__;__BMI2__;__AVX2__;OPENMVG_USE_OPENMP;EIGEN_MPL2_ONLY;USE_PATENTED_LIGT;LEMON_ONLY_TEMPLATES


#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/sfm/sfm_report.hpp"
#include "openMVG/system/timer.hpp"
#include "openMVG/system/logger.hpp" 

#include "openMVG/graph/graph.hpp"
#include "openMVG/graph/graph_stats.hpp"


#include "openMVG/sfm/pipelines/sfm_features_provider.hpp"
#include "openMVG/sfm/pipelines/sfm_matches_provider.hpp"
#include "openMVG/sfm/pipelines/sfm_regions_provider.hpp"
#include "openMVG/sfm/pipelines/sfm_regions_provider_cache.hpp" 

#include "openMVG/matching_image_collection/E_ACRobust.hpp"
#include "openMVG/matching_image_collection/Pair_Builder.hpp"
#include "openMVG/matching_image_collection/Matcher_Regions.hpp"
#include "openMVG/matching_image_collection/GeometricFilter.hpp"

#include "nonFree/sift/SIFT_describer_io.hpp"

// SfM Engines
#include "openMVG/sfm/pipelines/global/GlobalSfM_rotation_averaging.hpp"
#include "openMVG/sfm/pipelines/global/GlobalSfM_translation_averaging.hpp"
#include "openMVG/sfm/pipelines/global/sfm_global_engine_relative_motions.hpp"
#include "openMVG/sfm/pipelines/sequential/sequential_SfM.hpp"
#include "openMVG/sfm/pipelines/sequential/sequential_SfM2.hpp"
#include "openMVG/sfm/pipelines/sequential/SfmSceneInitializerMaxPair.hpp"
#include "openMVG/sfm/pipelines/sequential/SfmSceneInitializerStellar.hpp"
//#include "openMVG/sfm/pipelines/stellar/sfm_stellar_engine.hpp"
//#include "GlobalSfMReconstructionEngine_RelativeMotions_insert.h"

#include "openMVG/matching/indMatch.hpp"
#include "openMVG/matching/indMatch_utils.hpp"
#include "openMVG/matching/pairwiseAdjacencyDisplay.hpp"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"
#include "openMVG/stl/stl.hpp"
#include "Eigen/Eigen"
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
#include <sstream>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <locale>
#include <memory>
#include <map>
#include <array>
#include <string>
#include "mesh.h"
#include "omp.h"

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
 
openMVG::IndexT getImgIndex(const std::string& imgName, const openMVG::sfm::SfM_Data& sfm_data)
{
	for (const auto& d : sfm_data.views)
	{
		if (d.second->s_Img_path.length() < imgName.length())
		{
			continue;
		}
		std::string _ = d.second->s_Img_path.substr(d.second->s_Img_path.length() - imgName.length());
		if (_.compare(imgName) == 0)
		{
			return d.first;
		}
	}
	return (std::numeric_limits<openMVG::IndexT>::max)();
};
//{pointName:  [ imgName: x y featureIdInThisImg ]  }
std::map<std::string, std::map<std::string, std::pair<std::array<double, 2>, std::string>>	> loadAdditionalPointMap(const std::string& path)
{
	std::map<std::string, std::map<std::string, std::pair<std::array<double, 2>, std::string>>	> pointMap;
	std::fstream fin(path, std::ios::in);
	std::string aline;
	while (std::getline(fin, aline))
	{
		if (aline.length() > 0 && aline[0] == '#')continue;
		std::vector<std::string>segs = splitString(aline, " ", true);
		if (segs.size() % 4 != 1 && segs.size() > 1)
		{
			OPENMVG_LOG_WARNING << " err line: " << aline;
			continue;
		} 
		const std::string& thisPointName = segs[0];
		if (pointMap.count(thisPointName)>0)
		{
			OPENMVG_LOG_WARNING << " err PointName: " << aline;
			continue;
		}
		std::map<std::string, std::pair<std::array<double,2>, std::string>>thisPointMap;
		int imgCnt = segs.size() / 4;
		for (int i = 0; i < imgCnt; i++)
		{
			const std::string& imgName = segs[1 + 4 * i]; 
			thisPointMap[imgName] = std::pair<std::array<double,2>, std::string>();
			std::stringstream ss;
			ss << segs[2 + 4 * i] << " " << segs[3 + 4 * i];
			ss >> thisPointMap[imgName].first[0] >> thisPointMap[imgName].first[1];
			thisPointMap[imgName].second = segs[4 + 4 * i];
		}
		pointMap[thisPointName] = thisPointMap;
	}
	return pointMap;
}
std::map<std::string, openMVG::IndexT>getImgsIndex(const openMVG::sfm::SfM_Data& sfm_data,const std::map<std::string, std::map<std::string, std::pair<std::array<double, 2>, std::string>>	>& addPointMap)
{
	std::map<std::string, openMVG::IndexT>ret;
	for (auto& additionPoint : addPointMap)
	{
		for (auto& relativeImg : additionPoint.second)
		{
			openMVG::IndexT imgId = getImgIndex(relativeImg.first, sfm_data);
			if (imgId > sfm_data.views.size())
			{
				OPENMVG_LOG_WARNING << " err imgName: " << additionPoint.first;
				continue;
			}
			ret[relativeImg.first] = imgId;
		}
	}
	return ret;
}
void saveAdditionalPointMap(const std::string& path, const std::map<std::string, std::map<std::string, std::pair<std::array<double, 2>, std::string>>>& additionalPointMap)
{
	std::fstream fout(path,std::ios::out);
	for (auto& additionPoint : additionalPointMap)
	{
		fout << additionPoint.first << " ";
		for (auto& relativeImg : additionPoint.second)
		{
			fout << relativeImg.first << " " << relativeImg.second.first[0] << " " << relativeImg.second.first[1] << " " << relativeImg.second.second << " ";
		}
		fout << std::endl;
	}
	return;
}

void readAdditionalFeatures(const std::string& additionalMatchPath,const openMVG::sfm::SfM_Data&sfm_data,const std::string&featureRoot)
{
	std::unique_ptr<openMVG::features::SIFT_Regions> regions_ptr(new openMVG::features::SIFT_Regions());
	int descriptorLength = regions_ptr->DescriptorLength();
	std::unique_ptr<openMVG::features::SIFT_Image_describer> image_describer(new openMVG::features::SIFT_Image_describer(openMVG::features::SIFT_Image_describer::Params(), !false));


	auto additionalPointMap = loadAdditionalPointMap(additionalMatchPath);
	std::map<std::string, openMVG::IndexT>imgsIndex = getImgsIndex(sfm_data, additionalPointMap);
	for (auto& additionPoint : additionalPointMap)
	{
		openMVG::features::SIFT_Regions::DescriptorT randDescrip;
		for (int i = 0; i < descriptorLength; i++) randDescrip[i] = rand() % 256;		
		for (auto&relativeImg : additionPoint.second)
		{
			openMVG::IndexT imgId = imgsIndex[relativeImg.first];
			std::string thisFeaturePath = featureRoot + "/" + relativeImg.first;
			std::string extension = stlplus::extension_part(thisFeaturePath);
			std::string stem = thisFeaturePath.substr(0, thisFeaturePath.length() - extension.length() - 1);
			thisFeaturePath = stem + ".feat";
			std::string thisDescripPath = stem + ".desc";
			image_describer->Load(regions_ptr.get(), thisFeaturePath, thisDescripPath);
			openMVG::features::SIFT_Regions::FeatureT thisFeat;
			thisFeat.x() = relativeImg.second.first[0];
			thisFeat.y() = relativeImg.second.first[1];
			std::cout << regions_ptr.get()->Features().size() << " " << regions_ptr.get()->Descriptors().size() << std::endl;
			relativeImg.second.second = std::to_string(regions_ptr.get()->Features().size());
			regions_ptr.get()->Features().emplace_back(thisFeat);
			regions_ptr.get()->Descriptors().emplace_back(randDescrip);
			std::cout << regions_ptr.get()->Features().size() << " " << regions_ptr.get()->Descriptors().size() << std::endl;
			image_describer->Save(regions_ptr.get(), thisFeaturePath, thisDescripPath);
		}
	}
	saveAdditionalPointMap(additionalMatchPath, additionalPointMap);
	return;
}

openMVG::IndexT string2ID(const std::string& s)
{
	openMVG::IndexT ret;
	std::stringstream ss;
	ss << s;
	ss >> ret;
	std::string s2 = std::to_string(ret);
	if (s2.compare(s) != 0)
	{
		OPENMVG_LOG_ERROR << "err idx: " << s;
	}
	return ret;
}
openMVG::matching::PairWiseMatches generExclusionPairs(
	const std::map<std::string, std::map<std::string, std::pair<std::array<double, 2>, std::string>>>& additionalPtsMap,
	const std::map<std::string, openMVG::IndexT>& idxMap
)
{
	openMVG::matching::PairWiseMatches ret;
	for (const auto& additionPoint : additionalPtsMap)
	{
		std::vector<openMVG::IndexT>imgIds;
		std::vector<openMVG::IndexT>featureIds;
		for (const auto& relativeImg : additionPoint.second)
		{
			openMVG::IndexT imgId = idxMap.at(relativeImg.first);
			imgIds.emplace_back(imgId);
			openMVG::IndexT featureId = string2ID(relativeImg.second.second);
			featureIds.emplace_back(featureId);
		}
		if (imgIds.size() < 2)
		{
			OPENMVG_LOG_ERROR << "err imgIds.size()<2 ";
		}
		for (int i = 0; i < imgIds.size(); i++)
		{
			for (int j = i + 1; j < imgIds.size(); j++)
			{
				openMVG::IndexT imgA = imgIds[i];
				openMVG::IndexT imgB = imgIds[j];
				openMVG::IndexT featureA = featureIds[i];
				openMVG::IndexT featureB = featureIds[j];
				openMVG::Pair theImgPair(imgA, imgB);
				openMVG::matching::IndMatch featureMatch(featureA, featureB);
				if (imgA > imgB)
				{
					theImgPair = openMVG::Pair(imgB, imgA);
					featureMatch = openMVG::matching::IndMatch(featureB, featureA);
				}
				if (ret.count(theImgPair) == 0)
				{
					ret[theImgPair] = openMVG::matching::IndMatches();
				}
				ret.at(theImgPair).emplace_back(featureMatch);
			}
		}
	}
	return ret;
}

enum class ESfMSceneInitializer
{
	INITIALIZE_EXISTING_POSES,
	INITIALIZE_MAX_PAIR,
	INITIALIZE_AUTO_PAIR,
	INITIALIZE_STELLAR
};
enum class ESfMEngine
{
	INCREMENTAL,
	INCREMENTALV2,
	GLOBAL,
	STELLAR,
	GLOBAL_INSERT
};
bool StringToEnum(	const std::string& str,	ESfMSceneInitializer& scene_initializer)
{
	const std::map<std::string, ESfMSceneInitializer> string_to_enum_mapping =
	{
	  {"EXISTING_POSE", ESfMSceneInitializer::INITIALIZE_EXISTING_POSES},
	  {"MAX_PAIR", ESfMSceneInitializer::INITIALIZE_MAX_PAIR},
	  {"AUTO_PAIR", ESfMSceneInitializer::INITIALIZE_AUTO_PAIR},
	  {"STELLAR", ESfMSceneInitializer::INITIALIZE_STELLAR},
	};
	const auto it = string_to_enum_mapping.find(str);
	if (it == string_to_enum_mapping.end())
		return false;
	scene_initializer = it->second;
	return true;
}
struct Landmarks
{
	int imgHeight;
	int imgWidth;
	std::vector<std::vector<double>>frontLandmarks3d;
	std::vector<std::vector<int>>faces;
	template <class Archive>
	void serialize(Archive& ar)
	{
		ar(cereal::make_nvp("imgHeight", imgHeight));
		ar(cereal::make_nvp("imgWidth", imgWidth));
		ar(cereal::make_nvp("frontLandmarks3d", frontLandmarks3d));
		ar(cereal::make_nvp("faces", faces));
	}
}; 

static std::vector<int> randData;

openMVG::features::SIFT_Regions::DescriptorT getRandDescripBaesOnIdx(const int& descriptorLength, const int&  i)
{	 
	while (randData.size() < i + descriptorLength)
	{
		randData.emplace_back(rand() % 256);
	}
	openMVG::features::SIFT_Regions::DescriptorT randDescrip;
	for (int ii = 0; ii < descriptorLength; ii++) randDescrip[ii] = randData[i + ii];
	return randDescrip;
}




void reOrientFaces(Eigen::MatrixXf& pts, Eigen::MatrixXi& faces )
{}
int replaceFeature(const std::string& landmarksRoot, const std::string& sfmJsonPath)
{ 
	openMVG::sfm::SfM_Data sfm_data;
	if (!openMVG::sfm::Load(sfm_data, sfmJsonPath, openMVG::sfm::ESfM_Data(openMVG::sfm::ALL)))
	{
		OPENMVG_LOG_ERROR << "The input SfM_Data file \"" << sfmJsonPath << "\" cannot be read.";
		return EXIT_FAILURE;
	}
	const std::string featureRoot = stlplus::folder_part(sfmJsonPath);
	const std::string sImage_describer = stlplus::create_filespec(featureRoot, "image_describer", "json");
	std::unique_ptr<openMVG::features::Regions> regions_type = openMVG::features::Init_region_type_from_file(sImage_describer);
	if (!regions_type)
	{
		OPENMVG_LOG_ERROR << "Invalid: " << sImage_describer << " regions type file.";
		return EXIT_FAILURE;
	}


	std::unique_ptr<openMVG::features::SIFT_Regions> regions_ptr(new openMVG::features::SIFT_Regions());
	int descriptorLength = regions_ptr->DescriptorLength();
	std::unique_ptr<openMVG::features::SIFT_Image_describer> image_describer(new openMVG::features::SIFT_Image_describer(openMVG::features::SIFT_Image_describer::Params(), !false));


	//std::unique_ptr<openMVG::features::Regions> regions_type = openMVG::features::Init_region_type_from_file(sImage_describer);
	//if (!regions_type)
	//{
	//	OPENMVG_LOG_ERROR << "Invalid: " << sImage_describer << " regions type file.";
	//	return EXIT_FAILURE;
	//}

	int landmarkCnt = -1;
	
	
	auto iter = sfm_data.views.begin();
	while (  iter!= sfm_data.views.end())
	{   
		const std::string imgName= iter->second->s_Img_path;
		std::string jsonPath = stlplus::create_filespec(landmarksRoot, imgName, "json");   
		Landmarks data;
		try
		{
			std::fstream fin(jsonPath, std::ios::in);
			std::stringstream ss;
			ss << "{\"data\":";
			std::string aline;
			while (std::getline(fin, aline))
			{
				ss << aline;
			}
			ss << "}"; 
			{
				cereal::JSONInputArchive archive(ss);
				archive(cereal::make_nvp("data", data));
			}  
		}
		catch (const std::exception&)
		{
			std::cout << "  1 "<< iter->second->s_Img_path << std::endl;			
			try
			{
				 sfm_data.views.erase(iter++);
			}
			catch (const std::exception&e)
			{
				std::cout <<e.what() << std::endl;
			}
			std::cout << "  3 ";
			std::cout << "  2 " << iter->second->s_Img_path << std::endl;
			continue;
		}
		if (data.frontLandmarks3d.size()==0)
		{
			sfm_data.views.erase(iter++);
			continue;
		}
		std::vector<bool>isCoveredLandmark(data.frontLandmarks3d.size(), false);
		Eigen::MatrixXf pts;
		Eigen::MatrixXi faces;
		if (landmarkCnt<0)
		{
			landmarkCnt = data.frontLandmarks3d.size();			
		}
		else
		{
			if (landmarkCnt != data.frontLandmarks3d.size())
			{
				std::cout <<"  !!!  " << std::endl;
			}
		} 
		{
			pts = vector3ToEigen<float, double>(data.frontLandmarks3d);
			faces = vector3ToEigen<int, int>(data.faces);
			mesh::reorderFaces(faces);
			std::map<int, std::list<int>>ptBelongToFaces;
			for (int f = 0; f < faces.rows(); f++)
			{
				const int& pa = faces(f, 0);
				const int& pb = faces(f, 1);
				const int& pc = faces(f, 2);
				ptBelongToFaces[pa].emplace_back(f);
				ptBelongToFaces[pb].emplace_back(f);
				ptBelongToFaces[pc].emplace_back(f);
			}
#pragma omp parallel for
			for (int i = 0; i < pts.rows(); i++)
			{
				if (pts(i, 0) < 0.1 * data.imgWidth || pts(i, 0) > 0.9 * data.imgWidth
					|| pts(i, 1) < 0.1 * data.imgHeight || pts(i, 1) > 0.9 * data.imgHeight)
				{
					isCoveredLandmark[i] = true;
				}
			}
#pragma omp parallel for
			for (int i = 0; i < pts.rows(); i++)
			{
				if (isCoveredLandmark[i])continue;
				for (int f = 0; f < faces.rows(); f++)
				{
					const int& pa = faces(f, 0);
					const int& pb = faces(f, 1);
					const int& pc = faces(f, 2);
					float t, v, u;

					float original[3] = { pts(i, 0), pts(i, 1) , pts(i,2) - 3 };
					float parallelDir[3] = { 0.,0,1 };
					float p1[3] = { pts(pa, 0), pts(pa, 1) , pts(pa,2) };
					float p2[3] = { pts(pb, 0), pts(pb, 1) , pts(pb,2) };
					float p3[3] = { pts(pc, 0), pts(pc, 1) , pts(pc,2) };
					int hit = intersect_triangle1<float>(original, parallelDir, p1, p2, p3, &t, &u, &v);
					if (hit)
					{
						bool isOtherFace = (ptBelongToFaces[i].end() == std::find(ptBelongToFaces[i].begin(), ptBelongToFaces[i].end(), f));
						if (isOtherFace)
						{
							isCoveredLandmark[i] = true;
							break;
						}
					}
				}
			}
			std::string objPath = stlplus::create_filespec(landmarksRoot, imgName, "obj");
			std::fstream fout(objPath, std::ios::out);
			for (size_t i = 0; i < pts.rows(); i++)
			{
				fout << "v " << pts(i, 0) << " " << pts(i, 1) << " " << pts(i, 2) << std::endl;
			}
			for (size_t f = 0; f < faces.rows(); f++)
			{
				const int& pa = faces(f, 0);
				const int& pb = faces(f, 1);
				const int& pc = faces(f, 2);
				if (isCoveredLandmark[pa] || isCoveredLandmark[pb] || isCoveredLandmark[pc])
				{
					continue;
				}
				fout << "f " << pa + 1 << " " << pb + 1 << " " << pc + 1 << std::endl;
			}
		}
		std::string extension = stlplus::extension_part(imgName);
		std::string stem = imgName.substr(0, imgName.length() - extension.length() - 1);
		std::string thisFeaturePath = stlplus::create_filespec(featureRoot, stem, "feat"); 
		std::string thisDescripPath = stlplus::create_filespec(featureRoot, stem, "desc");
		//image_describer->Load(regions_ptr.get(), thisFeaturePath, thisDescripPath);
		regions_ptr.get()->Features().clear();
		regions_ptr.get()->Descriptors().clear();
		int validCnt = 0;
		for (int i = 0; i < data.frontLandmarks3d.size(); i++)
		{
			openMVG::features::SIFT_Regions::FeatureT thisFeat;
			if (isCoveredLandmark[i])
			{
				continue;
			}
			thisFeat.x() = data.frontLandmarks3d[i][0];
			thisFeat.y() = data.frontLandmarks3d[i][1]; 
			
			regions_ptr.get()->Features().emplace_back(thisFeat);
			
			regions_ptr.get()->Descriptors().emplace_back(getRandDescripBaesOnIdx(descriptorLength,i)); 
			validCnt++;
		}  
		std::cout << "thisFeaturePath : " << thisFeaturePath << " : "<< regions_ptr.get()->Features().size() << std::endl;
		std::cout << "thisDescripPath : " << thisDescripPath << " : "<< regions_ptr.get()->Descriptors().size()<<std::endl;
		image_describer->Save(regions_ptr.get(), thisFeaturePath, thisDescripPath);
		iter++;
	}

	if (!openMVG::sfm::Save(sfm_data, sfmJsonPath, openMVG::sfm::ESfM_Data(openMVG::sfm::ALL)))
	{
		OPENMVG_LOG_ERROR << "The input SfM_Data file \"" << sfmJsonPath << "\" cannot be write.";
		return EXIT_FAILURE;
	}
	std::cout << "end" << std::endl;
	return 0;
}

int main(int argc, char** argv)
{
	if (argc!=3)
	{
		std::cout<<"cmd landmarksRoot sfmJsonPath" << std::endl;
		return -1;
	}
	else
	{
		std::cout << " You called:" << std::endl;
		std::cout <<"  ---"<<argv[0] << std::endl;
		std::cout <<"  ---"<<argv[1] << std::endl;
		std::cout <<"  ---"<<argv[2] << std::endl;
	}
	{
		std::string landmarksRoot = argv[1];
		std::string sfmJsonPath = argv[2];
		replaceFeature(landmarksRoot, sfmJsonPath);
	}
	 return 0;
	//构造自己的sfm_data.bin
	if(0)
	{
		openMVG::sfm::SfM_Data new_sfm_data;
		new_sfm_data.s_root_path = "d:/repo/mvs_mvg_bat/worldbuild";
		std::shared_ptr<openMVG::cameras::IntrinsicBase> intrinsic = std::make_shared<openMVG::cameras::Pinhole_Intrinsic_Radial_K3>
			(720, 1280, 2000, 720/2, 1280/2, 0.0, 0.0, 0.0);  // setup no distortion as initial guess; 
		new_sfm_data.intrinsics[0] = intrinsic;
		{
			openMVG::sfm::View v("im_0.jpg", 0, 0, 0, 720, 1280);
			new_sfm_data.views[0] = std::make_shared<openMVG::sfm::View>(v); 
			new_sfm_data.poses[0]= openMVG::sfm::Pose3(openMVG::Mat3::Identity(), openMVG::Vec3::Zero());
		}
		{
			openMVG::sfm::View v("im_22.jpg", 1, 0, 1, 720, 1280);
			new_sfm_data.views[1] = std::make_shared<openMVG::sfm::View>(v); 
			openMVG::Mat3 R1;			R1<< 0.993072, -0.0185299, -0.116034,
				0.0222691, 0.999271, 0.0310122,
				0.115374, -0.0333813, 0.992761;
			openMVG::Vec3 t1;			t1 << -0.0802932, 0.0680057, -0.994449;
			new_sfm_data.poses[1] = openMVG::sfm::Pose3(R1, t1);
		}
		Save(new_sfm_data,	"E:/viewerout/sfm/sfm_data.bin", openMVG::sfm::ESfM_Data(openMVG::sfm::ALL));
		return 0;
	}



	std::string additionalMatchPath = "D:/repo/mvs_mvg_bat/additionalMatch.txt";
	std::string sfmJsonPath = "E:/viewerout/sfm/matches/sfm_data.json"; 
	std::string matchImgPairsPath = "E:/viewerout/sfm/matches/pairs.bin";;
	std::string matchImgPutativePairsPath = "E:/viewerout/sfm/matches/matches.putative.bin";
	std::string sfmOut = "E:/viewerout/sfm";
	std::string matchingMethod = "ANNL2";
	std::string geometricFilterOutPath = "E:/viewerout/sfm/matches/matches.e.bin";
	float        fDistRatio = 0.8f;

	if (sfmJsonPath.empty())
	{
		OPENMVG_LOG_ERROR << "It is an invalid SfM file";
		return EXIT_FAILURE;
	} 	

	const std::string featureRoot = stlplus::folder_part(sfmJsonPath);
	const std::string sImage_describer = stlplus::create_filespec(featureRoot, "image_describer", "json");
	std::unique_ptr<openMVG::features::Regions> regions_type = openMVG::features::Init_region_type_from_file(sImage_describer);	 
	if (!regions_type)
	{
		OPENMVG_LOG_ERROR << "Invalid: " << sImage_describer << " regions type file.";
		return EXIT_FAILURE;
	}
	openMVG::sfm::SfM_Data sfm_data;
	if (!openMVG::sfm::Load(sfm_data, sfmJsonPath, openMVG::sfm::ESfM_Data(openMVG::sfm::VIEWS)))
	{
		OPENMVG_LOG_ERROR << "The input SfM_Data file \"" << sfmJsonPath << "\" cannot be read.";
		return EXIT_FAILURE;
	}

	  
	readAdditionalFeatures(additionalMatchPath,   sfm_data,  featureRoot);

	/// <summary>
	/// openMVG_main_ComputeMatches 
	if (matchImgPairsPath.empty())
	{
		OPENMVG_LOG_ERROR << "No output file set."<< matchImgPairsPath;
		return EXIT_FAILURE;
	}
	std::unique_ptr<openMVG::matching_image_collection::Matcher> collectionMatcher(new openMVG::matching_image_collection::Matcher_Regions(fDistRatio, openMVG::matching::ANN_L2));;
	openMVG::matching::PairWiseMatches map_PutativeMatches;
	openMVG::system::Timer timerForMatch;
	openMVG::system::LoggerProgress progressForMatch;
	
	// From matching mode compute the pair list that have to be matched:
	openMVG::Pair_Set pairs = openMVG:: exhaustivePairs(sfm_data.GetViews().size());
	OPENMVG_LOG_INFO << "Running matching on #pairs: " << pairs.size();
	// Photometric matching of putative pairs
	std::shared_ptr<openMVG::sfm::Regions_Provider> regions_provider = std::make_shared<openMVG::sfm::Regions_Provider>();
	if (!regions_provider->load(sfm_data, featureRoot, regions_type, &progressForMatch)) {
		OPENMVG_LOG_ERROR << "Cannot load view regions from: " << featureRoot << ".";
		return EXIT_FAILURE;
	}
	collectionMatcher->Match(regions_provider, pairs, map_PutativeMatches, &progressForMatch);
	//---------------------------------------
	//-- Export putative matches & pairs
	//---------------------------------------
	if (!Save(map_PutativeMatches, matchImgPutativePairsPath))
	{
		OPENMVG_LOG_ERROR
			<< "Cannot save computed matches in: "
			<< matchImgPutativePairsPath;
		return EXIT_FAILURE;
	}
	// Save pairs
	const std::string sOutputPairFilename =
		stlplus::create_filespec(featureRoot, "preemptive_pairs", "txt");
	if (!openMVG::savePairs(
		sOutputPairFilename,
		getPairs(map_PutativeMatches)))
	{
		OPENMVG_LOG_ERROR
			<< "Cannot save computed matches pairs in: "
			<< sOutputPairFilename;
		return EXIT_FAILURE;
	}
	
	OPENMVG_LOG_INFO << "Task (Regions Matching) done in (s): " << timerForMatch.elapsed();

	/// <summary>
	/// openMVG_main_GeometricFilter 
	std::string  sGeometricModel = "e";
	int          imax_iteration = 2048;
	bool         bGuided_matching = false;
	const double  d_distance_ratio = 0.6;
	openMVG::system::LoggerProgress progressForGeometricFilter;
	enum EGeometricModel
	{
		FUNDAMENTAL_MATRIX = 0,
		ESSENTIAL_MATRIX = 1,
		HOMOGRAPHY_MATRIX = 2,
		ESSENTIAL_MATRIX_ANGULAR = 3,
		ESSENTIAL_MATRIX_ORTHO = 4,
		ESSENTIAL_MATRIX_UPRIGHT = 5
	};
	EGeometricModel eGeometricModelToCompute = ESSENTIAL_MATRIX;
	std::unique_ptr<openMVG::matching_image_collection::ImageCollectionGeometricFilter> filter_ptr(
		new openMVG::matching_image_collection::ImageCollectionGeometricFilter(&sfm_data, regions_provider));
	openMVG::matching::PairWiseMatches specifiedMatch;
	if (filter_ptr)
	{
		openMVG::system::Timer timerForGeometricFilter;
		const double  d_distance_ratio = 0.6;

		openMVG::matching::PairWiseMatches map_GeometricMatches;
		
		filter_ptr->Robust_model_estimation(
			openMVG::matching_image_collection::GeometricFilter_EMatrix_AC(4.0, imax_iteration),
			map_PutativeMatches,
			bGuided_matching,
			d_distance_ratio,
			&progressForGeometricFilter);
		map_GeometricMatches = filter_ptr->Get_geometric_matches();

		//-- Perform an additional check to remove pairs with poor overlap
		std::vector<openMVG::matching::PairWiseMatches::key_type> vec_toRemove;
		for (const auto& pairwisematches_it : map_GeometricMatches)
		{
			const size_t putativePhotometricCount = map_PutativeMatches.find(pairwisematches_it.first)->second.size();
			const size_t putativeGeometricCount = pairwisematches_it.second.size();
			const float  ratio = putativeGeometricCount / static_cast<float>(putativePhotometricCount);
			if (putativeGeometricCount < 50 || ratio < .3f)
			{
				// the pair will be removed
				vec_toRemove.push_back(pairwisematches_it.first);
			}
		}
		//-- remove discarded pairs
		for (const auto& pair_to_remove_it : vec_toRemove)
		{
			map_GeometricMatches.erase(pair_to_remove_it);
		}

		auto additionalPointMap = loadAdditionalPointMap("D:/repo/mvs_mvg_bat/additionalMatch.txt");
		std::map<std::string, openMVG::IndexT>additionalImgsIndex = getImgsIndex(sfm_data, additionalPointMap);
		specifiedMatch = generExclusionPairs(additionalPointMap, additionalImgsIndex);
		for (auto& d : specifiedMatch)
		{
			if (map_GeometricMatches.count(d.first) != 0)
			{
				map_GeometricMatches[d.first] = d.second;
			}
			else
			{
				map_GeometricMatches.insert(d);
			}
		}
		//---------------------------------------
		//-- Export geometric filtered matches
		//---------------------------------------
		if (!Save(map_GeometricMatches, geometricFilterOutPath))
		{
			OPENMVG_LOG_ERROR << "Cannot save filtered matches in: " << geometricFilterOutPath;
			return EXIT_FAILURE;
		}

		// -- export Geometric View Graph statistics
		openMVG::graph::getGraphStatistics(sfm_data.GetViews().size(), getPairs(map_GeometricMatches));

		OPENMVG_LOG_INFO << "Task done in (s): " << timerForGeometricFilter.elapsed();

		//-- export Adjacency matrix
		OPENMVG_LOG_INFO << "\n Export Adjacency Matrix of the pairwise's geometric matches";

		PairWiseMatchingToAdjacencyMatrixSVG(sfm_data.GetViews().size(),
			map_GeometricMatches,
			stlplus::create_filespec(featureRoot, "GeometricAdjacencyMatrix", "svg"));

		const openMVG::Pair_Set outputPairs = getPairs(map_GeometricMatches);

		//-- export view pair graph once geometric filter have been done
		{
			std::set<openMVG::IndexT> set_ViewIds;
			std::transform(sfm_data.GetViews().begin(), sfm_data.GetViews().end(), std::inserter(set_ViewIds, set_ViewIds.begin()), stl::RetrieveKey());
			openMVG::graph::indexedGraph putativeGraph(set_ViewIds, outputPairs);
			openMVG::graph::exportToGraphvizData(
				stlplus::create_filespec(featureRoot, "geometric_matches"),
				putativeGraph);
		}

		 
	}

	/// <summary>
	/// openMVG_main_sfm

	if (!openMVG::sfm::Load(sfm_data, sfmJsonPath, openMVG::sfm::ESfM_Data(openMVG::sfm::VIEWS| openMVG::sfm::INTRINSICS)))
	{
		OPENMVG_LOG_ERROR << "The input SfM_Data file \"" << sfmJsonPath << "\" cannot be read.";
		return EXIT_FAILURE;
	}
	int graph_simplification_value = 5;

	std::string sfm_initializer_method = "STELLAR";
	ESfMSceneInitializer scene_initializer_enum;
	if (!StringToEnum(sfm_initializer_method, scene_initializer_enum))
	{
		OPENMVG_LOG_ERROR << "Invalid input for the SfM initializer option";
		return EXIT_FAILURE;
	}
	ESfMEngine sfm_engine_type = ESfMEngine::GLOBAL_INSERT;

	  // Features reading
	std::shared_ptr<openMVG::sfm::Features_Provider> feats_provider = std::make_shared<openMVG::sfm::Features_Provider>();
	if (!feats_provider->load(sfm_data, featureRoot, regions_type)) {
		OPENMVG_LOG_ERROR << "Cannot load view corresponding features in directory: " << featureRoot << ".";
		return EXIT_FAILURE;
	}
	// Matches reading
	std::shared_ptr<openMVG::sfm::Matches_Provider> matches_provider = std::make_shared<openMVG::sfm::Matches_Provider>();
	if // Try to read the provided match filename or the default one (matches.f.txt/bin)
		(
			!(
				matches_provider->load(sfm_data, stlplus::create_filespec(featureRoot, "matches.f.txt")) ||
				matches_provider->load(sfm_data, stlplus::create_filespec(featureRoot, "matches.f.bin")) ||
				matches_provider->load(sfm_data, stlplus::create_filespec(featureRoot, "matches.e.txt")) ||
				matches_provider->load(sfm_data, stlplus::create_filespec(featureRoot, "matches.e.bin")))
			)
	{
		OPENMVG_LOG_ERROR << "Cannot load the match file.";
		return EXIT_FAILURE;
	}
	std::unique_ptr<openMVG::sfm::SfMSceneInitializer> scene_initializer;
	switch (scene_initializer_enum)
	{
	case ESfMSceneInitializer::INITIALIZE_AUTO_PAIR:
		OPENMVG_LOG_ERROR << "Not yet implemented.";
		return EXIT_FAILURE;
		break;
	case ESfMSceneInitializer::INITIALIZE_MAX_PAIR:
		scene_initializer.reset(new openMVG::sfm::SfMSceneInitializerMaxPair(sfm_data,
			feats_provider.get(),
			matches_provider.get()));
		break;
	case ESfMSceneInitializer::INITIALIZE_EXISTING_POSES:
		scene_initializer.reset(new openMVG::sfm::SfMSceneInitializer(sfm_data,
			feats_provider.get(),
			matches_provider.get()));
		break;
	case ESfMSceneInitializer::INITIALIZE_STELLAR:
		scene_initializer.reset(new openMVG::sfm::SfMSceneInitializerStellar(sfm_data,
			feats_provider.get(),
			matches_provider.get()));
		break;
	default:
		OPENMVG_LOG_ERROR << "Unknown SFM Scene initializer method";
		return EXIT_FAILURE;
	}
	if (!scene_initializer)
	{
		OPENMVG_LOG_ERROR << "Invalid scene initializer.";
		return EXIT_FAILURE;
	}

	std::unique_ptr<openMVG::sfm::ReconstructionEngine> sfm_engine;
	switch (sfm_engine_type)
	{
	case ESfMEngine::INCREMENTAL:
	{
		openMVG::sfm::SequentialSfMReconstructionEngine* engine =
			new openMVG::sfm::SequentialSfMReconstructionEngine(
				sfm_data,
				sfmOut,
				stlplus::create_filespec(sfmOut, "Reconstruction_Report.html"));

		// Configuration:
		engine->SetFeaturesProvider(feats_provider.get());
		engine->SetMatchesProvider(matches_provider.get());

		// Configure reconstruction parameters
		engine->SetUnknownCameraType(openMVG::cameras::EINTRINSIC::PINHOLE_CAMERA_RADIAL3);
		engine->SetTriangulationMethod(openMVG::ETriangulationMethod::DEFAULT);
		engine->SetResectionMethod(openMVG::resection::SolverType::DEFAULT);

		// Handle Initial pair parameter
		//if (!initial_pair_string.first.empty() && !initial_pair_string.second.empty())
		//{
		//	Pair initial_pair_index;
		//	if (!computeIndexFromImageNames(sfm_data, initial_pair_string, initial_pair_index))
		//	{
		//		OPENMVG_LOG_ERROR << "Could not find the initial pairs <" << initial_pair_string.first
		//			<< ", " << initial_pair_string.second << ">!";
		//		return EXIT_FAILURE;
		//	}
		//	engine->setInitialPair(initial_pair_index);
		//}
		//sfm_engine.reset(engine);
	}
	break;
	case ESfMEngine::INCREMENTALV2:
	{
		//SequentialSfMReconstructionEngine2* engine =
		//	new SequentialSfMReconstructionEngine2(
		//		scene_initializer.get(),
		//		sfm_data,
		//		directory_output,
		//		stlplus::create_filespec(directory_output, "Reconstruction_Report.html"));
		//engine->SetFeaturesProvider(feats_provider.get());
		//engine->SetMatchesProvider(matches_provider.get());
		//engine->SetTriangulationMethod(static_cast<ETriangulationMethod>(triangulation_method));
		//engine->SetUnknownCameraType(EINTRINSIC(user_camera_model));
		//engine->SetResectionMethod(static_cast<resection::SolverType>(resection_method));
		//sfm_engine.reset(engine);
	}
	break;
	case ESfMEngine::GLOBAL:
	{
		openMVG::sfm::GlobalSfMReconstructionEngine_RelativeMotions* engine =
			new openMVG::sfm::GlobalSfMReconstructionEngine_RelativeMotions(
				sfm_data,
				sfmOut,
				stlplus::create_filespec(sfmOut, "Reconstruction_Report.html"));

		// Configuration:
		engine->SetFeaturesProvider(feats_provider.get());
		engine->SetMatchesProvider(matches_provider.get());

		// Configure motion averaging method
		engine->SetRotationAveragingMethod(openMVG::sfm::ERotationAveragingMethod(openMVG::sfm::ERotationAveragingMethod::ROTATION_AVERAGING_L2));
		engine->SetTranslationAveragingMethod(openMVG::sfm::ETranslationAveragingMethod(openMVG::sfm::ETranslationAveragingMethod::TRANSLATION_AVERAGING_SOFTL1));

		sfm_engine.reset(engine);
	}
	break;
	case ESfMEngine::GLOBAL_INSERT:
	{
		//openMVG::sfm::GlobalSfMReconstructionEngine_RelativeMotions_Insert* engine =
		//	new openMVG::sfm::GlobalSfMReconstructionEngine_RelativeMotions_Insert(
		//		sfm_data,
		//		specifiedMatch,
		//		sfmOut,
		//		stlplus::create_filespec(sfmOut, "Reconstruction_Report.html"));
		// Configuration:
		//engine->SetFeaturesProvider(feats_provider.get());
		//engine->SetMatchesProvider(matches_provider.get());
		//// Configure motion averaging method
		//engine->SetRotationAveragingMethod(openMVG::sfm::ERotationAveragingMethod(openMVG::sfm::ERotationAveragingMethod::ROTATION_AVERAGING_L2));
		//engine->SetTranslationAveragingMethod(openMVG::sfm::ETranslationAveragingMethod(openMVG::sfm::ETranslationAveragingMethod::TRANSLATION_AVERAGING_SOFTL1));
		//sfm_engine.reset(engine);
	}
	break;
	case ESfMEngine::STELLAR:
	{
		//openMVG::sfm::StellarSfMReconstructionEngine* engine =
		//	new openMVG::sfm::StellarSfMReconstructionEngine(
		//		sfm_data,
		//		sfmOut,
		//		stlplus::create_filespec(sfmOut, "Reconstruction_Report.html"));
		//// Configure the features_provider & the matches_provider
		//engine->SetFeaturesProvider(feats_provider.get());
		//engine->SetMatchesProvider(matches_provider.get());
		//// Configure reconstruction parameters
		//engine->SetGraphSimplification(openMVG::sfm::EGraphSimplification::MST_X, graph_simplification_value);
		//sfm_engine.reset(engine);
	}
	break;
	default:
		break;
	}
	if (!sfm_engine)
	{
		OPENMVG_LOG_ERROR << "Cannot create the requested SfM Engine.";
		return EXIT_FAILURE;
	}

	sfm_engine->Set_Intrinsics_Refinement_Type(openMVG::cameras::Intrinsic_Parameter_Type::ADJUST_ALL);
	sfm_engine->Set_Extrinsics_Refinement_Type(openMVG::sfm::Extrinsic_Parameter_Type::ADJUST_ALL);
	sfm_engine->Set_Use_Motion_Prior(false);
	openMVG::system::Timer timerForSfm;
	if (sfm_engine->Process())
	{
		OPENMVG_LOG_INFO << " Total Sfm took (s): " << timerForSfm.elapsed();

		OPENMVG_LOG_INFO << "...Generating SfM_Report.html";
		openMVG::sfm::Generate_SfM_Report(sfm_engine->Get_SfM_Data(),
			stlplus::create_filespec(sfmOut, "SfMReconstruction_Report.html"));

		//-- Export to disk computed scene (data & viewable results)
		OPENMVG_LOG_INFO << "...Export SfM_Data to disk.";
		Save(sfm_engine->Get_SfM_Data(),
			stlplus::create_filespec(sfmOut, "sfm_data", ".bin"),
			openMVG::sfm::ESfM_Data::ALL);

		Save(sfm_engine->Get_SfM_Data(),
			stlplus::create_filespec(sfmOut, "cloud_and_poses", ".ply"),
			openMVG::sfm::ESfM_Data::ALL);

		return EXIT_SUCCESS;
	}

	return 0;
}