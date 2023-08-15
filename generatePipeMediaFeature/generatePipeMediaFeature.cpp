
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
#include "openMVG/sfm/pipelines/sfm_preemptive_regions_provider.hpp"
#include "openMVG/sfm/pipelines/sfm_regions_provider.hpp"
#include "openMVG/sfm/pipelines/sfm_regions_provider_cache.hpp"
//#include "openMVG/sfm/pipelines/stellar/sfm_stellar_engine.hpp"
//#include "GlobalSfMReconstructionEngine_RelativeMotions_insert.h"

#include "openMVG/features/akaze/image_describer_akaze_io.hpp"
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
#include "omp.h"

#include "mesh.h"

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

static std::vector<unsigned char> randData; 
openMVG::features::SIFT_Image_describer::Regions_type::DescriptorT getRandDescripBaesOnIdx(const int & descriptorLength, const int&  i)
{
	
	openMVG::features::SIFT_Image_describer::Regions_type::DescriptorT randDescrip;
	while (randData.size() < i + descriptorLength)
	{
		randData.emplace_back(rand() % 256);
	}
	for (int ii = 0; ii < descriptorLength; ii++) randDescrip[ii] = randData[i + ii];
	return randDescrip;
}

int generateFeature(int argc, char** argv)
{
	//std::string landmarksRoot = argv[1];
	//std::string sfmJsonPath  = argv[2];
	//std::string featureOutdir = argv[3];
	std::string landmarksRoot = "D:/repo/mvs_mvg_bat/viewerout/landmarks";
	std::string sfmJsonPath = "D:/repo/mvs_mvg_bat/viewerout/sfm/matches/sfm_data.json";
	std::string  featureOutdir = "D:/repo/mvs_mvg_bat/viewerout/sfm/matches";
	const std::string sOutputMatchesFilename = stlplus::create_filespec(featureOutdir, "matches.putative.bin");
	bool bUpRight = false;
	int iNumThreads = 0; 
	openMVG::sfm::SfM_Data sfm_data;
	if (!Load(sfm_data, sfmJsonPath, openMVG::sfm::ESfM_Data(openMVG::sfm::VIEWS | openMVG::sfm::INTRINSICS))) {
		OPENMVG_LOG_ERROR
			<< "The input file \"" << sfmJsonPath << "\" cannot be read";
		return EXIT_FAILURE;
	}
	std::unique_ptr<openMVG::features::Image_describer> image_describer;
	image_describer.reset(new openMVG::features::SIFT_Image_describer
	(openMVG::features::SIFT_Image_describer::Params(), !bUpRight));
	const std::string sImage_describer = stlplus::create_filespec(featureOutdir, "image_describer", "json");
	{
		std::ofstream stream(sImage_describer.c_str());
		if (!stream)
			return EXIT_FAILURE;

		cereal::JSONOutputArchive archive(stream);
		auto regionsType = image_describer->Allocate();
		archive(cereal::make_nvp("regions_type", regionsType));
	} 
	std::unique_ptr<openMVG::features::Regions> regions_type = openMVG::features::Init_region_type_from_file(sImage_describer);
	std::map<openMVG::IndexT, std::vector<openMVG::IndexT>> imgFeatureIdx;
	std::vector<int> imgIdx;
	imgIdx.reserve(sfm_data.views.size()); 
	std::vector<std::map<openMVG::IndexT, int>> featureIdxs(sfm_data.views.size());
	{
		openMVG::system::Timer timer;
		openMVG::image::Image<unsigned char> imageGray;

		openMVG::system::LoggerProgress my_progress_bar(sfm_data.GetViews().size(), "- EXTRACT FEATURES -");

		// Use a boolean to track if we must stop feature extraction
		std::atomic<bool> preemptive_exit(false);
#ifdef OPENMVG_USE_OPENMP
		const unsigned int nb_max_thread = omp_get_max_threads();

		if (iNumThreads > 0) {
			omp_set_num_threads(iNumThreads);
		}
		else {
			omp_set_num_threads(nb_max_thread);
		}
#pragma omp parallel for
#endif
		for (int viewIdx = 0; viewIdx < static_cast<int>(sfm_data.views.size()); ++viewIdx)
		{
			openMVG::sfm::Views::const_iterator iterViews = sfm_data.views.begin();
			std::advance(iterViews, viewIdx);
			const openMVG::sfm::View* view = iterViews->second.get();
			const std::string sView_filename = stlplus::create_filespec(sfm_data.s_root_path, view->s_Img_path);
			const std::string	sFeat = stlplus::create_filespec(featureOutdir, stlplus::basename_part(sView_filename), "feat");
			const std::string	sDesc = stlplus::create_filespec(featureOutdir, stlplus::basename_part(sView_filename), "desc");
			std::string jsonPath = stlplus::create_filespec(landmarksRoot, stlplus::basename_part(sView_filename), "json");
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
				continue;
			}
			if (data.frontLandmarks3d.size() == 0)
			{
				continue;
			}
			std::vector<bool>isCoveredLandmark(data.frontLandmarks3d.size(), false);
			Eigen::MatrixXf pts;
			Eigen::MatrixXi faces;
			if (data.frontLandmarks3d.size() < 0)
			{
				continue;
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
				for (int ptIdx = 0; ptIdx < pts.rows(); ptIdx++)
				{
					if (pts(ptIdx, 0) < 0.1 * data.imgWidth || pts(ptIdx, 0) > 0.9 * data.imgWidth
						|| pts(ptIdx, 1) < 0.1 * data.imgHeight || pts(ptIdx, 1) > 0.9 * data.imgHeight)
					{
						isCoveredLandmark[ptIdx] = true;
					}
				}
#pragma omp parallel for
				for (int ptIdx = 0; ptIdx < pts.rows(); ptIdx++)
				{
					if (isCoveredLandmark[ptIdx])continue;
					for (int f = 0; f < faces.rows(); f++)
					{
						const int& pa = faces(f, 0);
						const int& pb = faces(f, 1);
						const int& pc = faces(f, 2);
						float t, v, u;

						float original[3] = { pts(ptIdx, 0), pts(ptIdx, 1) , pts(ptIdx,2) - 3 };
						float parallelDir[3] = { 0.,0,1 };
						float p1[3] = { pts(pa, 0), pts(pa, 1) , pts(pa,2) };
						float p2[3] = { pts(pb, 0), pts(pb, 1) , pts(pb,2) };
						float p3[3] = { pts(pc, 0), pts(pc, 1) , pts(pc,2) };
						int hit = intersect_triangle1<float>(original, parallelDir, p1, p2, p3, &t, &u, &v);
						if (hit)
						{
							bool isOtherFace = (ptBelongToFaces[ptIdx].end() == std::find(ptBelongToFaces[ptIdx].begin(), ptBelongToFaces[ptIdx].end(), f));
							if (isOtherFace)
							{
								isCoveredLandmark[ptIdx] = true;
								break;
							}
						}
					}
				}
				std::string objPath = stlplus::create_filespec(landmarksRoot, stlplus::basename_part(sView_filename), "obj");
				std::fstream fout(objPath, std::ios::out);
				for (size_t ptIdx = 0; ptIdx < pts.rows(); ptIdx++)
				{
					fout << "v " << pts(ptIdx, 0) << " " << pts(ptIdx, 1) << " " << pts(ptIdx, 2) << std::endl;
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

			std::unique_ptr<openMVG::features::SIFT_Image_describer::Regions_type> regions_ptr(new openMVG::features::SIFT_Image_describer::Regions_type());
			 
			int descriptorLength = regions_ptr->DescriptorLength();
			regions_ptr.get()->Features().clear();
			regions_ptr.get()->Descriptors().clear();
			//auto region = image_describer.get()->Allocate();
			//image_describer.get()->
			int validCnt = 0;
			imgFeatureIdx[view->id_view] = std::vector<openMVG::IndexT>();
			imgFeatureIdx[view->id_view].reserve(512);
			imgIdx.emplace_back(view->id_view);
			featureIdxs[view->id_view];
			for (openMVG::IndexT landmarkIdx = 0; landmarkIdx < data.frontLandmarks3d.size(); landmarkIdx++)
			{
				openMVG::features::SIFT_Regions::FeatureT thisFeat;
				if (isCoveredLandmark[landmarkIdx])
				{
					featureIdxs[view->id_view][landmarkIdx] = -1;
					continue;
				}
				thisFeat.x() = data.frontLandmarks3d[landmarkIdx][0];
				thisFeat.y() = data.frontLandmarks3d[landmarkIdx][1];
				regions_ptr.get()->Features().emplace_back(thisFeat);
				regions_ptr.get()->Descriptors().emplace_back(getRandDescripBaesOnIdx(descriptorLength, landmarkIdx));
				imgFeatureIdx[view->id_view].emplace_back(landmarkIdx); 
				featureIdxs[view->id_view][landmarkIdx] = validCnt;
				validCnt++;
			}
			//std::cout << "thisFeaturePath : " << sFeat << " : " << regions_ptr.get()->Features().size() << std::endl;
			//std::cout << "thisDescripPath : " << sDesc << " : " << regions_ptr.get()->Descriptors().size() << std::endl;
			regions_ptr->Save(sFeat, sDesc);
			//image_describer->Save(image_describer.get()->Allocate(), sFeat, sDesc)
			++my_progress_bar;
		}
		auto iter = sfm_data.views.begin();

		for (; iter != sfm_data.views.end();)
		{
			const openMVG::sfm::View* view = iter->second.get();
			const std::string sView_filename = stlplus::create_filespec(sfm_data.s_root_path, view->s_Img_path);
			const std::string	sFeat = stlplus::create_filespec(featureOutdir, stlplus::basename_part(sView_filename), "feat");
			if (!stlplus::file_exists(sFeat))
			{
				iter = sfm_data.views.erase(iter);
				std::cout << "empty feature : " << sView_filename << std::endl;
			}
			else
			{
				//image_describer.get()->LoadFeatures(sFeat);
				//std::cout << "thisDescripPath : " << sView_filename << " : " << image_describer.get()->Features().size() << std::endl;
				iter++;
			}
		}
		OPENMVG_LOG_INFO << "Task done in (s): " << timer.elapsed();
	}
	Save(sfm_data, sfmJsonPath, openMVG::sfm::ESfM_Data(openMVG::sfm::VIEWS | openMVG::sfm::INTRINSICS));
	//return 0;
	
	openMVG::matching::PairWiseMatches map_PutativeMatches;
	for (openMVG::IndexT i = 0; i < imgIdx.size(); i++)
	{
		for (openMVG::IndexT j = i+1; j < imgIdx.size(); j++)
		{
			openMVG::Pair currentPair(i,j);
			openMVG::matching::IndMatches currentMatch;
			for (openMVG::IndexT k = 0; k < imgFeatureIdx[i].size(); k++)
			{				
				if (featureIdxs[i][k]>=0 && featureIdxs[j][k] >= 0)
				{
					currentMatch.emplace_back(openMVG::matching::IndMatch(featureIdxs[i][k], featureIdxs[j][k]));
				}
				
			}
			if (currentMatch.size()>10)
			{
				map_PutativeMatches.insert(std::pair<openMVG::Pair, openMVG::matching::IndMatches>(currentPair, currentMatch));
			}			
		}
	}
	if (!Save(map_PutativeMatches, std::string(sOutputMatchesFilename)))
	{
		OPENMVG_LOG_ERROR
			<< "Cannot save computed matches in: "
			<< sOutputMatchesFilename;
		return EXIT_FAILURE;
	}
	if (!Save(map_PutativeMatches, std::string(sOutputMatchesFilename)+".2.txt"))
	{
		OPENMVG_LOG_ERROR
			<< "Cannot save computed matches in: "
			<< sOutputMatchesFilename;
		return EXIT_FAILURE;
	}
	return 0;
}
int pairGenerator(int argc, char** argv)
{
	std::string landmarksRoot = argv[1];
	std::string sfmJsonPath  = argv[2];
	std::string featureOutdir = argv[3];
	//std::string landmarksRoot = "D:/repo/mvs_mvg_bat/viewerout/landmarks";
	//std::string sfmJsonPath = "D:/repo/mvs_mvg_bat/viewerout/sfm/matches/sfm_data.json";
	//std::string  featureOutdir = "D:/repo/mvs_mvg_bat/viewerout/sfm/matches";
	const std::string sPredefinedPairList = stlplus::create_filespec(featureOutdir, "pairs.txt");
	openMVG::sfm::SfM_Data sfm_data;
	if (!Load(sfm_data, sfmJsonPath, openMVG::sfm::ESfM_Data(openMVG::sfm::VIEWS | openMVG::sfm::INTRINSICS)))
	{
		std::cerr << std::endl
			<< "The input SfM_Data file \"" << sfmJsonPath << "\" cannot be read." << std::endl;
		exit(EXIT_FAILURE);
	}
	const size_t NImage = sfm_data.GetViews().size();
	openMVG::Pair_Set pairs = openMVG::exhaustivePairs(NImage);
	if (!openMVG::savePairs(sPredefinedPairList, pairs))
	{
		std::cerr << "Failed to save pairs to file: \"" << sPredefinedPairList << "\"" << std::endl;
		exit(EXIT_FAILURE);
	}
}
int matchFeature(int argc, char** argv)
{
	std::string landmarksRoot = argv[1];
	std::string sfmJsonPath  = argv[2];
	std::string featureOutdir = argv[3];
	//std::string landmarksRoot = "D:/repo/mvs_mvg_bat/viewerout/landmarks";
	//std::string sfmJsonPath = "D:/repo/mvs_mvg_bat/viewerout/sfm/matches/sfm_data.json";
	//std::string  featureOutdir = "D:/repo/mvs_mvg_bat/viewerout/sfm/matches";
	const std::string sPredefinedPairList = stlplus::create_filespec(featureOutdir, "pairs.txt");
	const std::string sOutputMatchesFilename = stlplus::create_filespec(featureOutdir, "matches.putative.bin");
	// Create output dir
	if (!stlplus::folder_exists(featureOutdir))
	{
		if (!stlplus::folder_create(featureOutdir))
		{
			OPENMVG_LOG_ERROR << "Cannot create output directory";
			return EXIT_FAILURE;
		}
	}
	openMVG::sfm::SfM_Data sfm_data;
	if (!Load(sfm_data, sfmJsonPath, openMVG::sfm::ESfM_Data(openMVG::sfm::VIEWS | openMVG::sfm::INTRINSICS))) {
		OPENMVG_LOG_ERROR << "The input file \"" << sfmJsonPath << "\" cannot be read";
		return EXIT_FAILURE;
	}
	std::unique_ptr<openMVG::features::Binary_Regions<openMVG::features::SIOPointFeature, 256>> regions_ptr(new openMVG::features::Binary_Regions<openMVG::features::SIOPointFeature, 256>());
	std::unique_ptr<openMVG::features::Regions> regions_type;
	regions_type.reset(new openMVG::features::Binary_Regions<openMVG::features::SIOPointFeature, 256>());
	openMVG::system::LoggerProgress progress;
	std::shared_ptr<openMVG::sfm::Regions_Provider> regions_provider = std::make_shared<openMVG::sfm::Regions_Provider>();
	if (!regions_provider->load(sfm_data, featureOutdir, regions_type, &progress)) {
		OPENMVG_LOG_ERROR << "Cannot load view regions from: " << featureOutdir << ".";
		return EXIT_FAILURE;
	}

	openMVG::matching::PairWiseMatches map_PutativeMatches; 
	std::vector<std::string>               vec_fileNames;
	std::vector<std::pair<size_t, size_t>> vec_imagesSize;
	{
		vec_fileNames.reserve(sfm_data.GetViews().size());
		vec_imagesSize.reserve(sfm_data.GetViews().size());
		for (const auto view_it : sfm_data.GetViews())
		{
			const openMVG::sfm::View* v = view_it.second.get();
			vec_fileNames.emplace_back(stlplus::create_filespec(sfm_data.s_root_path,
				v->s_Img_path));
			vec_imagesSize.emplace_back(v->ui_width, v->ui_height);
		}
	}
	std::unique_ptr<openMVG::matching_image_collection::Matcher> collectionMatcher;
	OPENMVG_LOG_INFO << "Using BRUTE_FORCE_HAMMING matcher";
	collectionMatcher.reset(new openMVG::matching_image_collection::Matcher_Regions(0.99, openMVG::matching::BRUTE_FORCE_HAMMING));
	openMVG::system::Timer timer;
	{
		// From matching mode compute the pair list that have to be matched:
		openMVG::Pair_Set pairs;
		if (sPredefinedPairList.empty())
		{
			OPENMVG_LOG_INFO << "No input pair file set. Use exhaustive match by default."; 
		}
		else
			if (!openMVG::loadPairs(sfm_data.GetViews().size(), sPredefinedPairList, pairs))
			{
				OPENMVG_LOG_ERROR << "Failed to load pairs from file: \"" << sPredefinedPairList << "\"";
				return EXIT_FAILURE;
			}
		OPENMVG_LOG_INFO << "Running matching on #pairs: " << pairs.size();
		// Photometric matching of putative pairs
		collectionMatcher->Match(regions_provider, pairs, map_PutativeMatches, &progress);



		//---------------------------------------
		//-- Export putative matches & pairs
		//---------------------------------------
		if (!Save(map_PutativeMatches, std::string(sOutputMatchesFilename)))
		{
			OPENMVG_LOG_ERROR
				<< "Cannot save computed matches in: "
				<< sOutputMatchesFilename;
			return EXIT_FAILURE;
		}
		if (!Save(map_PutativeMatches, std::string(sOutputMatchesFilename)+".txt"))
		{
			OPENMVG_LOG_ERROR
				<< "Cannot save computed matches in: "
				<< sOutputMatchesFilename;
			return EXIT_FAILURE;
		}
		// Save pairs
		const std::string sOutputPairFilename =
			stlplus::create_filespec(featureOutdir, "preemptive_pairs", "txt");
		if (!openMVG::savePairs(
			sOutputPairFilename,
			getPairs(map_PutativeMatches)))
		{
			OPENMVG_LOG_ERROR
				<< "Cannot save computed matches pairs in: "
				<< sOutputPairFilename;
			return EXIT_FAILURE;
		}
	}
	OPENMVG_LOG_INFO << "Task (Regions Matching) done in (s): " << timer.elapsed();
}
int main(int argc, char** argv)
{
	//if (argc!=4)
	//{
	//	std::cout<<"cmd landmarksRoot sfmJsonPath featureOutdir" << std::endl;
	//	return -1;
	//}
	//else
	//{
	//	std::cout << " You called:" << std::endl;
	//	std::cout << "  ---" << argv[0] << std::endl;
	//	std::cout << "  ---" << argv[1] << std::endl;
	//	std::cout << "  ---" << argv[2] << std::endl;
	//	std::cout << "  ---" << argv[3] << std::endl;
	//}	
	generateFeature(argc, argv);
	//pairGenerator(argc, argv);;
	//matchFeature(argc, argv);

	return 0;
}