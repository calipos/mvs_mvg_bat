//debug D:\ucl360\libraries2019\dlib1924\include;D:\ucl360\libraries2019;D:\BaiduNetdiskDownload\opencv-4.7.0\static_build\install\include;D:\ucl360\libraries2019\glog_debug\include
//release D:\ucl360\libraries2019\dlib1924\include;D:\ucl360\libraries2019;D:\BaiduNetdiskDownload\opencv-4.7.0\static_build\install\include;D:\ucl360\libraries2019\glog_release\include
//GLOG_NO_ABBREVIATED_SEVERITIES GOOGLE_GLOG_DLL_DECL =
//release D:\ucl360\libraries2019\dlib1924\lib;D:\BaiduNetdiskDownload\opencv-4.7.0\static_build\install\x64\vc16\staticlib;D:\ucl360\libraries2019\glog_release\lib
//debug D:\ucl360\libraries2019\dlib1924\lib;D:\BaiduNetdiskDownload\opencv-4.7.0\static_build\install\x64\vc16\staticlib;D:\ucl360\libraries2019\glog_debug\lib
#include "glog/logging.h"
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
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>
#include <dlib/gui_widgets.h>
#include <dlib/image_io.h> 
#include <dlib/dir_nav.h>
#include <iostream>
#include <vector>
#include <string>
#include <tuple>
#ifdef _DEBUG
//#pragma comment(lib,"opencv_core470d.lib")
//#pragma comment(lib,"opencv_highgui470d.lib")
//#pragma comment(lib,"opencv_imgcodecs470d.lib")
//#pragma comment(lib,"opencv_imgproc470d.lib")  
//#pragma comment(lib,"opencv_calib3d470d.lib")  
//#pragma comment(lib,"opencv_features2d470d.lib")
//#pragma comment(lib,"opencv_xfeatures2d470d.lib")
//#pragma comment(lib,"opencv_flann470d.lib")
//#pragma comment(lib,"zlibd.lib") 
//#pragma comment(lib,"libjpeg-turbod.lib")
//#pragma comment(lib,"libopenjp2d.lib") 
//#pragma comment(lib,"libtiffd.lib")
//#pragma comment(lib,"libpngd.lib")
//#pragma comment(lib,"IlmImfd.lib")
//#pragma comment(lib,"libwebpd.lib")
//#pragma comment(lib,"ittnotifyd.lib")
#pragma comment(lib,"dlib19.24.0_debug_64bit_msvc1929.lib")
#else
//#pragma comment(lib,"opencv_core470.lib")
//#pragma comment(lib,"opencv_highgui470.lib")
//#pragma comment(lib,"opencv_imgcodecs470.lib")
//#pragma comment(lib,"opencv_imgproc470.lib")  
//#pragma comment(lib,"opencv_features2d470.lib")
//#pragma comment(lib,"opencv_xfeatures2d470.lib")
//#pragma comment(lib,"opencv_flann470.lib")
//#pragma comment(lib,"zlib.lib") 
//#pragma comment(lib,"libjpeg-turbo.lib")
//#pragma comment(lib,"libopenjp2.lib") 
//#pragma comment(lib,"libtiff.lib")
//#pragma comment(lib,"libpng.lib")
//#pragma comment(lib,"IlmImf.lib")
//#pragma comment(lib,"libwebp.lib")
//#pragma comment(lib,"ittnotify.lib")
#pragma comment(lib,"dlib19.24.0_release_64bit_msvc1929.lib")
#endif // _DEBUG
#pragma comment(lib,"glog.lib")

struct Landmarks
{
    std::vector<std::vector<double>>landmarks;
	template <class Archive>
	void serialize(Archive& ar)
	{
		ar(cereal::make_nvp("landMarks", landmarks));
	}
};
static std::vector<std::string> splitString(const std::string& src, const std::string& symbols, bool repeat)
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
std::string deleteJsonShell(const std::string& json)
{
	std::vector<int>fronts;
	std::vector<int>backs;
	for (int i = 0; i < json.length(); i++)
	{
		if (json[i] == '{')
		{
			fronts.emplace_back(i);
			continue;
		}
		if (json[i] == '}')
		{
			backs.emplace_back(i);
			continue;
		}
	}
	int start = fronts[1];
	int end = backs[backs.size() - 2];
	std::string retJson = json.substr(start, end - start + 1);
	auto it = retJson.begin();
	while (it != retJson.end()) {
		if (*it == '\r' || *it == '\n') {
			it = retJson.erase(it);
		}
		else {
			++it;
		}
	}
	return retJson;
}
//std::string changeTail(const std::string&path, const std::string& newTail)
std::string landmarkDataPath = "D:/ucl360/libraries2019/dlib1924/shape_predictor_68_face_landmarks.dat";
 
std::pair<std::vector<std::string>, std::vector<std::string>>readFromParam(const std::string& path)
{
	std::vector<std::string> segs = splitString(path, ".", true);
	if (segs.size()>1 && segs[segs.size()-1].compare("txt") == 0)
	{
		std::vector<std::string>imgsPath;
		std::vector<std::string>jsonPath; 
		std::string aline;
		std::fstream fin(path,std::ios::in);
		while (std::getline(fin,aline))
		{
			imgsPath.emplace_back(aline);
			std::getline(fin, aline);
			jsonPath.emplace_back(aline);
		}
		return std::make_pair(imgsPath, jsonPath);
	}
	else
	{
		CHECK(false);
	}
	
}
int main(int argc, char** argv)
{
	std::vector<dlib::file> files;
	std::string jsonRoot;
	if (argc<2)
	{
		std::cout<<"cmd picPath landmarksDir" << std::endl;
		return -1;
	}
	try
	{
 
		dlib::directory picPath(argv[1]);
		dlib::directory jsonPath(argv[2]);
		
		std::cout << "directory: " << picPath.name() << std::endl;
		std::cout << "full path: " << picPath.full_name() << std::endl;
		std::cout << "is root:   " << ((picPath.is_root()) ? "yes" : "no") << std::endl;
		jsonRoot = jsonPath.full_name();
		std::vector<dlib::directory> dirs = picPath.get_dirs();
		files = picPath.get_files();

		// sort the files and directories
		sort(files.begin(), files.end());
		sort(dirs.begin(), dirs.end());
	}
	catch (dlib::file::file_not_found& e)
	{
		std::cout << "file not found or accessible: " << e.info << std::endl;
	}
	catch (dlib::directory::dir_not_found& e)
	{
		std::cout << "dir not found or accessible: " << e.info << std::endl;
	}
	catch (dlib::directory::listing_error& e)
	{
		std::cout << "listing error: " << e.info << std::endl;
	}
    dlib::frontal_face_detector detector = dlib::get_frontal_face_detector();
    dlib::shape_predictor sp;
    dlib::deserialize(landmarkDataPath.c_str()) >> sp; 
	for (int imgIdx = 0; imgIdx < files.size(); imgIdx++)
    {
		const std::string& imgPath = files[imgIdx];
		std::string imgName = files[imgIdx].name();
		if (imgName.length() <= 4)continue;
		std::string tail = imgPath.substr(imgPath.length() - 4);
		if (tail.compare(".jpg") != 0)continue; 
		const std::string& jsonPath = jsonRoot + "/" + imgName + ".json";
        std::cout << "processing image " << imgPath << std::endl;
        dlib::array2d<dlib::rgb_pixel> img;
        dlib::load_image(img, imgPath); 
        //dlib::pyramid_up(img); 
        std::vector<dlib::rectangle> dets = detector(img); 
		if (dets.size() != 1)
		{
			std::cout << "Number of faces detected: " << dets.size() << " => " << imgPath << std::endl;
			continue;
		} 
        dlib::full_object_detection shape = sp(img, dets[0]);    
		Landmarks thisImgLandmark;
		thisImgLandmark.landmarks.resize(shape.num_parts());
        for (int i = 0; i < shape.num_parts(); i++)
        {
			thisImgLandmark.landmarks[i] = std::vector<double>{ static_cast<double>(shape.part(i).x()),static_cast<double>(shape.part(i).y()) };
        }
		std::stringstream os;
		{
			cereal::JSONOutputArchive archive_out(os);
			archive_out(CEREAL_NVP(thisImgLandmark));
		} 
		std::string json_str = deleteJsonShell(os.str()); 
		std::fstream fout(jsonPath,std::ios::out);
		fout << json_str << std::endl;
    }

	return 0;
}