#include <iostream>
#include <set>
#include <vector>
#include <fstream>
#include <string> 
#include <sstream> 
#include "glog/logging.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "opencv2/opencv.hpp"
#include "omp.h"
#include "SelectViews.h"
#include "types.h"
#include "utils.h"
// compute visibility for the reference image
// and select the best views for reconstructing the dense point-cloud;
// extract also all 3D points seen by the reference image;
// (inspired by: "Multi-View Stereo for Community Photo Collections", Goesele, 2007)
//  - nInsideROI: 0 - ignore ROI, 1 - weight more ROI points, 2 - consider only ROI points
bool SelectNeighborViews(const std::vector<Camera>& cameras,
	const std::map<int, ImageData>& imgs,
	const std::map<int, Point3dData>& objPts,
	unsigned nMinViews, unsigned nMinPointViews, float fOptimAngle, unsigned nInsideROI)
{
	int nCalibratedImages = imgs.size();
	for (const auto&img: imgs)
	{
		unsigned nPoints = 0;
		dType avgDepth = 0;
		///extract also all 3D points seen by the reference image;
		std::vector<int>pointsID;
		int imgID = img.first;
		struct Score {
			float score;
			float avgScale;
			float avgAngle;
			uint32_t points;
		};
		std::vector<Score> scores(imgs.size());
		memset(&scores[0], 0, imgs.size() * sizeof(Score));
		if (nMinPointViews > nCalibratedImages)
			nMinPointViews = nCalibratedImages;
		const float sigmaAngleSmall(-1.f / (2.f * SQUARE(fOptimAngle * 0.38f)));
		const float sigmaAngleLarge(-1.f / (2.f * SQUARE(fOptimAngle * 0.7f)));
		for (const auto&objPt:objPts)
		{
			const auto& thisObjPtID = objPt.first;
			const auto& trace = objPt.second.tracks;
			if (trace.end()==std::find(trace.begin(), trace.end(), imgID))
			{
				continue;
			}
			dType depth = img.second.worldPtDepth(objPt.second.objPt);
			if (depth <= 0)				continue;
			if (trace.size()> nMinPointViews)
			{
				pointsID.emplace_back(thisObjPtID);
			}
			avgDepth += depth;
			++nPoints;
			// score shared views
			const cv::Point3_<dType> V1(eigen2pt(img.second.t) - objPt.second.objPt);
			const float footprint1(imageData.camera.GetFootprintImage(point));
		}
	}
	return true;
}