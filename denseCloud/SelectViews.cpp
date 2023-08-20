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
dType Footprint(const Camera& camera, const ImageData&img, const cv::Point3_<dType>& X)
{
	return (dType)(camera.focalLength / img.worldPtDepth(X));
}
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
			const auto& trace = objPt.second.tracks_imgId_imgPtId;
			if (trace.count(imgID)==0)
			{
				continue;
			}
			float wROI(1.f);
			dType depth = img.second.worldPtDepth(objPt.second.objPt);
			if (depth <= 0)				continue;
			if (trace.size()> nMinPointViews)
			{
				pointsID.emplace_back(thisObjPtID);
			}
			avgDepth += depth;
			++nPoints;
			// score shared views
			const cv::Point3_<dType> V1(eigen2pt(img.second.camera_t) - objPt.second.objPt);
			const float footprint1(Footprint(cameras[img.second.cameraId], img.second, objPt.second.objPt));
			for (const auto&view : imgs)
			{
				if (view.second.imageId == imgID)
					continue;
				const cv::Point3_<dType> V2(eigen2pt(view.second.camera_t) - objPt.second.objPt);
				const float fAngle(ACOS(ComputeAngle(V1, V2)));
				const float wAngle(EXP(SQUARE(fAngle - fOptimAngle)* (fAngle < fOptimAngle ? sigmaAngleSmall : sigmaAngleLarge)));
				const float footprint2(Footprint(cameras[view.second.cameraId], view.second, objPt.second.objPt));
				const float fScaleRatio(footprint1 / footprint2);
				float wScale;
				if (fScaleRatio > 1.6f)
					wScale = SQUARE(1.6f / fScaleRatio);
				else if (fScaleRatio >= 1.f)
					wScale = 1.f;
				else
					wScale = SQUARE(fScaleRatio);
				Score& score = scores[view.second.imageId];
				score.score += std::max(wAngle, 0.1f) * wScale * wROI;
				score.avgScale += fScaleRatio;
				score.avgAngle += fAngle;
				++score.points;
			}
		}
		avgDepth /= nPoints;
		CHECK(nPoints > 3);

		// select best neighborViews
		std::vector<cv::Point_<dType>>projs(objPts.size());
		for (const auto&img:imgs)
		{
			const int& ID_B = img.second.imageId;
			const Score& score = scores[ID_B];
			if (score.points < 3)
				continue;
			CHECK(imgID != ID_B);
			// compute how well the matched features are spread out (image covered area)
			const cv::Point2f boundsA(cameras.at(imgs.at(imgID).cameraId).width, cameras.at(imgs.at(imgID).cameraId).height);
			const cv::Point2f boundsB(cameras.at(imgs.at(ID_B).cameraId).width, cameras.at(imgs.at(ID_B).cameraId).height);
			for (const auto&idx:pointsID)
			{
				CHECK(imgs.count(imgID));
				if (imgs.count(imgID) == 0)continue;
				const cv::Point3_<dType>& point = objPts.at(idx).objPt;
				cv::Point3_<dType>& ptA = projs.emplace_back(imageData.camera.ProjectPointP(point));
				cv::Point3_<dType> ptB = imageDataB.camera.ProjectPointP(point);
				if (!imageData.camera.IsInside(ptA, boundsA) || !imageDataB.camera.IsInside(ptB, boundsB))
					projs.RemoveLast();
			}
		}
	}
	return true;
}