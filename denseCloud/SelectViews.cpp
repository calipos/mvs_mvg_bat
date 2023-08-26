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


// keep only the best neighbors for the reference image
bool FilterNeighborViews(std::vector<ViewScore>& neighbors, float fMinArea = 0.1f, float fMinScale = 0.2f, float fMaxScale = 2.4f, float fMinAngle = FD2R(3), float fMaxAngle = FD2R(45), unsigned nMaxViews = 12)
{
	// remove invalid neighbor views
	const unsigned nMinViews(std::max(4u, nMaxViews * 3 / 4));
	auto iter = neighbors.rbegin();
	while (iter != neighbors.rend())
	{
		const ViewScore& neighbor = *iter;
		if (neighbors.size() > nMinViews &&
			(neighbor.idx.area < fMinArea ||
				!ISINSIDE(neighbor.idx.scale, fMinScale, fMaxScale) ||
				!ISINSIDE(neighbor.idx.angle, fMinAngle, fMaxAngle)))
		{ 
			iter = decltype(iter)(neighbors.erase(std::next(iter).base()));
		}
		else
		{
			iter++;
		}
	} 
	if (neighbors.size() > nMaxViews)
		neighbors.resize(nMaxViews);
	return neighbors.size()!=0;
}

// compute visibility for the reference image
// and select the best views for reconstructing the dense point-cloud;
// extract also all 3D points seen by the reference image;
// (inspired by: "Multi-View Stereo for Community Photo Collections", Goesele, 2007)
//  - nInsideROI: 0 - ignore ROI, 1 - weight more ROI points, 2 - consider only ROI points
bool SelectNeighborViews(const std::vector<Camera>& cameras,
	std::map<int, ImageData>& imgs,
	const std::map<int, Point3dData>& objPts,
	unsigned nMinViews, unsigned nMinPointViews, float fOptimAngle, unsigned nInsideROI)
{
	unsigned nCalibratedImages = imgs.size();
	for (auto&img: imgs)
	{
		unsigned nPoints = 0;
		dType avgDepth = 0;
		///extract also all 3D points seen by the reference image;
		std::vector<int>pointsID;
		int imgID = img.first;
		std::vector<ViewScore>& neighbors = img.second.neighbors;
		CHECK(neighbors.size()==0);
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
		std::vector<cv::Point_<dType>>projs;
		projs.reserve(objPts.size());
		if (neighbors.size() == 0)
		{
			
			for (const auto& img : imgs)
			{
				const int& ID_B = img.second.imageId;
				const Score& score = scores[ID_B];
				if (score.points < 3)
					continue;
				CHECK(imgID != ID_B);
				// compute how well the matched features are spread out (image covered area)
				const cv::Point2f boundsA(cameras.at(imgs.at(imgID).cameraId).width, cameras.at(imgs.at(imgID).cameraId).height);
				const cv::Point2f boundsB(cameras.at(imgs.at(ID_B).cameraId).width, cameras.at(imgs.at(ID_B).cameraId).height);
				for (const auto& idx : pointsID)
				{
					objPts.at(idx).tracks_imgId_imgPtId;
					CHECK(objPts.at(idx).tracks_imgId_imgPtId.count(imgID));
					if (objPts.at(idx).tracks_imgId_imgPtId.count(ID_B) == 0)
						continue;
					const cv::Point3_<dType>& point = objPts.at(idx).objPt;
					cv::Point_<dType> ptA = cameras.at(imgs.at(imgID).cameraId).ptInView(imgs.at(imgID).worldPtInView(point));
					cv::Point_<dType> ptB = cameras.at(imgs.at(ID_B).cameraId).ptInView(imgs.at(ID_B).worldPtInView(point));
					//if (!imageData.camera.IsInside(ptA, boundsA) || !imageDataB.camera.IsInside(ptB, boundsB)) 
					if (IsInside<dType>(ptA, boundsA) && IsInside<dType>(ptB, boundsB))
					{
						projs.emplace_back(ptA);
						//std::cout << point << ptA << std::endl;
					}
				}
				CHECK(projs.size() <= score.points);
				if (projs.empty())				continue;
				const float area(ComputeCoveredArea<dType, 2, 16, false>(projs, boundsA));
				projs.clear();
				// store image score
				ViewScore neighbor;
				neighbor.idx.ID = ID_B;
				neighbor.idx.points = score.points;
				neighbor.idx.scale = score.avgScale / score.points;
				neighbor.idx.angle = score.avgAngle / score.points;
				neighbor.idx.area = area;
				neighbor.score = score.score * std::max(area, 0.01f);
				neighbors.emplace_back(neighbor);
			}
			{
				std::sort(neighbors.begin(), neighbors.end(), [](const auto& a, const auto& b) {return a.score > b.score; });
				std::stringstream msg;
				msg << "\nReference image " << imgID << " sees " << neighbors.size() << " views : ";
				for (const auto&n: neighbors)
				{
					msg << n.idx.ID << "(" << n.idx.points << "," << n.idx.scale << ")  ";
				}
				msg << "(" << nPoints << "  shared points)";
				std::cout << msg.str() << std::endl;
			}
			
		}
		if (pointsID.size() <= 3 || neighbors.size() < std::min(nMinViews, nCalibratedImages - 1))
		{
			std::cout << "error: reference image " << imgID <<" has not enough images in view " << std::endl;;
			return false;
		}
#define __fMinArea__ (0.05)
#define __fMinAngle__ (3)
#define __fMaxAngle__ (65)
#define __nMaxViews__ (12)
		const float fMinArea(__fMinArea__);
		const float fMinScale(0.2f), fMaxScale(3.2f);
		const float fMinAngle(FD2R(__fMinAngle__));
		const float fMaxAngle(FD2R(__fMaxAngle__));
		FilterNeighborViews(neighbors, fMinArea, fMinScale, fMaxScale, fMinAngle, fMaxAngle, __nMaxViews__);
	}
	return true;
}