#ifndef _SELECT_VIEWS_H_
#define _SELECT_VIEWS_H_

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
#include "common.h"
#include "utils.h"
#include "types.h"
//std::vector<Camera>cameras = Camera::readCameras(cameraTXT);
//std::map<int, ImageData>imgs = ImageData::readImageData(imagesTXT);
//std::map<int, Point3dData>objPts = Point3dData::readPoint3dData(points3DTXT);
// compute visibility for the reference image
// and select the best views for reconstructing the dense point-cloud;
// extract also all 3D points seen by the reference image;
// (inspired by: "Multi-View Stereo for Community Photo Collections", Goesele, 2007)
//  - nInsideROI: 0 - ignore ROI, 1 - weight more ROI points, 2 - consider only ROI points
bool SelectNeighborViews(const std::vector<Camera>&cameras, 
	std::map<int, ImageData>&imgs,
	const std::map<int, Point3dData>&objPts,
	unsigned nMinViews = 3, unsigned nMinPointViews = 2, float fOptimAngle = FD2R(12), unsigned nInsideROI = 1);

#endif // !_SELECT_VIEWS_H_