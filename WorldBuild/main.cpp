#include "fivePtsFigureRt.h"
#include "opencv2/opencv.hpp"
#include "opencv2/xfeatures2d.hpp" 
#include "gms_matcher.h"
#include "glog/logging.h"
#include "Eigen/Core"
#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <string>
#include <tuple>
#ifdef _DEBUG
#pragma comment(lib,"opencv_core470d.lib")
#pragma comment(lib,"opencv_highgui470d.lib")
#pragma comment(lib,"opencv_imgcodecs470d.lib")
#pragma comment(lib,"opencv_imgproc470d.lib")  
#pragma comment(lib,"opencv_calib3d470d.lib")  
#pragma comment(lib,"opencv_features2d470d.lib")
#pragma comment(lib,"opencv_xfeatures2d470d.lib")
#pragma comment(lib,"opencv_flann470d.lib")
#pragma comment(lib,"zlibd.lib") 
#pragma comment(lib,"libjpeg-turbod.lib")
#pragma comment(lib,"libopenjp2d.lib") 
#pragma comment(lib,"libtiffd.lib")
#pragma comment(lib,"libpngd.lib")
#pragma comment(lib,"IlmImfd.lib")
#pragma comment(lib,"libwebpd.lib")
#pragma comment(lib,"ittnotifyd.lib")
#else
#pragma comment(lib,"opencv_core470.lib")
#pragma comment(lib,"opencv_highgui470.lib")
#pragma comment(lib,"opencv_imgcodecs470.lib")
#pragma comment(lib,"opencv_imgproc470.lib")  
#pragma comment(lib,"opencv_features2d470.lib")
#pragma comment(lib,"opencv_xfeatures2d470.lib")
#pragma comment(lib,"opencv_flann470.lib")
#pragma comment(lib,"zlib.lib") 
#pragma comment(lib,"libjpeg-turbo.lib")
#pragma comment(lib,"libopenjp2.lib") 
#pragma comment(lib,"libtiff.lib")
#pragma comment(lib,"libpng.lib")
#pragma comment(lib,"IlmImf.lib")
#pragma comment(lib,"libwebp.lib")
#pragma comment(lib,"ittnotify.lib")
#endif // _DEBUG
#pragma comment(lib,"glog.lib")
 
std::tuple<cv::Mat, cv::Mat, cv::Mat> tripletSiftMatch(const std::string& path1, const std::string& path2, const std::string& path3)
{
	LOG(INFO)<<123;
	cv::Mat img1 = cv::imread(path1);
	cv::Mat img2 = cv::imread(path2);
	cv::Mat img3 = cv::imread(path3);
	cv::Ptr<cv::SIFT>detector = cv::SIFT::create(2000);//与SURF一样，剩余的取默认值
	std::vector<cv::KeyPoint>kp1, kp2, kp3; 
	cv::Mat d1, d2,d3;
	detector->detectAndCompute(img1, cv::Mat(), kp1, d1);
	detector->detectAndCompute(img2, cv::Mat(), kp2, d2);
	detector->detectAndCompute(img3, cv::Mat(), kp3, d3);
	//cv::drawKeypoints(img1, kp1, resultImg, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT); 
	std::vector<cv::DMatch> matches_12, matches_12_gms;
	std::vector<cv::DMatch> matches_13, matches_13_gms;
	std::vector<cv::DMatch> matches_23, matches_23_gms;
#ifdef USE_GPU
	GpuMat gd1(d1), gd2(d2);
	Ptr<cuda::DescriptorMatcher> matcher = cv::cuda::DescriptorMatcher::createBFMatcher(NORM_HAMMING);
	matcher->match(gd1, gd2, matches_all);
#else
	cv::BFMatcher matcher12(cv::NORM_L2);
	matcher12.match(d1, d2, matches_12);
	cv::BFMatcher matcher13(cv::NORM_L2);
	matcher13.match(d1, d3, matches_13);
	cv::BFMatcher matcher23(cv::NORM_L2);
	matcher23.match(d2, d3, matches_23);
#endif
	 

	std::vector<bool> vbInliers12;
	gms::gms_matcher gms12(kp1, img1.size(), kp2, img2.size(), matches_12);
	int num_inliers12 = gms12.GetInlierMask(vbInliers12, false, false);
	std::vector<bool> vbInliers13;
	gms::gms_matcher gms13(kp1, img1.size(), kp3, img3.size(), matches_13);
	int num_inliers13 = gms13.GetInlierMask(vbInliers13, false, false);
	std::vector<bool> vbInliers23;
	gms::gms_matcher gms23(kp2, img2.size(), kp3, img3.size(), matches_23);
	int num_inliers23 = gms23.GetInlierMask(vbInliers23, false, false); 
	for (size_t i = 0; i < vbInliers12.size(); ++i)
		if (vbInliers12[i] == true)
			matches_12_gms.push_back(matches_12[i]);
	for (size_t i = 0; i < vbInliers13.size(); ++i)
		if (vbInliers13[i] == true)
			matches_13_gms.push_back(matches_13[i]);
	for (size_t i = 0; i < vbInliers23.size(); ++i)
		if (vbInliers23[i] == true)
			matches_23_gms.push_back(matches_23[i]);

	// draw matching
	cv::Mat show12 = gms::DrawInlier(img1, img2, kp1, kp2, matches_12_gms, 1);
	cv::Mat show13 = gms::DrawInlier(img1, img3, kp1, kp3, matches_13_gms, 1);
	cv::Mat show23 = gms::DrawInlier(img2, img3, kp2, kp3, matches_23_gms, 1);

 	return std::make_tuple(show12, show13, show23);
}
Eigen::Matrix3d cameraK(const double& fx, const double& fy, const double& ppx, const double& ppy)
{
	Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
	K(0, 0) = fx;
	K(1, 1) = fy;
	K(0, 2) = ppx;
	K(1, 2) = ppy;
	return K;
}
Eigen::MatrixXd pts2eigen(const std::vector < cv::Point3d >& pts)
{
	Eigen::MatrixXd ret(3,pts.size());
	for (int i = 0; i < pts.size(); i++)
	{
		ret(0, i) = pts[i].x;
		ret(1, i) = pts[i].y;
		ret(2, i) = pts[i].z;
	}
	return ret;
}
static int sgn(double val) { return (0 < val) - (val < 0); }

static void getPlanes(cv::InputArray points3d_, std::vector<int>& labels, std::vector<cv::Vec4d>& planes, int desired_num_planes, double thr_, double conf_, int max_iters_)
{
	using namespace cv;
	Mat points3d = points3d_.getMat();
	points3d.convertTo(points3d, CV_64F); // convert points to have double precision
	if (points3d_.isVector())
		points3d = Mat((int)points3d.total(), 3, CV_64F, points3d.data);
	else {
		if (points3d.type() != CV_64F)
			points3d = points3d.reshape(1, (int)points3d.total()); // convert point to have 1 channel
		if (points3d.rows < points3d.cols)
			transpose(points3d, points3d); // transpose so points will be in rows
		CV_CheckEQ(points3d.cols, 3, "Invalid dimension of point");
	}

	/*
	 * 3D plane fitting with RANSAC
	 * @best_model contains coefficients [a b c d] s.t. ax + by + cz = d
	 *
	 */
	auto plane_ransac = [](const Mat& pts, double thr, double conf, int max_iters, Vec4d& best_model, std::vector<bool>& inliers) {
		const int pts_size = pts.rows, max_lo_inliers = 15, max_lo_iters = 10;
		int best_inls = 0;
		if (pts_size < 3) return false;
		RNG rng;
		const auto* const points = (double*)pts.data;
		std::vector<int> min_sample(3);
		inliers = std::vector<bool>(pts_size);
		const double log_conf = log(1 - conf);
		Vec4d model, lo_model;
		std::vector<int> random_pool(pts_size);
		for (int p = 0; p < pts_size; p++)
			random_pool[p] = p;

		// estimate plane coefficients using covariance matrix
		auto estimate = [&](const std::vector<int>& sample, Vec4d& model_) {
			// https://www.ilikebigbits.com/2017_09_25_plane_from_points_2.html
			const int n = static_cast<int>(sample.size());
			if (n < 3) return false;
			double sum_x = 0, sum_y = 0, sum_z = 0;
			for (int s : sample) {
				sum_x += points[3 * s];
				sum_y += points[3 * s + 1];
				sum_z += points[3 * s + 2];
			}
			const double c_x = sum_x / n, c_y = sum_y / n, c_z = sum_z / n;
			double xx = 0, yy = 0, zz = 0, xy = 0, xz = 0, yz = 0;
			for (int s : sample) {
				const double x_ = points[3 * s] - c_x, y_ = points[3 * s + 1] - c_y, z_ = points[3 * s + 2] - c_z;
				xx += x_ * x_; yy += y_ * y_; zz += z_ * z_; xy += x_ * y_; yz += y_ * z_; xz += x_ * z_;
			}
			xx /= n; yy /= n; zz /= n; xy /= n; yz /= n; xz /= n;
			Vec3d weighted_normal(0, 0, 0);
			const double det_x = yy * zz - yz * yz, det_y = xx * zz - xz * xz, det_z = xx * yy - xy * xy;
			Vec3d axis_x(det_x, xz * xz - xy * zz, xy * yz - xz * yy);
			Vec3d axis_y(xz * yz - xy * zz, det_y, xy * xz - yz * xx);
			Vec3d axis_z(xy * yz - xz * yy, xy * xz - yz * xx, det_z);
			weighted_normal += axis_x * det_x * det_x;
			weighted_normal += sgn(weighted_normal.dot(axis_y)) * axis_y * det_y * det_y;
			weighted_normal += sgn(weighted_normal.dot(axis_z)) * axis_z * det_z * det_z;
			weighted_normal /= norm(weighted_normal);
			if (std::isinf(weighted_normal(0)) ||
				std::isinf(weighted_normal(1)) ||
				std::isinf(weighted_normal(2))) return false;
			// find plane model from normal and centroid
			model_ = Vec4d(weighted_normal(0), weighted_normal(1), weighted_normal(2),
				weighted_normal.dot(Vec3d(c_x, c_y, c_z)));
			return true;
		};

		// calculate number of inliers
		auto getInliers = [&](const Vec4d& model_) {
			const double a = model_(0), b = model_(1), c = model_(2), d = model_(3);
			int num_inliers = 0;
			std::fill(inliers.begin(), inliers.end(), false);
			for (int p = 0; p < pts_size; p++) {
				inliers[p] = fabs(a * points[3 * p] + b * points[3 * p + 1] + c * points[3 * p + 2] - d) < thr;
				if (inliers[p]) num_inliers++;
				if (num_inliers + pts_size - p < best_inls) break;
			}
			return num_inliers;
		};
		// main RANSAC loop
		for (int iters = 0; iters < max_iters; iters++) {
			// find minimal sample: 3 points
			min_sample[0] = rng.uniform(0, pts_size);
			min_sample[1] = rng.uniform(0, pts_size);
			min_sample[2] = rng.uniform(0, pts_size);
			if (!estimate(min_sample, model))
				continue;
			int num_inliers = getInliers(model);
			if (num_inliers > best_inls) {
				// store so-far-the-best
				std::vector<bool> best_inliers = inliers;
				// do Local Optimization
				for (int lo_iter = 0; lo_iter < max_lo_iters; lo_iter++) {
					std::vector<int> inliers_idx; inliers_idx.reserve(max_lo_inliers);
					randShuffle(random_pool);
					for (int p : random_pool) {
						if (best_inliers[p]) {
							inliers_idx.emplace_back(p);
							if ((int)inliers_idx.size() >= max_lo_inliers)
								break;
						}
					}
					if (!estimate(inliers_idx, lo_model))
						continue;
					int lo_inls = getInliers(lo_model);
					if (best_inls < lo_inls) {
						best_model = lo_model;
						best_inls = lo_inls;
						best_inliers = inliers;
					}
				}
				if (best_inls < num_inliers) {
					best_model = model;
					best_inls = num_inliers;
				}
				// update max iters
				// because points are quite noisy we need more iterations
				const double max_hyp = 3 * log_conf / log(1 - pow(double(best_inls) / pts_size, 3));
				if (!std::isinf(max_hyp) && max_hyp < max_iters)
					max_iters = static_cast<int>(max_hyp);
			}
		}
		getInliers(best_model);
		return best_inls != 0;
	};

	labels = std::vector<int>(points3d.rows, 0);
	Mat pts3d_plane_fit = points3d.clone();
	// keep array of indices of points corresponding to original points3d
	std::vector<int> to_orig_pts_arr(pts3d_plane_fit.rows);
	for (int i = 0; i < (int)to_orig_pts_arr.size(); i++)
		to_orig_pts_arr[i] = i;
	for (int num_planes = 1; num_planes <= desired_num_planes; num_planes++) {
		Vec4d model;
		std::vector<bool> inl;
		if (!plane_ransac(pts3d_plane_fit, thr_, conf_, max_iters_, model, inl))
			break;
		planes.emplace_back(model);

		const int pts3d_size = pts3d_plane_fit.rows;
		pts3d_plane_fit = Mat();
		pts3d_plane_fit.reserve(points3d.rows);

		int cnt = 0;
		for (int p = 0; p < pts3d_size; p++) {
			if (!inl[p]) {
				// if point is not inlier to found plane - add it to next run
				to_orig_pts_arr[cnt] = to_orig_pts_arr[p];
				pts3d_plane_fit.push_back(points3d.row(to_orig_pts_arr[cnt]));
				cnt++;
			}
			else labels[to_orig_pts_arr[p]] = num_planes; // otherwise label this point
		}
	}
}

int main()
{
	cv::Mat image1 = cv::imread("im_0.jpg");
	cv::Mat image2 = cv::imread("im_22.jpg");
	CV_CheckEQ((int)image1.empty(), 0, "Image 1 is not found!");
	CV_CheckEQ((int)image2.empty(), 0, "Image 2 is not found!");

	std::vector<cv::Point3d>p1{ {308,419,1},{456,854,1},{615,879,1},{345,284,1},{680.5,377.5,1} };
	std::vector<cv::Point3d>p2{ {6,475,1},{224,912,1},{386,917,1},{75,328,1},{421,428.5,1} };



	Eigen::MatrixXd pts2d1 = pts2eigen(p1);
	Eigen::MatrixXd pts2d2 = pts2eigen(p2);

	Eigen::Matrix3d K1 = cameraK(2000,2000,720/2,1280/2);
	Eigen::Matrix3d K2 = cameraK(2000,2000,720/2,1280/2);

	Eigen::MatrixXd pts3d1 = K1.inverse() * pts2d1;
	Eigen::MatrixXd pts3d2 = K2.inverse() * pts2d2;

	std::cout << pts3d1 << std::endl;
	std::cout << pts3d2 << std::endl;
	std::cout  << std::endl;

	Eigen::Matrix3d model;
	rt::fivePtsFigureRt(pts3d1, pts3d2, K1, K2, model);
	std::cout<< model <<std::endl;

	{
		std::vector<cv::Point2d> pts1(5), pts2(5);
		cv::Mat points1(2, 5, CV_64FC1);
		cv::Mat points2(2, 5, CV_64FC1);
		for (int i = 0; i < 5; i++)
		{
			pts1[i].x = p1[i].x;
			pts1[i].y = p1[i].y;
			pts2[i].x = p2[i].x;
			pts2[i].y = p2[i].y;
			points1.ptr<double>(0)[i] =0.0001*( p1[i].x- 720 / 2);
			points1.ptr<double>(1)[i] =0.0001*( p1[i].y- 1280 / 2);
			points2.ptr<double>(0)[i] =0.0001*( p2[i].x- 720 / 2);
			points2.ptr<double>(1)[i] =0.0001*( p2[i].y- 1280 / 2);
		}
		cv::Mat K=cv::Mat::eye(3, 3, CV_64FC1);
		K.ptr<double>(0)[0] = 2000;
		K.ptr<double>(0)[2] = 720 / 2;
		K.ptr<double>(1)[1] = 2000;
		K.ptr<double>(1)[2] = 1280 / 2;
		cv::Mat R1, R2, t;
		cv::Mat E(3, 3, CV_64FC1);
		E.ptr<double>(0)[0] = model(0, 0);
		E.ptr<double>(0)[1] = model(0, 1);
		E.ptr<double>(0)[2] = model(0, 2);
		E.ptr<double>(1)[0] = model(1, 0);
		E.ptr<double>(1)[1] = model(1, 1);
		E.ptr<double>(1)[2] = model(1, 2);
		E.ptr<double>(2)[0] = model(2, 0);
		E.ptr<double>(2)[1] = model(2, 1);
		E.ptr<double>(2)[2] = model(2, 2);
		cv::decomposeEssentialMat(E, R1, R2, t);
		std::cout << R1 << std::endl;
		std::cout << R2 << std::endl;
		std::cout << t << std::endl;
		// Create two relative pose
		// P1 = K [  I    |   0  ]
		// P2 = K [R{1,2} | {+-}t]
		cv::Mat P1;
		cv::hconcat(K, cv::Vec3d::zeros(), P1);
		std::vector< cv::Mat> P2s(4);
		cv::hconcat(K * R1, K * t, P2s[0]);
		cv::hconcat(K * R1, -K * t, P2s[1]);
		cv::hconcat(K * R2, K * t, P2s[2]);
		cv::hconcat(K * R2, -K * t, P2s[3]);
		std::vector<std::vector<cv::Vec3d>> obj_pts_per_cam(4);
		// vector to keep indices of image points corresponding to object points
		std::vector<std::vector<int>> img_idxs_per_cam(4);
		int cam_idx = 0, best_cam_idx = 0, max_obj_pts = 0;
		for (const auto& P2 : P2s) {
			obj_pts_per_cam[cam_idx].reserve(5);
			img_idxs_per_cam[cam_idx].reserve(5);
			for (int i = 0; i < 5; i++) { 
				cv::Vec4d obj_pt;
				// find object point using triangulation
				triangulatePoints(P1, P2, points1.col(i), points2.col(i), obj_pt);
				obj_pt /= obj_pt(3); // normalize 4d point
				if (obj_pt(2) > 0) { // check if projected point has positive depth
					obj_pts_per_cam[cam_idx].emplace_back(cv::Vec3d(obj_pt(0), obj_pt(1), obj_pt(2)));
					img_idxs_per_cam[cam_idx].emplace_back(i);
				}
			}
			if (max_obj_pts < (int)obj_pts_per_cam[cam_idx].size()) {
				max_obj_pts = (int)obj_pts_per_cam[cam_idx].size();
				best_cam_idx = cam_idx;
			}
			cam_idx++;
		}

		std::cout << "Number of object points " << max_obj_pts << "\n";
		obj_pts_per_cam[best_cam_idx];
	}
	return 0;

	const std::string& path1="E:/viewer/im_21.jpg";
	const std::string& path2="E:/viewer/im_23.jpg";
	const std::string& path3="E:/viewer/im_25.jpg";
	cv::Mat img1, img2, img3;
	std::tie(img1, img2, img3) = tripletSiftMatch(path1, path2, path3);
	return 0;
}