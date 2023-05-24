#include "gms_matcher.h"

//#define USE_GPU 
#ifdef USE_GPU
#include <opencv2/cudafeatures2d.hpp>
using cuda::GpuMat;
#endif

namespace gms
{
	int gms_matcher::GetInlierMask(vector<bool>& vbInliers, bool WithScale, bool WithRotation) {

		int max_inlier = 0;

		if (!WithScale && !WithRotation)
		{
			SetScale(0);
			max_inlier = run(1);
			vbInliers = mvbInlierMask;
			return max_inlier;
		}

		if (WithRotation && WithScale)
		{
			for (int Scale = 0; Scale < 5; Scale++)
			{
				SetScale(Scale);
				for (int RotationType = 1; RotationType <= 8; RotationType++)
				{
					int num_inlier = run(RotationType);

					if (num_inlier > max_inlier)
					{
						vbInliers = mvbInlierMask;
						max_inlier = num_inlier;
					}
				}
			}
			return max_inlier;
		}

		if (WithRotation && !WithScale)
		{
			SetScale(0);
			for (int RotationType = 1; RotationType <= 8; RotationType++)
			{
				int num_inlier = run(RotationType);

				if (num_inlier > max_inlier)
				{
					vbInliers = mvbInlierMask;
					max_inlier = num_inlier;
				}
			}
			return max_inlier;
		}

		if (!WithRotation && WithScale)
		{
			for (int Scale = 0; Scale < 5; Scale++)
			{
				SetScale(Scale);

				int num_inlier = run(1);

				if (num_inlier > max_inlier)
				{
					vbInliers = mvbInlierMask;
					max_inlier = num_inlier;
				}

			}
			return max_inlier;
		}

		return max_inlier;
	}

	void gms_matcher::AssignMatchPairs(int GridType) {

		for (size_t i = 0; i < mNumberMatches; i++)
		{
			Point2f& lp = mvP1[mvMatches[i].first];
			Point2f& rp = mvP2[mvMatches[i].second];

			int lgidx = mvMatchPairs[i].first = GetGridIndexLeft(lp, GridType);
			int rgidx = -1;

			if (GridType == 1)
			{
				rgidx = mvMatchPairs[i].second = GetGridIndexRight(rp);
			}
			else
			{
				rgidx = mvMatchPairs[i].second;
			}

			if (lgidx < 0 || rgidx < 0)	continue;

			mMotionStatistics.at<int>(lgidx, rgidx)++;
			mNumberPointsInPerCellLeft[lgidx]++;
		}

	}

	void gms_matcher::VerifyCellPairs(int RotationType) {

		const int* CurrentRP = mRotationPatterns[RotationType - 1];

		for (int i = 0; i < mGridNumberLeft; i++)
		{
			if (sum(mMotionStatistics.row(i))[0] == 0)
			{
				mCellPairs[i] = -1;
				continue;
			}

			int max_number = 0;
			for (int j = 0; j < mGridNumberRight; j++)
			{
				int* value = mMotionStatistics.ptr<int>(i);
				if (value[j] > max_number)
				{
					mCellPairs[i] = j;
					max_number = value[j];
				}
			}

			int idx_grid_rt = mCellPairs[i];

			const int* NB9_lt = mGridNeighborLeft.ptr<int>(i);
			const int* NB9_rt = mGridNeighborRight.ptr<int>(idx_grid_rt);

			int score = 0;
			double thresh = 0;
			int numpair = 0;

			for (size_t j = 0; j < 9; j++)
			{
				int ll = NB9_lt[j];
				int rr = NB9_rt[CurrentRP[j] - 1];
				if (ll == -1 || rr == -1)	continue;

				score += mMotionStatistics.at<int>(ll, rr);
				thresh += mNumberPointsInPerCellLeft[ll];
				numpair++;
			}

			thresh = THRESH_FACTOR * sqrt(thresh / numpair);

			if (score < thresh)
				mCellPairs[i] = -2;
		}
	}

	int gms_matcher::run(int RotationType) {

		mvbInlierMask.assign(mNumberMatches, false);

		// Initialize Motion Statisctics
		mMotionStatistics = Mat::zeros(mGridNumberLeft, mGridNumberRight, CV_32SC1);
		mvMatchPairs.assign(mNumberMatches, pair<int, int>(0, 0));

		for (int GridType = 1; GridType <= 4; GridType++)
		{
			// initialize
			mMotionStatistics.setTo(0);
			mCellPairs.assign(mGridNumberLeft, -1);
			mNumberPointsInPerCellLeft.assign(mGridNumberLeft, 0);

			AssignMatchPairs(GridType);
			VerifyCellPairs(RotationType);

			// Mark inliers
			for (size_t i = 0; i < mNumberMatches; i++)
			{
				if (mvMatchPairs[i].first >= 0) {
					if (mCellPairs[mvMatchPairs[i].first] == mvMatchPairs[i].second)
					{
						mvbInlierMask[i] = true;
					}
				}
			}
		}
		int num_inlier = sum(mvbInlierMask)[0];
		return num_inlier;
	}

	void runImagePair() {
		Mat img1 = imread("../data/01.jpg");
		Mat img2 = imread("../data/02.jpg");

		GmsMatch(img1, img2);
	}



	void GmsMatch(Mat& img1, Mat& img2) {
		vector<KeyPoint> kp1, kp2;
		Mat d1, d2;
		vector<DMatch> matches_all, matches_gms;

		Ptr<ORB> orb = ORB::create(10000);
		orb->setFastThreshold(0);

		orb->detectAndCompute(img1, Mat(), kp1, d1);
		orb->detectAndCompute(img2, Mat(), kp2, d2);

#ifdef USE_GPU
		GpuMat gd1(d1), gd2(d2);
		Ptr<cuda::DescriptorMatcher> matcher = cv::cuda::DescriptorMatcher::createBFMatcher(NORM_HAMMING);
		matcher->match(gd1, gd2, matches_all);
#else
		BFMatcher matcher(NORM_HAMMING);
		matcher.match(d1, d2, matches_all);
#endif

		// GMS filter
		std::vector<bool> vbInliers;
		gms_matcher gms(kp1, img1.size(), kp2, img2.size(), matches_all);
		int num_inliers = gms.GetInlierMask(vbInliers, false, false);
		cout << "Get total " << num_inliers << " matches." << endl;

		// collect matches
		for (size_t i = 0; i < vbInliers.size(); ++i)
		{
			if (vbInliers[i] == true)
			{
				matches_gms.push_back(matches_all[i]);
			}
		}

		// draw matching
		Mat show = DrawInlier(img1, img2, kp1, kp2, matches_gms, 1);
		imshow("show", show);
		waitKey();
	}

	Mat DrawInlier(Mat& src1, Mat& src2, vector<KeyPoint>& kpt1, vector<KeyPoint>& kpt2, vector<DMatch>& inlier, int type) {
		const int height = max(src1.rows, src2.rows);
		const int width = src1.cols + src2.cols;
		Mat output(height, width, CV_8UC3, Scalar(0, 0, 0));
		src1.copyTo(output(Rect(0, 0, src1.cols, src1.rows)));
		src2.copyTo(output(Rect(src1.cols, 0, src2.cols, src2.rows)));

		if (type == 1)
		{
			for (size_t i = 0; i < inlier.size(); i++)
			{
				Point2f left = kpt1[inlier[i].queryIdx].pt;
				Point2f right = (kpt2[inlier[i].trainIdx].pt + Point2f((float)src1.cols, 0.f));
				line(output, left, right, Scalar(0, 255, 255));
			}
		}
		else if (type == 2)
		{
			for (size_t i = 0; i < inlier.size(); i++)
			{
				Point2f left = kpt1[inlier[i].queryIdx].pt;
				Point2f right = (kpt2[inlier[i].trainIdx].pt + Point2f((float)src1.cols, 0.f));
				line(output, left, right, Scalar(255, 0, 0));
			}

			for (size_t i = 0; i < inlier.size(); i++)
			{
				Point2f left = kpt1[inlier[i].queryIdx].pt;
				Point2f right = (kpt2[inlier[i].trainIdx].pt + Point2f((float)src1.cols, 0.f));
				circle(output, left, 1, Scalar(0, 255, 255), 2);
				circle(output, right, 1, Scalar(0, 255, 0), 2);
			}
		}

		return output;
	}
}