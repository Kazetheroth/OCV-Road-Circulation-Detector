#pragma once

#include "Detection.h"
#include "Constants.h"

class Transformation
{
private:
	Ptr<ORB> orb;
	Ptr<DescriptorMatcher> matcher;
	Ptr<StereoBM> stereoBm;
	Mat t, recover;
	Mat recoverCopy, tCopy;

	string pathSelected;
	
	double getAbsoluteScale(int frameId);
	vector<double> scales;

public:
	Transformation();

	void setPath(string path);
	
	Mat readImgFromPath(string path, int& outX, int& outY);
	
	vector<Mat> computeInterestingPoints(Mat& img, Mat& prevImg);
	Mat computeMatchings(Mat& img, Mat& prevImg);
	Mat computeDisparity(Mat& img1, Mat& img2);
	Mat computeTrajectoryPoints(Mat& img, Mat& prevImg);
	void drawTrajectory(Mat& img1, Mat& img2, int idx, Mat& traj);

	void loadScale();
};