#pragma once

#include "Detection.h"
#include "Constants.h"

class Transformation
{
private:
	Ptr<ORB> orb;
	Ptr<DescriptorMatcher> matcher;
	Ptr<StereoBM> stereoBm;
	Mat t, r;
	Mat rCopy, tCopy;

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
	void detectVector(Mat& img, Mat& prevImg);
	void getMoyVector(Mat& img, Mat& prevImg, Point2f& point1, Point2f& point2);

	void loadScale();
};