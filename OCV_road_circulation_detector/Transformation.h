#pragma once

#include "Detection.h"

class Transformation
{
private:
	Ptr<ORB> orb;
	Ptr<DescriptorMatcher> matcher;
	Ptr<StereoBM> stereoBm;
public:
	Transformation();

	Mat readImgFromPath(string path, int& outX, int& outY);
	
	vector<Mat> computeInterestingPoints(Mat& img, Mat& prevImg);
	Mat computeMatchings(Mat& img, Mat& prevImg);
	Mat computeDisparity(Mat& img1, Mat& img2);
};