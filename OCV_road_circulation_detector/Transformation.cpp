#include "Transformation.h"

Transformation::Transformation()
{
	orb = ORB::create();
	matcher = DescriptorMatcher::create("BruteForce-Hamming");
	stereoBm = StereoBM::create();
}

Mat Transformation::readImgFromPath(string path, int& outX, int& outY)
{
	Mat img = imread(path);

	if (img.data)
	{
		if (outX == 0 && outY == 0)
		{
			outX = img.cols;
			outY = img.rows;
		}

		resize(img, img, Size(outX, outY));
		cvtColor(img, img, COLOR_BGR2GRAY);
	}

	return img;
}

vector<Mat> Transformation::computeInterestingPoints(Mat& img, Mat& prevImg)
{
	vector<Point2f> cornerA, cornerB;

	vector<uchar> currentStatus = Detection::findMatchings(img, prevImg, cornerA, cornerB);
	vector<uchar> prevStatus = Detection::findMatchings(prevImg, img, cornerB, cornerA);

	Mat match1, match2;

	img.copyTo(match1);
	prevImg.copyTo(match2);

	Detection::displayMatchings(match1, match2, cornerA, cornerB, currentStatus, prevStatus);

	return vector<Mat> {match1, match2};
}

Mat Transformation::computeMatchings(Mat& img, Mat& prevImg)
{
	vector<KeyPoint> keypoints_current, keypoints_prev;
	Mat descriptor_current, descriptor_prev;
	
	orb->detectAndCompute(img, noArray(), keypoints_current, descriptor_current);
	orb->detectAndCompute(prevImg, noArray(), keypoints_prev, descriptor_prev);
	vector<DMatch> matches;

	matcher->match(descriptor_current, descriptor_prev, matches);

	Mat outImg;
	drawMatches(img, keypoints_current, prevImg, keypoints_prev, matches, outImg);

	return outImg;
}

Mat Transformation::computeDisparity(Mat& img1, Mat& img2)
{
	Mat disparity;

	stereoBm->compute(img1, img2, disparity);

	double min;
	double max;

	minMaxLoc(disparity, &min, &max);

	double coef = 255.0f / (max - min);
	double offset = (-min) * 255.0f / (max - min);
	disparity.convertTo(disparity, CV_8U, coef, offset);

	return disparity;
}