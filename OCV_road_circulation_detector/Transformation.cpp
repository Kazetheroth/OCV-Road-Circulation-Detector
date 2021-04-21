#include "Transformation.h"

#include <fstream>

Transformation::Transformation()
{
	orb = ORB::create();
	matcher = DescriptorMatcher::create("BruteForce-Hamming");
	stereoBm = StereoBM::create();
}


void Transformation::setPath(string path)
{
	pathSelected = path;
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


Mat Transformation::computeTrajectoryPoints(Mat& img, Mat& prevImg)
{
	vector<Point2f> prevPoints, currentPoints;
	vector<uchar> currentStatus = Detection::findMatchings(img, prevImg, prevPoints, currentPoints);

	Mat mask = Mat::zeros(prevImg.size(), prevImg.type());

	vector<Point2f> goodNew;
	for (int i = 0; i < prevPoints.size(); ++i)
	{
		if (currentStatus[i] == 1)
		{
			goodNew.push_back(prevPoints[i]);
			line(mask, prevPoints[i], currentPoints[i], Scalar(255, 0, 0), 2);
			circle(img, prevPoints[i], 5, Scalar(255, 0, 0), -1);
		}
	}

	Mat finalImg;
	add(img, mask, finalImg);

	return finalImg;
}

void Transformation::drawTrajectory(Mat& prevImg, Mat& currentImg, int idx, Mat& traj)
{
	vector<Point2f> prevPoints, currentPoints;
	Mat mask;

	vector<uchar> currentStatus;
	Detection::tracking(prevImg, currentImg, prevPoints, currentPoints, currentStatus);

	double focal = 718.8560 / 3;
	Point2d point = Point2d(607.1928 / 3, 185.2157 / 3);

	Mat essentialMat = findEssentialMat(prevPoints, currentPoints, focal, point, RANSAC, 0.999, 1.0, mask);
	recoverPose(essentialMat, prevPoints, currentPoints, recover, t, focal, point, mask);

	if (idx == 1)
	{
		recoverCopy = recover.clone();
		tCopy = t.clone();
	} else
	{
		double scale = getAbsoluteScale(idx);

		if ((scale > 0.1) && (t.at<double>(2) > t.at<double>(0)) && (t.at<double>(2) > t.at<double>(1))) {
			tCopy = tCopy + scale * (recoverCopy * t);
			recoverCopy = recover * recoverCopy;
		}
		else {
			cout << "scale below 0.1, or incorrect translation" << endl;
		}

		if (prevPoints.size() < 2000) {
			Detection::tracking(prevImg, currentImg, prevPoints, currentPoints, currentStatus);
		}

		int x = int(tCopy.at<double>(0)) + 300;
		int y = int(tCopy.at<double>(2)) + 100;
		
		circle(traj, Point(x, y), 1, CV_RGB(255, 0, 0), 2);
		rectangle(traj, Point(10, 30), Point(550, 50), CV_RGB(0, 0, 0), 1);
	}

	imshow("Trajectory", traj);
}

void Transformation::loadScale()
{
	string line;
	ifstream poseFile(imgDirectory + pathSelected + ".txt");

	double x = 0, y = 0, z = 0;
	double x_prev, y_prev, z_prev;

	int i = 0;
	if (poseFile.is_open())
	{
		while (getline(poseFile, line))
		{
			z_prev = z;
			x_prev = x;
			y_prev = y;
			
			istringstream in(line);
			for (int j = 0; j < 12; j++) {
				in >> z;

				if (j == 7) {
					y = z;
				}
				else if (j == 3) {
					x = z;
				}
			}

			if (i != 0)
			{
				scales.push_back(
					sqrt((x - x_prev) * (x - x_prev) + (y - y_prev) * (y - y_prev) + (z - z_prev) * (z - z_prev))
				);
			}
			
			i++;
		}

		poseFile.close();
	}
	else {
		cout << "Unable to open file";
	}
}

double Transformation::getAbsoluteScale(int frameId)
{
	return scales[frameId - 2];
}