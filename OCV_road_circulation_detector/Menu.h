#pragma once

#include <iostream>
#include <string>

#include "dirent.h"
#include "Transformation.h"

class Menu
{
private:
	Transformation transformation;
	
	void startVideo(DIR* dir, string path);
	void printImg(vector<Mat> imgs);
	void printImg(Mat img, int idx);
	void printImg(Mat img);
	
public:
	void startMenu();
};