#include "Menu.h"
#include "Constants.h"

void Menu::startMenu()
{
	string selection = "";
	string defaultSelection = "road1";
	DIR* directory;

	do 
	{
		cout << "Choissisez un dossier : (road1_grey) " << endl;
		cin >> selection;

		if (selection == "\n") selection = defaultSelection;
		
		selection = imgDirectory + selection;

		cout << "Dossier choisi : " << selection << endl;
		directory = opendir(selection.c_str());

		if (directory == NULL)
		{
			cout << "Dossier non existant" << endl;
		}
	} while (directory == NULL);

	startVideo(directory, selection);

	closedir(directory);
}

void Menu::startVideo(DIR* dir, string path)
{
	dirent* entry;
	Mat img;
	Mat prevImg;

	vector<Point2f> cornerA, cornerB;
	int x = 0;
	int y = 0;

	bool displayImg;
	
	while (entry = readdir(dir))
	{
		displayImg = false;
		prevImg = img;

		img = imread(path + "/" + entry->d_name);
		
		if (img.data)
		{
			if (x == 0 && y == 0)
			{
				x = img.cols;
				y = img.rows;
			}
			
			resize(img, img, Size(x, y));
			cvtColor(img, img, COLOR_BGR2GRAY);
		}

		if (img.data && prevImg.data)
		{
			cornerA.clear();
			cornerB.clear();
			vector<uchar> currentStatus = Detection::findMatchings(img, prevImg, cornerA, cornerB);
			vector<uchar> prevStatus = Detection::findMatchings(prevImg, img, cornerB, cornerA);

			Mat match1, match2;

			img.copyTo(match1);
			prevImg.copyTo(match2);

 			Detection::displayMatchings(match1, match2, cornerA, cornerB, currentStatus, prevStatus);

			displayImg = true;
			imshow("image_1", match1);
			imshow("image_2", match2);
		}

		if (displayImg)
		{
			waitKey(0);
		}
	}
}
