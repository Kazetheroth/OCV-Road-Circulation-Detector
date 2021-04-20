#pragma once

#include <iostream>
#include <string>

#include "dirent.h"
#include "Detection.h"

class Menu
{
private:
	void startVideo(DIR* dir, string path);
	
public:
	void startMenu();
};