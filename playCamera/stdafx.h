// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once

#include "targetver.h"

#include <stdio.h>
#include <tchar.h>



// TODO: reference additional headers your program requires here

#include <cstdio>
#include <iostream>
#include <stdlib.h>
#include <omp.h>
#include <iomanip>
#include <string>
#include <xstring>
#include <fstream>
#include <iostream>
#include <exception>
#include <sstream>
#include <windows.h>
#include <mmdeviceapi.h>
#include <endpointvolume.h>
#include <iomanip>
#include <thread>
#include <mutex>
#include <atomic>
#include "FlyCapture2.h"

using namespace std;
using namespace FlyCapture2;



#include <opencv2/opencv.hpp>

#include <curl/curl.h>

#include <tuple>

#define HAVE_IMAGE_HASH 1
#define cimg_plugin1 "cvMat.h"
#include "pHash.h"
#pragma comment(lib, "pHash.lib")

enum Room {DINING, LIVING, STUDY, UNDEFINED, NUM_ROOMS, ON, OFF};

struct RoomPhotos
{
	CImg<uchar>img;
	double score;
	Room  room;
};

struct Texture {
	cv::Scalar color;
	cv::Mat image;
	vector<cv::KeyPoint> keypoints;
	cv::Mat descriptors;
	int max_instances;
	int type;
};

struct ColorRect {
	cv::RotatedRect rect;
	cv::Scalar color;
};

string getString(Room room);
vector<cv::Point2f> getROIPts2f();
string getCurrentTime();
int getCurrentHour();
int CheckError(Error error);
void PrintError(Error error);
cv::Mat getROIMask();
void show(string window, cv::Mat img);