// stdafx.cpp : source file that includes just the standard includes
// playCamera.pch will be the pre-compiled header
// stdafx.obj will contain the pre-compiled type information

#include "stdafx.h"

// TODO: reference any additional headers you need in STDAFX.H
// and not in this file

string getString(Room room)
{
	switch (room)
	{
		case UNDEFINED:
			return "undefined";
		case DINING:
			return "dining";
		case LIVING:
			return "living";
		case STUDY:
			return "study";
	}
}

vector<cv::Point2f> getROIPts2f()
{
	vector<cv::Point2f> ROI_PTS2f;
	ROI_PTS2f.push_back(cv::Point2f(400, 335));
	ROI_PTS2f.push_back(cv::Point2f(1470, 665));
	ROI_PTS2f.push_back(cv::Point2f(1370, 1450));
	ROI_PTS2f.push_back(cv::Point2f(140, 1285));
	//ROI_PTS2f.push_back(ROI_PTS2f[0]);
	return ROI_PTS2f;
}
		 
string getCurrentTime()
{	
	time_t curr_time = time(NULL);
	char time_char[128];
	strftime(time_char, 128, "%Y-%m-%d-%H%M%S", localtime(&curr_time));
	string time_str(time_char);
	return time_str;
}

void PrintError( Error error )
{
    error.PrintErrorTrace();
}

int CheckError( Error error ){
	if (error != PGRERROR_OK)
    {
		//Ignore image consistency error for now...
        PrintError( error );
		//system("pause");
        //exit(-1);
		return -1;
    }
	return 0;
}