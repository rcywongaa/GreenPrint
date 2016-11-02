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
	ROI_PTS2f.push_back(cv::Point2f(425, 490));
	ROI_PTS2f.push_back(cv::Point2f(1515, 530));
	ROI_PTS2f.push_back(cv::Point2f(1540, 1360));
	ROI_PTS2f.push_back(cv::Point2f(360, 1345));
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

int getCurrentHour()
{
    time_t curr_time = time(NULL);
    tm* curr_time_tm = localtime(&curr_time);
    int hour = curr_time_tm->tm_hour;
    return hour;
}

int getCurrentMinute()
{
    time_t curr_time = time(NULL);
    tm* curr_time_tm = localtime(&curr_time);
	int minute = curr_time_tm->tm_min;
    return minute;
}

void show(string window, cv::Mat img)
{
	cv::Mat resized;
	cv::resize(img, resized, cv::Size(320, 240));
	//cv::resize(img, resized, cv::Size(640, 480));
	imshow(window, resized);
}

cv::Mat getROIMask()
{
	cv::Mat mask(1944, 1944, CV_8UC1);
	cv::Point pts[4];
	for (int i = 0; i < 4; i++) pts[i] = getROIPts2f()[i];
	cv::fillConvexPoly(mask, &pts[0], 4, cv::Scalar(255));
	//show("roi mask", mask);
	return mask;
}

void rebootSystem()
{
	system("c:\\windows\\system32\\shutdown /r /f /t 0\n\n");
	/*
	PHANDLE token;
	OpenProcessToken(GetCurrentProcess(), TOKEN_ADJUST_PRIVILEGES, token);
	SetPrivilege(*token, SE_SHUTDOWN_NAME, true);
	InitiateSystemShutdown(NULL, NULL, 0, SHUTDOWN_RESTART, SHTDN_REASON_MAJOR_APPLICATION);
	//ExitWindowsEx(EWX_REBOOT, SHTDN_REASON_MAJOR_APPLICATION);
	*/
}


/*
#pragma comment(lib, "cmcfg32.lib")

BOOL SetPrivilege(
    HANDLE hToken,          // access token handle
    LPCTSTR lpszPrivilege,  // name of privilege to enable/disable
    BOOL bEnablePrivilege   // to enable or disable privilege
    ) 
{
    TOKEN_PRIVILEGES tp;
    LUID luid;

    if ( !LookupPrivilegeValue( 
            NULL,            // lookup privilege on local system
            lpszPrivilege,   // privilege to lookup 
            &luid ) )        // receives LUID of privilege
    {
        printf("LookupPrivilegeValue error: %u\n", GetLastError() ); 
        return FALSE; 
    }

    tp.PrivilegeCount = 1;
    tp.Privileges[0].Luid = luid;
    if (bEnablePrivilege)
        tp.Privileges[0].Attributes = SE_PRIVILEGE_ENABLED;
    else
        tp.Privileges[0].Attributes = 0;

    // Enable the privilege or disable all privileges.

    if ( !AdjustTokenPrivileges(
           hToken, 
           FALSE, 
           &tp, 
           sizeof(TOKEN_PRIVILEGES), 
           (PTOKEN_PRIVILEGES) NULL, 
           (PDWORD) NULL) )
    { 
          printf("AdjustTokenPrivileges error: %u\n", GetLastError() ); 
          return FALSE; 
    } 

    if (GetLastError() == ERROR_NOT_ALL_ASSIGNED)

    {
          printf("The token does not have the specified privilege. \n");
          return FALSE;
    } 

    return TRUE;
}
*/