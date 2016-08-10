#pragma once

#include "stdafx.h"
#include "FlyCapture2.h"

//Should be singleton...
class MultithreadCam
{
public:
	MultithreadCam();
	~MultithreadCam(void);
	cv::Mat getImage();
	cv::Mat getImage(bool isBright);
	cv::Size getImageSize();
	void setShutter(bool isBright);
    void stop();
private:
	GigECamera* m_cam;
	cv::Size m_image_size;
	std::mutex readLock;
	std::mutex modeLock;
	bool m_isBright;
};

int CheckError(Error error);
void PrintError(Error error);
void PrintBuildInfo();
void PrintCameraInfo( CameraInfo* pCamInfo );
void PrintFormat7Capabilities( Format7Info fmt7Info );
GigECamera setupCam();

