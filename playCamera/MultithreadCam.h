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
	cv::Size getImageSize();
	void setShutter(bool isBright);
    void stop();
	bool isInit();
private:
	FlyCapture2::GigECamera *m_cam;
	cv::Size m_image_size;
	std::mutex readLock;
	std::mutex modeLock;
};

int CheckError(FlyCapture2::Error error);
void PrintError(FlyCapture2::Error error);
void PrintBuildInfo();
void PrintCameraInfo( FlyCapture2::CameraInfo* pCamInfo );
void PrintFormat7Capabilities( FlyCapture2::Format7Info fmt7Info );
FlyCapture2::GigECamera* setupCam();

