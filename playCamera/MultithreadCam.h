#pragma once

#include "stdafx.h"

//Should be singleton...
class MultithreadCam
{
public:
	MultithreadCam(GigECamera* cam, cv::Size image_size);
	~MultithreadCam(void);
	cv::Mat getImage();
	cv::Mat getImage(bool isBright);
	cv::Size getImageSize();
	void setShutter(bool isBright);
private:
	GigECamera* m_cam;
	cv::Size m_image_size;
	std::mutex readLock;
	std::mutex modeLock;
	bool m_isBright;
};

