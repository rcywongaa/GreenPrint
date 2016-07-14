#include "MultithreadCam.h"

MultithreadCam::MultithreadCam(GigECamera* cam, cv::Size image_size)
{
	m_cam = cam;
	m_image_size = image_size;
	m_isBright = false;
	//setShutter(m_isBright);
}


MultithreadCam::~MultithreadCam(void)
{
}

cv::Mat MultithreadCam::getImage()
{
	readLock.lock();
	//cout << "locking readLock" << endl;
	Image rawImage;
	cv::Mat NowFrame;
	CheckError(m_cam->RetrieveBuffer( &rawImage ));
	NowFrame = cv::Mat(m_image_size, CV_8UC3, rawImage.GetData());
	//cout << "unlocking readLock" << endl;
	readLock.unlock();
	return NowFrame;
}

cv::Mat MultithreadCam::getImage(bool isBright)
{
	modeLock.lock();
	//cout << "locking modeLock" << endl;
	//cout << "isBright = " << isBright << endl;
	if (m_isBright != isBright)
	{
		m_isBright = isBright;
		setShutter(isBright);
	}
	cv::Mat ret = getImage();

	//cout << "m_isBright = " << m_isBright << endl;
	//cout << "unlocking modeLock" << endl;
	modeLock.unlock();
	return ret;
}

void MultithreadCam::setShutter(bool isBright)
{
	Property shutterProp;
	shutterProp.type = SHUTTER;
	shutterProp.autoManualMode = false;
	shutterProp.absControl = true;
	shutterProp.absValue = isBright? 0.1 : 5;
	CheckError(m_cam->SetProperty(&shutterProp));
	CheckError(m_cam->GetProperty(&shutterProp));
	//cout << "shutterProp = " << shutterProp.absValue << endl;
}

cv::Size MultithreadCam::getImageSize()
{
	return m_image_size;
}