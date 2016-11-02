#include "MultithreadCam.h"

using namespace FlyCapture2;

MultithreadCam::MultithreadCam()
{
    m_cam = setupCam();
	if (m_cam != NULL)
	{
		GigEImageSettings imageSettings;
		CheckError(m_cam->GetGigEImageSettings(&imageSettings));
		m_image_size = cv::Size(imageSettings.width, imageSettings.height);
	}
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
	//Sleep(10);
    //cout << "unlocking readLock" << endl;
    readLock.unlock();
    cv::cvtColor(NowFrame, NowFrame, CV_RGB2BGR);
    return NowFrame.clone();
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

void MultithreadCam::stop()
{
    // Stop capturing images
    CheckError(m_cam->StopCapture());

    // Disconnect the camera
    CheckError(m_cam->Disconnect());
}

bool MultithreadCam::isInit()
{
	return (m_cam != NULL);
}


/********** Camera control **********/
void PrintError( Error error )
{
    error.PrintErrorTrace();
}

int CheckError( Error error )
{
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

void PrintBuildInfo()
{
    FC2Version fc2Version;
    Utilities::GetLibraryVersion( &fc2Version );

    ostringstream version;
    version << "FlyCapture2 library version: " << fc2Version.major << "." << fc2Version.minor << "." << fc2Version.type << "." << fc2Version.build;
    cout << version.str() <<endl;

    ostringstream timeStamp;
    timeStamp << "Application build date: " << __DATE__ << " " << __TIME__;
    cout << timeStamp.str() << endl << endl;
}

void PrintCameraInfo( CameraInfo* pCamInfo )
{
    cout << endl;
    cout << "*** CAMERA INFORMATION ***" << endl;
    cout << "Serial number -" << pCamInfo->serialNumber << endl;
    cout << "Camera model - " << pCamInfo->modelName << endl;
    cout << "Camera vendor - " << pCamInfo->vendorName << endl;
    cout << "Sensor - " << pCamInfo->sensorInfo << endl;
    cout << "Resolution - " << pCamInfo->sensorResolution << endl;
    cout << "Firmware version - " << pCamInfo->firmwareVersion << endl;
    cout << "Firmware build time - " << pCamInfo->firmwareBuildTime << endl << endl;

}

void PrintFormat7Capabilities( Format7Info fmt7Info )
{
    cout << "Max image pixels: (" << fmt7Info.maxWidth << ", " << fmt7Info.maxHeight << ")" << endl;
    cout << "Image Unit size: (" << fmt7Info.imageHStepSize << ", " << fmt7Info.imageVStepSize << ")" << endl;
    cout << "Offset Unit size: (" << fmt7Info.offsetHStepSize << ", " << fmt7Info.offsetVStepSize << ")" << endl;
    cout << "Pixel format bitfield: 0x" << fmt7Info.pixelFormatBitField << endl;

}

GigECamera* setupCam()
{
    /********** Set up camera **********/
    PrintBuildInfo();
    BusManager busMgr;
    unsigned int numCameras;
    CheckError(busMgr.GetNumOfCameras(&numCameras));
    cout << "Number of cameras detected: " << numCameras <<endl;

    if ( numCameras < 1 )
    {
        cout << "Insufficient number of cameras... exiting" << endl;
		return NULL;
	}

    PGRGuid guid;
    CheckError(busMgr.GetCameraFromIndex(0, &guid));

    GigECamera *cam = new GigECamera();
    // Connect to a camera
    CheckError(cam->Connect(&guid));

    // Get the camera information
    CameraInfo camInfo;
    CheckError(cam->GetCameraInfo(&camInfo));

    PrintCameraInfo(&camInfo);

    GigEImageSettingsInfo imageInfo;
    CheckError(cam->GetGigEImageSettingsInfo(&imageInfo));
    if ((PIXEL_FORMAT_BGR & imageInfo.pixelFormatBitField) == 0)
    {
        cout << "Pixel format is not supported" << endl;
		system("pause");
    }

    Mode mode = MODE_0;
    CheckError(cam->SetGigEImagingMode(mode));

    unsigned int maxPacketSize;
    CheckError(cam->DiscoverGigEPacketSize(&maxPacketSize));
    cout << "Max packet size = " << maxPacketSize << endl;

    GigEProperty packetSizeProp;
    packetSizeProp.propType = PACKET_SIZE;
    packetSizeProp.isReadable = true;
    packetSizeProp.value = maxPacketSize;
    CheckError(cam->SetGigEProperty(&packetSizeProp));
    cout << "Packet size = " << maxPacketSize << endl;
    int PACKET_DELAY_VAL = 0;
    GigEProperty packetDelayProp;
    packetDelayProp.propType = PACKET_DELAY;
    packetDelayProp.isReadable = true;
    packetDelayProp.value = PACKET_DELAY_VAL;
    CheckError(cam->SetGigEProperty(&packetDelayProp));
    cout << "Packet delay = " << PACKET_DELAY_VAL << endl;

    Property brightnessProp;
    brightnessProp.type = BRIGHTNESS;
    brightnessProp.absValue = 25; //% (0-25)
    brightnessProp.autoManualMode = false;
    brightnessProp.onOff = true;
    brightnessProp.absControl = true;
    CheckError(cam->SetProperty(&brightnessProp));
    CheckError(cam->GetProperty(&brightnessProp));
    cout << "Brightness = " << brightnessProp.absValue << endl;
    Property exposureProp;
    exposureProp.type = AUTO_EXPOSURE;
    exposureProp.absValue = 1.5;
    exposureProp.autoManualMode = false;
    exposureProp.onOff = true;
    exposureProp.absControl = true;
    CheckError(cam->SetProperty(&exposureProp));
    CheckError(cam->GetProperty(&exposureProp));
    cout << "Exposure = " << exposureProp.absValue << endl;
    Property sharpnessProp;
    sharpnessProp.type = SHARPNESS;
    sharpnessProp.valueA = 1048; //abs (0 - 4095)
    sharpnessProp.autoManualMode = false;
    sharpnessProp.onOff = true;
    sharpnessProp.absControl = false; //absControl not supported
    CheckError(cam->SetProperty(&sharpnessProp));
    CheckError(cam->GetProperty(&sharpnessProp));
    cout << "Sharpness = " << sharpnessProp.valueA << endl;
    Property gammaProp;
    gammaProp.type = GAMMA;
    gammaProp.absValue = 0.5;
    gammaProp.autoManualMode = false;
    gammaProp.onOff = true;
    gammaProp.absControl = true;
    CheckError(cam->SetProperty(&gammaProp));
    CheckError(cam->GetProperty(&gammaProp));
    cout << "gamma = " << gammaProp.absValue << endl;

    Property whitebalanceProp;
    whitebalanceProp.type = WHITE_BALANCE;
    whitebalanceProp.onOff = false;
    whitebalanceProp.autoManualMode = true;
    CheckError(cam->SetProperty(&whitebalanceProp));

    Property shutterProp;
    shutterProp.type = SHUTTER;
    shutterProp.autoManualMode = true;
    shutterProp.absControl = true;
    shutterProp.absValue = 0.5;
    CheckError(cam->SetProperty(&shutterProp));

    Property gainProp;
    gainProp.type = GAIN;
    gainProp.autoManualMode = true;
    gainProp.absControl = true;
    gainProp.absValue = 0;
    CheckError(cam->SetProperty(&gainProp));

    const int WIDTH = min(floor(sqrt(maxPacketSize * 1000)), (double)min(imageInfo.maxWidth, imageInfo.maxHeight));
    const int HEIGHT = WIDTH;
    GigEImageSettings imageSettings;
    imageSettings.width = (WIDTH/4) * 4; //Must be of intervals of 4
    imageSettings.height = (HEIGHT/2) * 2; //Must be of intervals of 2
    imageSettings.offsetX = ((imageInfo.maxWidth - WIDTH) / 2) / 4 * 4; //Must be of intervals of 4
    imageSettings.offsetY = ((imageInfo.maxHeight - HEIGHT) / 2) / 2 * 2; //Must be of intervals of 2
    imageSettings.pixelFormat = PIXEL_FORMAT_RGB8;
    // Set the settings to the camera
    CheckError(cam->SetGigEImageSettings(&imageSettings));
    CheckError(cam->GetGigEImageSettings(&imageSettings));
    cv::Size imgSize(imageSettings.width, imageSettings.height);
    cout << "Image size = " << imgSize << endl;
    cout << "Image X-offset = " << imageSettings.offsetX << ", Y-offset = " << imageSettings.offsetY << endl;
    cout << "Pixel format = " << imageSettings.pixelFormat << endl;

    // Start capturing images
    CheckError(cam->StartCapture());
    // Retrieve frame rate property
    Property frameProp;
    frameProp.type = FRAME_RATE;
    CheckError(cam->GetProperty( &frameProp ));
    cout << "Frame rate is " << fixed << setprecision(2) << frameProp.absValue << " fps" << endl;
    return cam;
}

