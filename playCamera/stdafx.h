// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once

#include "targetver.h"

#include <stdio.h>
#include <tchar.h>



// TODO: reference additional headers your program requires here

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