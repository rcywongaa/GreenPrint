#include "RectFinder.h"

#define DEBUG
#define WATERSHED

#define MIN_DIM 100
#define MAX_DIM 400
#define FILL_THRESH 0.8
#define LOW_FILL_THRESH 0.2
#define MAX_MOVE 100
#define NUM_SAMPLES 500

//TODO: Check Mat type consistency

RectFinder::RectFinder(vector<tuple<cv::Scalar, cv::Scalar>> color_ranges, cv::Scalar rect_color)
{
    //TODO: Add assert
	m_color_ranges = vector<tuple<cv::Scalar, cv::Scalar>>(NUM_ROOMS);
    m_color_ranges = color_ranges;
    m_color = rect_color;
}

RectFinder::RectFinder(cv::Scalar color1, cv::Scalar color2)
{
	m_color_ranges = vector<tuple<cv::Scalar, cv::Scalar>>(NUM_ROOMS);
    tuple<cv::Scalar, cv::Scalar> param(color1, color2);
	m_color_ranges[ON] = param;
}

void RectFinder::process(cv::Mat input)
{
    process(input, ON);
}

void RectFinder::process(cv::Mat input, Room room)
{
    m_size = input.size();
	show("input", input);
    cv::pyrMeanShiftFiltering(input, input, 50, 50);
    //cv::bilateralFilter(input, input, 5, 150, 150);
#ifdef DEBUG
	show("filtered", input);
#endif

    cv::Mat mask = findColor(input, room);
	cv::bitwise_and(mask, getROIMask(), mask);

    cv::Mat fg = findForeground(input, mask);

#ifdef WATERSHED
    vector<cv::RotatedRect> curr_rects = findGrayRects(fg);
#elif GRABCUT
    vector<cv::RotatedRect> curr_rects = findBinaryRects(fg);
#endif
    vector<cv::RotatedRect> true_rects = curr_rects;

    for (auto prev_rect = m_prev_rects.begin(); prev_rect != m_prev_rects.end(); prev_rect++)
    {
        bool isMatched = false;
        for (auto curr_rect = curr_rects.begin(); curr_rect != curr_rects.end(); curr_rect++)
        {
            vector<cv::Point2f> curr_rect_pts(4);
            curr_rect->points(&curr_rect_pts[0]);
            if (cv::pointPolygonTest(curr_rect_pts, prev_rect->center, false) > 0)
            {
                isMatched = true;
                break;
            }
            //if (cv::norm(prev_rect->rect.center - now_rect->rect.center) < MAX_MOVE)
        }
        auto min_dist_rect = min_element(curr_rects.begin(), curr_rects.end(), 
                [=](cv::RotatedRect v1, cv::RotatedRect v2)
                {
                    return (cv::norm(prev_rect->center - v1.center) < cv::norm(prev_rect->center - v2.center));
                }
        );
        if (cv::norm(prev_rect->center - min_dist_rect->center) < MAX_MOVE)
        {
            isMatched = true;
        }
        //Rects near edge?

        if (isMatched == false && getFillRate(getRotatedRectROI(mask, *prev_rect)) > LOW_FILL_THRESH)
        {
            true_rects.push_back(*prev_rect);
        }
    }
    m_prev_rects = true_rects;
}

cv::Mat RectFinder::drawColorRects()
{
    cv::Mat output(m_size, CV_8UC3);
    return drawColorRects(output);
}

cv::Mat RectFinder::drawColorRects(cv::Mat original)
{
    cv::Mat output = original.clone();
	for (auto rect : m_prev_rects)
	{
		cv::Mat this_rect_frame(m_size, CV_8UC3);
        this_rect_frame = drawRotatedRect(this_rect_frame, rect, m_color);
		cv::max(this_rect_frame, output, output);
	}
	return output;
}

cv::Mat RectFinder::findColor(cv::Mat input, Room room)
{
    assert(input.type == CV_8UC3);
	cv::Mat hsv;
	cv::Mat hsv_array[3];
	cv::Mat mask;
	cv::cvtColor(input, hsv, CV_BGR2HSV);
	cv::split(hsv, hsv_array);
#ifdef DEBUG
	show("h", hsv_array[0]);
	show("s", hsv_array[1]);
	show("v", hsv_array[2]);
#endif
    int hour = getCurrentHour();
    if (hour > 18 && hour < 19)
    {
        cv::inRange(hsv, get<0>(m_color_ranges[ON]), get<1>(m_color_ranges[ON]), mask);
    }
    else
    {
        cv::inRange(hsv, get<0>(m_color_ranges[room]), get<1>(m_color_ranges[room]), mask);
    }
	cv::bitwise_and(mask, getROIMask(), mask);
#ifdef DEBUG
	show("mask", mask);
#endif
    return mask;
}

cv::Mat RectFinder::findForeground(cv::Mat input, cv::Mat mask)
{
    int BG_MARKER = 100;
    int FG_MARKER = 200;
    int MIN_DIST = 100;
    //distanceTransform
    cv::Mat dist_mask;
    cv::distanceTransform(mask, dist_mask, CV_DIST_L2, 3);
    cv::Mat fg;
    cv::threshold(dist_mask, fg, MIN_DIST, FG_MARKER, THRESH_BINARY);
    show("fg", fg);
    vector<vector<Point>> contours;
    findContours(fg, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    cv::Mat bg;
    cv::dilate(mask, bg, cv::Mat(), cv::Point(-1, -1), 10);
    cv::bitwise_not(bg, bg);
    show("bg", bg);
    bg = cv::min(bg, BG_MARKER);
#ifdef WATERSHED
    cv::Mat markers = bg;
    for (int i = 0; i < contours.size(); i++)
    {
        drawContours(markers, contours, i, cv::Scalar::all(FG_MARKER + i), -1);
    }
    show("markers", markers);
    cv::watershed(input, markers);
    show("watershed", markers);
#elif GRABCUT
    cv::Mat bgdModel;
    cv::Mat fgdModel;
    cv::Mat markers = cv::max(fg, bg);
    markers.setTo(cv::Scalar(255), markers == 0);
    markers.setTo(GC_FGD, markers == FG_MARKER);
    markers.setTo(GC_BGD, markers == BG_MARKER);
    show("markers", markers);
    cv::grabCut(input, markers, cv::Rect(), bgdModel, fgdModel, 5, cv::GC_INIT_WITH_MASK);
    show("grabcut", markers);
#endif

    //bounding circle?
    return markers;
}

vector<cv::RotatedRect> RectFinder::findGrayRects(cv::Mat mask)
{
    vector<cv::RotatedRect> ret;
    vector<unsigned char> values = unique(mask, true);
    for (auto value : values)
    {
        if (value == 0) continue;
        vector<cv::RotatedRect> rects = findBinaryRects(mask == value);
        ret.insert(ret.end(), rects.begin(), rects.end());
    }
    return ret;
}

vector<cv::RotatedRect> RectFinder::findBinaryRects(cv::Mat mask)
{
    assert(input.type == CV_8UC1);
    //TODO: Recognize rectangles of different gray level
    vector<cv::RotatedRect> rects;
	vector<vector<cv::Point>> contours;
	cv::findContours(mask, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	cv::Mat contourFrame = cv::Mat::zeros(mask.size(), CV_8UC3);
	cv::drawContours(contourFrame, contours, -1, cv::Scalar(255, 255, 255));
	cv::Mat rectFrame = cv::Mat::zeros(mask.size(), CV_8UC3);
	cv::Mat polyFrame = cv::Mat::zeros(mask.size(), CV_8UC3);
	for (auto contour : contours)
	{
		vector<cv::Point> polygon;
		cv::approxPolyDP(contour, polygon, 0.05 * cv::arcLength(cv::Mat(contour), true), true);
		int npt[] = {polygon.size()}; //Because fillPoly assumes an array polygons...
		const cv::Point* ppt[1] = {&polygon[0]};
		cv::fillPoly(polyFrame, ppt, npt, 1, cv::Scalar(255, 255, 255));

		cv::RotatedRect rect;
		rect = cv::minAreaRect(polygon);


        //Consider using average cosAngles
        //if (cv::contourArea(polygon) / rect.size.area() > 0.9)
		if 
        (
            (rect.size.height > MIN_DIM && rect.size.height < 400 && rect.size.width > MIN_DIM && rect.size.width < 400)
            && (polygon.size() >= 4 && polygon.size() <= 6)
            && (getFillRate(getRotatedRectROI(mask, rect)) > FILL_THRESH)
        )
        {
            rects.push_back(rect);
            //TODO: Delete after testing
            rectFrame = drawRotatedRect(rectFrame, rect, cv::Scalar(255, 255, 255));
        }
	}
#ifdef DEBUG
	show("polygon", polyFrame);
	show("rect", rectFrame);
#endif
	return rects;
}

/******************** HELPER FUNCTIONS ********************/

std::vector<unsigned char> unique(const cv::Mat& input, bool sort = false)
{
    if (input.channels() > 1 || input.type() != CV_UC1) 
    {
        std::cerr << "unique !!! Only works with CV_UC 1-channel Mat" << std::endl;
        return std::vector<unsigned char>();
    }

    std::vector<unsigned char> out;
    for (int y = 0; y < input.rows; ++y)
    {
        const unsigned char* row_ptr = input.ptr<unsigned char>(y);
        for (int x = 0; x < input.cols; ++x)
        {
            unsigned char value = row_ptr[x];

            if ( std::find(out.begin(), out.end(), value) == out.end() )
                out.push_back(value);
        }
    }

    if (sort)
        std::sort(out.begin(), out.end());

    return out;
}

cv::RotatedRect findBestFitRect(cv::Mat mask)
{
    //http://www.visiondummy.com/2014/04/draw-error-ellipse-representing-covariance-matrix/
    //Optional: + random sampling within bounding rect
#ifdef DEBUG
    show("findBestFitRect input", mask);
#endif
    cv::Mat points = ransam(mask, NUM_SAMPLES);
    cv::Mat covar;
    cv::Mat mean;
    cv::calcCovarMatrix(points, covar, mean, CV_COVAR_ROWS, CV_32S);
    cv::RotatedRect best_fit_rect = getErrorEllipse(cv::Point2f(mean.at<int>(0, 0), mean.at<int>(0, 1)), covar);
#ifdef DEBUG
    show("bestfitrect", drawRotatedRect(mask, best_fit_rect, cv::Scalar(255)));
#endif
    return best_fit_rect;
}

cv::Mat ransam(cv::Mat mask, int size)
{
    vector<cv::Point> points;
    while (points.size() < size)
    {
        int col = rand() % mask.cols;
        int row = rand() % mask.rows;
        if (mask.at<uchar>(row, col) > 0)
        {
            points.push_back(cv::Point(row, col));
        }
    }
    return cv::Mat(points);
}

//http://www.visiondummy.com/2014/04/draw-error-ellipse-representing-covariance-matrix/
cv::RotatedRect getErrorEllipse(cv::Point2f mean, cv::Mat covmat)
{
    //Get the eigenvalues and eigenvectors
    cv::Mat eigenvalues, eigenvectors;
    cv::eigen(covmat, eigenvalues, eigenvectors);

    //Calculate the angle between the largest eigenvector and the x-axis
    double angle = atan2(eigenvectors.at<double>(0,1), eigenvectors.at<double>(0,0));

    //Shift the angle to the [0, 2pi] interval instead of [-pi, pi]
    if(angle < 0)
        angle += 6.28318530718;

    //Conver to degrees instead of radians
    angle = 180*angle/3.14159265359;

    //Calculate the size of the minor and major axes
    //double halfmajoraxissize=chisquare_val*sqrt(eigenvalues.at<double>(0));
    //double halfminoraxissize=chisquare_val*sqrt(eigenvalues.at<double>(1));
    double halfmajoraxissize=2*sqrt(eigenvalues.at<double>(0));
    double halfminoraxissize=2*sqrt(eigenvalues.at<double>(1));

    //Return the oriented ellipse
    //The -angle is used because OpenCV defines the angle clockwise instead of anti-clockwise
    return cv::RotatedRect(mean, cv::Size2f(halfmajoraxissize, halfminoraxissize), -angle);
}

double cosAngle(cv::Point pt1, cv::Point pt2, cv::Point pt0 )
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

cv::Mat getRotatedRectROI(cv::Mat input, cv::RotatedRect rect)
{
    cv::Mat output;
    float angle = rect.angle;
    cv::Size rect_size = rect.size;
    if (rect.angle < -45.0) 
    {
        angle += 90.0;
        swap(rect_size.width, rect_size.height);
    }
    cv::Mat rot_mat = cv::getRotationMatrix2D(rect.center, angle, 1.0);
    cv::Mat rotated_input;
    cv::warpAffine(input, rotated_input, rot_mat, input.size(), CV_INTER_CUBIC);
    cv::getRectSubPix(rotated_input, rect_size, rect.center, output);
    return output;
}

float getFillRate(cv::Mat input)
{
    assert(input.type() == CV_8UC1);
    return cv::sum(input)[0] / (255 * input.rows * input.cols);
}

cv::Mat drawRotatedRect(cv::Mat input, cv::RotatedRect rect, cv::Scalar color)
{
    cv::Mat ret = input.clone();
    cv::Point2f pts2f[4];
    rect.points(pts2f);
    cv::Point pts[4];
    for (int i = 0; i < 4; i++) pts[i] = pts2f[i];
    cv::fillConvexPoly(ret, &pts[0], 4, color);
    return ret;
}

vector<ColorRect> findColorRects(cv::Mat input, tuple<cv::Scalar, cv::Scalar> colors, cv::Scalar rect_color)
{
	return findColorRects(input, get<0>(colors), get<1>(colors), rect_color);
}

vector<ColorRect> findColorRects(cv::Mat input, cv::Scalar color1, cv::Scalar color2, cv::Scalar rect_color)
{
	vector<ColorRect> color_rects;
#ifdef DEBUG
	show("input", input);
#endif
	cv::medianBlur(input, input, 31);
#ifdef DEBUG
	show("filtered", input);
#endif
	cv::Mat hsv;
	cv::Mat hsv_array[3];
	cv::Mat mask;
	cv::cvtColor(input, hsv, CV_BGR2HSV);
	cv::split(hsv, hsv_array);
#ifdef DEBUG
	show("h", hsv_array[0]);
	show("s", hsv_array[1]);
	show("v", hsv_array[2]);
#endif
	cv::inRange(hsv, color1, color2, mask);
	cv::bitwise_and(mask, getROIMask(), mask);
#ifdef DEBUG
	show("mask", mask);
#endif
	cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 10);
	cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 10);
#ifdef DEBUG
	show("dilated_mask", mask);
#endif

	vector<vector<cv::Point>> contours;
	vector<cv::Vec4i> contour_hierarchy;
	cv::findContours(mask, contours, contour_hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	cv::Mat contourFrame = cv::Mat::zeros(input.size(), input.type());
	cv::drawContours(contourFrame, contours, -1, cv::Scalar(255, 255, 255));
	cv::Mat rectFrame = cv::Mat::zeros(input.size(), input.type());
	cv::Mat polyFrame = cv::Mat::zeros(input.size(), input.type());
	for (auto contour : contours)
	{
		vector<cv::Point> polygon;
		cv::approxPolyDP(contour, polygon, 0.05 * cv::arcLength(cv::Mat(contour), true), true);
		double maxCos = 0.0;
		vector<float> cos;
		for (int i = 0; i < polygon.size(); i++)
		{
			int size = polygon.size();
			cos.push_back(cosAngle(polygon[i], polygon[(i+2) % size], polygon[(i+1) % size]));
			//maxCos = std::max(maxCos, abs(cosAngle(polygon[i], polygon[(i+2) % size], polygon[(i+1) % size])));
		}
		int npt[] = {polygon.size()}; //Because fillPoly assumes an array polygons...
		const cv::Point* ppt[1] = {&polygon[0]};
		cv::fillPoly(polyFrame, ppt, npt, 1, cv::Scalar(255, 255, 255));

		cv::RotatedRect rect;
		rect = cv::minAreaRect(polygon);

		if (rect.size.height > 100  && rect.size.height < 400 && rect.size.width > 100 && rect.size.width < 400)
		{
			//Consider using average cosAngles
			if (polygon.size() >= 4 && polygon.size() <= 6)
			{
				//if (cv::contourArea(polygon) / rect.size.area() > 0.9)
				if (cv::pointPolygonTest(getROIPts2f(), rect.center, false) > 0)
				{
					ColorRect color_rect;
					color_rect.rect = rect;
					color_rect.color = rect_color;
					color_rects.push_back(color_rect);
					cv::Point2f pts2f[4];
					rect.points(pts2f);
					cv::Point pts[4];
					for (int i = 0; i < 4; i++) pts[i] = pts2f[i];
					cv::fillConvexPoly(rectFrame, &pts[0], 4, cv::Scalar(255, 255, 255));

				}
			}
		}
	}
#ifdef DEBUG
	show("polygon", polyFrame);
	show("rect", rectFrame);
#endif
	return color_rects;
}

   
