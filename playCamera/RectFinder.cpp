#include "RectFinder.h"

#define DEBUG
#define MIN_DIM 100
#define FILL_THRESH 0.8
#define LOW_FILL_THRESH 0.2

RectFinder::RectFinder(vector<tuple<cv::Scalar, cv::Scalar>> color_ranges, cv::Scalar rect_color)
{
    //TODO: Add assert
    m_color_ranges = color_ranges;
    m_color = rect_color;
}

RectFinder::RectFinder(cv::Scalar color1, cv::Scalar color2)
{
    vector<tuple<cv::Scalar, cv::Scalar>> param;
    param.push_back(tuple<cv::Scalar, cv::Scalar>(color1, color2));
}

void RectFinder::process(cv::Mat input)
{
    process(input, 0);
}

void RectFinder::process(cv::Mat input, Room room)
{
    m_size = input.size();
    cv::Mat mask = findColor(input, room);
    vector<cv::RotatedRect> curr_rects = findRects(mask);
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
        if (isMatched == false && getFillRate(mask, *prev_rect) > LOW_FILL_THRESH)
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
		cv::Point2f pts2f[4];
		rect.rect.points(pts2f);
		cv::Point pts[4];
		for (int i = 0; i < 4; i++) pts[i] = pts2f[i];
		cv::fillConvexPoly(this_rect_frame, &pts[0], 4, rect.color);
		cv::max(this_rect_frame, output, output);
	}
	return output;
}

cv::Mat RectFinder::findColor(cv::Mat input, Room room)
{
	cv::Mat hsv;
	cv::Mat hsv_array[3];
	cv::Mat mask;
	cv::cvtColor(input, hsv, CV_BGR2HSV);
	cv::split(hsv, hsv_array);
#ifdef DEBUG
	show("h", hsv_array[0]);
	show("s", hsv_array[1]);
	how("v", hsv_array[2]);
#endif
    int hour = getCurrentHour();
    if (hour > 18 && hour < 19)
    {
        cv::inRange(hsv, get<0>(m_color_ranges[SIX]), get<1>(m_color_ranges[SIX]), mask);
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

vector<cv::RotatedRect> RectFinder::findRects(cv::Mat mask)
{
    vector<cv::RotatedRect> rects;
	vector<vector<cv::Point>> contours;
	cv::findContours(mask, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	cv::Mat contourFrame = cv::Mat::zeros(mask.size(), mask.type());
	cv::drawContours(contourFrame, contours, -1, cv::Scalar(255, 255, 255));
	cv::Mat rectFrame = cv::Mat::zeros(mask.size(), mask.type());
	cv::Mat polyFrame = cv::Mat::zeros(mask.size(), mask.type());
	for (auto contour : contours)
	{
		vector<cv::Point> polygon;
		cv::approxPolyDP(contour, polygon, 0.05 * cv::arcLength(cv::Mat(contour), true), true);
		double maxCos = 0.0;
		vector<float> cos;
		float medianCos = 0.0;
		for (int i = 0; i < polygon.size(); i++)
		{
			int size = polygon.size();
			cos.push_back(cosAngle(polygon[i], polygon[(i+2) % size], polygon[(i+1) % size]));
			//maxCos = std::max(maxCos, abs(cosAngle(polygon[i], polygon[(i+2) % size], polygon[(i+1) % size])));
		}
		int median_idx = cos.size() / 2;
		std::nth_element(cos.begin(), cos.begin() + median_idx, cos.end());
		medianCos = cos[median_idx];
		int npt[] = {polygon.size()}; //Because fillPoly assumes an array polygons...
		const cv::Point* ppt[1] = {&polygon[0]};
		cv::fillPoly(polyFrame, ppt, npt, 1, cv::Scalar(255, 255, 255));

		cv::RotatedRect rect;
		rect = cv::minAreaRect(polygon);

		if (rect.size.height > MIN_DIM && rect.size.height < 400 && rect.size.width > MIN_DIM && rect.size.width < 400)
		{
			//Consider using average cosAngles
			if (polygon.size() >= 4 && polygon.size() <= 6)
			{
				//if (cv::contourArea(polygon) / rect.size.area() > 0.9)
				//if (cv::pointPolygonTest(getROIPts2f(), rect.center, false) > 0)
                if (getFillRate(getRotatedROI(mask, rect)) > FILL_THRESH)
				{
                    rects.push_back(rect);
                    //TODO: Delete after testing
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
	getROIMask();
    //TODO
	return rects;
}



/******************** HELPER FUNCTIONS ********************/

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
    cv::warpAffine(input, rotated_input, rot_mat, input.size(), INTER_CUBIC);
    cv::getRectSubPix(rotated_input, rect_size, rect.center, output);
    return output;
}

float getFillRate(cv::Mat input)
{
    assert(input.type() == CV_8UC1);
    return cv::sum(input) / (255 * input.rows * input.cols);
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
	how("v", hsv_array[2]);
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
		float medianCos = 0.0;
		for (int i = 0; i < polygon.size(); i++)
		{
			int size = polygon.size();
			cos.push_back(cosAngle(polygon[i], polygon[(i+2) % size], polygon[(i+1) % size]));
			//maxCos = std::max(maxCos, abs(cosAngle(polygon[i], polygon[(i+2) % size], polygon[(i+1) % size])));
		}
		int median_idx = cos.size() / 2;
		std::nth_element(cos.begin(), cos.begin() + median_idx, cos.end());
		medianCos = cos[median_idx];
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
	getROIMask();
    //TODO
	return color_rects;
}

    
