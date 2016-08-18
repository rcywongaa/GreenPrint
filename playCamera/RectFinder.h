#include "stdafx.h"

struct ColorRect {
	cv::RotatedRect rect;
	cv::Scalar color;
};

class RectFinder
{
    public:
        RectFinder(vector<tuple<cv::Scalar, cv::Scalar>> color_ranges, cv::Scalar rect_color);
        RectFinder(cv::Scalar color1, cv::Scalar color2);
        void process(cv::Mat input);
        void process(cv::Mat input, Room room);
        cv::Mat drawColorRects(cv::Mat original);
        cv::Mat drawColorRects();

    private:
        vector<tuple<cv::Scalar, cv::Scalar>> m_color_ranges;
        vector<cv::RotatedRect> m_prev_rects;
        cv::Scalar m_color;
        cv::Size m_size;
        cv::Mat findColor(cv::Mat input, Room room);
		cv::Mat findForeground(cv::Mat input, cv::Mat mask);
        vector<cv::RotatedRect> findGrayRects(cv::Mat mask);
        vector<cv::RotatedRect> findBinaryRects(cv::Mat mask);
};

std::vector<unsigned char> find_unique(const cv::Mat& input, bool sort = false);
cv::RotatedRect findBestFitRect(cv::Mat mask);
cv::Mat ransam(cv::Mat mask, int size);
cv::RotatedRect getErrorEllipse(cv::Point2f mean, cv::Mat covmat);
double cosAngle(cv::Point pt1, cv::Point pt2, cv::Point pt0);
cv::Mat getRotatedRectROI(cv::Mat input, cv::RotatedRect rect);
float getFillRate(cv::Mat input);
cv::Mat drawRotatedRect(cv::Mat input, cv::RotatedRect rect, cv::Scalar color);
vector<ColorRect> findColorRects(cv::Mat input, tuple<cv::Scalar, cv::Scalar> colors, cv::Scalar rect_color);
vector<ColorRect> findColorRects(cv::Mat input, cv::Scalar color1, cv::Scalar color2, cv::Scalar rect_color);
