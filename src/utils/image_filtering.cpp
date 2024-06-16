#include <opencv2/opencv.hpp>
#include "point_cloud_utils.h"


using namespace cv;

float gaussianWeight(float distance, float sigma) {
    return exp(-(distance*distance) / (2 * sigma * sigma));
}

cv::Point uvToPixel(const Vector2& uv, const cv::Mat& image) {
    return cv::Point(uv.x * image.cols, (1-uv.y) * image.rows);
}

std::tuple<cv::Point, cv::Point> boundingBox(FaceStructure fs, const cv::Mat& image) {
    cv::Point p0 = uvToPixel(fs.t0, image);
    cv::Point p1 = uvToPixel(fs.t1, image);
    cv::Point p2 = uvToPixel(fs.t2, image);
    cv::Point min = cv::Point(std::min(p0.x, std::min(p1.x, p2.x)), std::min(p0.y, std::min(p1.y, p2.y)));
    cv::Point max = cv::Point(std::max(p0.x, std::max(p1.x, p2.x)), std::max(p0.y, std::max(p1.y, p2.y)));
    return std::make_tuple(min, max);
}


// Vector2 pixelToUV(const Point& pixel, const Mat& image) {
//     return Vector2(pixel.x / (double)image.cols, pixel.y / (double)image.rows);
// }