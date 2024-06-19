#ifndef IMAGE_FILTERING_H
#define IMAGE_FILTERING_H

#include <opencv2/opencv.hpp>
#include "point_cloud_utils.h"
#include "geometrycentral/pointcloud/point_cloud_heat_solver.h"
#include <tuple>
#include <vector>
#include <thread>

// Function declarations
float gaussianWeight(float distance, float sigma);

cv::Point uvToPixel(const Vector2& uv, const cv::Mat& image);

Vector2 uvToVec2(const Vector2& p, const cv::Mat& image);

Vector2 pixelToUV(const cv::Point& p, const cv::Mat& image);

bool isInTriangle(const Vector2& p, const Vector2& a, const Vector2& b, const Vector2& c);

std::tuple<cv::Point, cv::Point> boundingBox(FaceStructure fs, const cv::Mat& image);

cv::Vec3b applyGaussianForTexel(const cv::Mat& image, cv::Point& p, Vector3& pos, PointCloud& pCloud, PointPositionGeometry& geom, PointData<Vector2>& uvs, PointCloudHeatSolver& phsolver, double sigmaSpatial, double maxDistance);

void processFaceStructure(cv::Mat& image, cv::Mat& result, FaceStructure fs, PointCloud& pCloud, PointPositionGeometry& geom, PointData<Vector2>& uvs, PointCloudHeatSolver& phsolver, double sigmaSpatial, double maxDistance);

cv::Mat applyGaussianFilterForMesh(cv::Mat& image, ManifoldSurfaceMesh& mesh, VertexPositionGeometry& geometry, CornerData<Vector2>& texCoords, PointCloud& pCloud, PointPositionGeometry& geom, PointData<Vector2>& uvs, double sigmaSpatial, double maxDistance);

cv::Mat applyBilateralFilterForMesh(cv::Mat& image, ManifoldSurfaceMesh& mesh, VertexPositionGeometry& geometry, CornerData<Vector2>& texCoords, PointCloud& pCloud, PointPositionGeometry& geom, PointData<Vector2>& uvs, double sigmaSpatial, double maxDistance, double sigmaRange);

#endif // IMAGE_FILTERING_H
