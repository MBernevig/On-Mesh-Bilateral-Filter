#ifndef IMAGE_FILTERING_H
#define IMAGE_FILTERING_H

#include <opencv2/opencv.hpp>
#include "point_cloud_utils.h"
#include "geometrycentral/pointcloud/point_cloud_heat_solver.h"
#include <tuple>
#include <vector>
#include <thread>

// Function declarations
/**
 * @brief Computes the spatial weight based on the distance and sigma.
 * 
 * This function calculates the spatial weight for a given distance and standard deviation (sigma)
 * using the Gaussian function. The result can be used in filtering processes where the influence of
 * a point decreases with its distance from a reference point.
 * 
 * @param distance The distance between points.
 * @param sigma The standard deviation of the Gaussian distribution.
 * @return The computed spatial weight.
 */
float spatialWeight(float distance, float sigma);

/**
 * @brief Converts UV coordinates to pixel coordinates in the given image.
 * 
 * This function transforms UV coordinates, which typically range from 0 to 1, into pixel coordinates
 * based on the dimensions of the provided image. This is useful for mapping texture coordinates to
 * image pixels.
 * 
 * @param uv The UV coordinates.
 * @param image The image for reference dimensions.
 * @return The pixel coordinates as cv::Point.
 */
cv::Point uvToPixel(const Vector2& uv, const cv::Mat& image);

/**
 * @brief Converts pixel coordinates to UV coordinates in the given image.
 * 
 * This function transforms pixel coordinates into UV coordinates, which range from 0 to 1,
 * based on the dimensions of the provided image. This is useful for reverse-mapping pixel positions
 * to texture coordinates.
 * 
 * @param p The pixel coordinates.
 * @param image The image for reference dimensions.
 * @return The UV coordinates as Vector2.
 */
Vector2 uvToVec2(const Vector2& p, const cv::Mat& image);

/**
 * @brief Converts pixel coordinates to UV coordinates in the given image.
 * 
 * This function transforms pixel coordinates into UV coordinates, which range from 0 to 1,
 * based on the dimensions of the provided image. This is useful for reverse-mapping pixel positions
 * to texture coordinates.
 * 
 * @param p The pixel coordinates.
 * @param image The image for reference dimensions.
 * @return The UV coordinates as Vector2.
 */
Vector2 pixelToUV(const cv::Point& p, const cv::Mat& image);

/**
 * @brief Checks if a point is inside a triangle defined by three vertices.
 * 
 * This function determines if a given point lies within a triangle formed by three vertices in 2D space.
 * It uses the barycentric coordinate system to perform the check.
 * 
 * @param p The point to check.
 * @param a The first vertex of the triangle.
 * @param b The second vertex of the triangle.
 * @param c The third vertex of the triangle.
 * @return True if the point is inside the triangle, false otherwise.
 */
bool isInTriangle(const Vector2& p, const Vector2& a, const Vector2& b, const Vector2& c);

/**
 * @brief Computes the bounding box for a given face structure in the image.
 * 
 * This function calculates the axis-aligned bounding box that encloses a triangular face defined by
 * three vertices in UV space. The bounding box is clamped to the image boundaries to ensure it lies
 * within the valid pixel range.
 * 
 * @param fs The face structure containing triangle vertices.
 * @param image The image for reference dimensions.
 * @return A tuple containing the minimum and maximum points of the bounding box.
 */
std::tuple<cv::Point, cv::Point> boundingBox(FaceStructure fs, const cv::Mat& image);

/**
 * @brief Applies a Gaussian filter to a texel in the image.
 * 
 * This function filters the color of a specific texel (texture element) in an image using a Gaussian
 * filter. It computes the weighted average color of nearby texels based on their spatial distance
 * and a given sigma value, resulting in a smoothed color for the texel.
 * 
 * @param image The image to filter.
 * @param p The pixel position in the image.
 * @param pos The 3D position of the point.
 * @param pCloud The point cloud data.
 * @param geom The point position geometry.
 * @param uvs The UV coordinates data.
 * @param phsolver The heat solver for the point cloud.
 * @param sigmaSpatial The spatial sigma value for the Gaussian filter.
 * @param maxDistance The maximum distance for filtering.
 * @return The filtered color as cv::Vec3b.
 */
cv::Vec3b applyGaussianForTexel(const cv::Mat& image, cv::Point& p, Vector3& pos, PointCloud& pCloud, PointPositionGeometry& geom, PointData<Vector2>& uvs, PointCloudHeatSolver& phsolver, double sigmaSpatial, double maxDistance);

/**
 * @brief Processes a face structure and applies Gaussian filtering to its area in the image.
 * 
 * This function processes a given face structure, which is a triangle defined in UV space. It determines
 * the bounding box of the triangle, checks each pixel within this box to see if it lies inside the triangle,
 * and applies Gaussian filtering to those pixels, updating the result image.
 * 
 * @param image The original image.
 * @param result The image to store the filtered result.
 * @param fs The face structure to process.
 * @param pCloud The point cloud data.
 * @param geom The point position geometry.
 * @param uvs The UV coordinates data.
 * @param phsolver The heat solver for the point cloud.
 * @param sigmaSpatial The spatial sigma value for the Gaussian filter.
 * @param maxDistance The maximum distance for filtering.
 */
void processFaceStructure(cv::Mat& image, cv::Mat& result, FaceStructure fs, PointCloud& pCloud, PointPositionGeometry& geom, PointData<Vector2>& uvs, PointCloudHeatSolver& phsolver, double sigmaSpatial, double maxDistance);

/**
 * @brief Applies a Gaussian filter to a mesh based on its UV texture coordinates.
 * 
 * This function applies Gaussian filtering to the entire mesh represented by UV texture coordinates.
 * It generates face structures from the mesh, processes each face using Gaussian filtering, and
 * produces a filtered image.
 * 
 * @param image The original image.
 * @param mesh The manifold surface mesh.
 * @param geometry The vertex position geometry.
 * @param texCoords The texture coordinates for mesh corners.
 * @param pCloud The point cloud data.
 * @param geom The point position geometry.
 * @param uvs The UV coordinates data.
 * @param sigmaSpatial The spatial sigma value for the Gaussian filter.
 * @param maxDistance The maximum distance for filtering.
 * @return The filtered image.
 */
cv::Mat applyGaussianFilterForMesh(cv::Mat& image, ManifoldSurfaceMesh& mesh, VertexPositionGeometry& geometry, CornerData<Vector2>& texCoords, PointCloud& pCloud, PointPositionGeometry& geom, PointData<Vector2>& uvs, double sigmaSpatial, double maxDistance);

/**
 * @brief Computes the range weight based on color distance and sigma.
 * 
 * This function calculates the range weight for a given color distance and standard deviation (sigma)
 * using the Gaussian function. The result can be used in bilateral filtering processes where the
 * influence of a color decreases with its difference from a reference color.
 * 
 * @param distance The color distance between points.
 * @param sigmaRange The standard deviation of the Gaussian distribution.
 * @return The computed range weight.
 */
double rangeWeight(double distance, double sigmaRange);

/**
 * @brief Applies a bilateral filter to a texel in the image.
 * 
 * This function filters the color of a specific texel (texture element) in an image using a bilateral
 * filter. It computes the weighted average color of nearby texels based on both their spatial distance
 * and color distance, resulting in a smoothed color that preserves edges.
 * 
 * @param image The image to filter.
 * @param p The pixel position in the image.
 * @param pos The 3D position of the point.
 * @param pCloud The point cloud data.
 * @param geom The point position geometry.
 * @param uvs The UV coordinates data.
 * @param phsolver The heat solver for the point cloud.
 * @param sigmaSpatial The spatial sigma value for the Gaussian filter.
 * @param maxDistance The maximum distance for filtering.
 * @param sigmaRange The range sigma value for the bilateral filter.
 * @return The filtered color as cv::Vec3b.
 */
cv::Vec3b applyBilateralForTexel(const cv::Mat& image, cv::Point& p, Vector3& pos, PointCloud& pCloud, PointPositionGeometry& geom, PointData<Vector2>& uvs, PointCloudHeatSolver& phsolver, double sigmaSpatial, double maxDistance, double sigmaRange);

/**
 * @brief Processes a face structure and applies bilateral filtering to its area in the image.
 * 
 * This function processes a given face structure, which is a triangle defined in UV space. It determines
 * the bounding box of the triangle, checks each pixel within this box to see if it lies inside the triangle,
 * and applies bilateral filtering to those pixels, updating the result image.
 * 
 * @param image The original image.
 * @param result The image to store the filtered result.
 * @param fs The face structure to process.
 * @param pCloud The point cloud data.
 * @param geom The point position geometry.
 * @param uvs The UV coordinates data.
 * @param phsolver The heat solver for the point cloud.
 * @param sigmaSpatial The spatial sigma value for the Gaussian filter.
 * @param maxDistance The maximum distance for filtering.
 * @param sigmaRange The range sigma value for the bilateral filter.
 */
void processFaceStructureBilateral(cv::Mat& image, cv::Mat& result, FaceStructure fs, PointCloud& pCloud, PointPositionGeometry& geom, PointData<Vector2>& uvs, PointCloudHeatSolver& phsolver, double sigmaSpatial, double maxDistance, double sigmaRange);

/**
 * @brief Applies a bilateral filter to a mesh based on its UV texture coordinates.
 * 
 * This function applies bilateral filtering to the entire mesh represented by UV texture coordinates.
 * It generates face structures from the mesh, processes each face using bilateral filtering, and
 * produces a filtered image.
 * 
 * @param image The original image.
 * @param mesh The manifold surface mesh.
 * @param geometry The vertex position geometry.
 * @param texCoords The texture coordinates for mesh corners.
 * @param pCloud The point cloud data.
 * @param geom The point position geometry.
 * @param uvs The UV coordinates data.
 * @param sigmaSpatial The spatial sigma value for the Gaussian filter.
 * @param maxDistance The maximum distance for filtering.
 * @param sigmaRange The range sigma value for the bilateral filter.
 * @return The filtered image.
 */
cv::Mat applyBilateralFilterForMesh(cv::Mat& image, ManifoldSurfaceMesh& mesh, VertexPositionGeometry& geometry, CornerData<Vector2>& texCoords, PointCloud& pCloud, PointPositionGeometry& geom, PointData<Vector2>& uvs, double sigmaSpatial, double maxDistance, double sigmaRange);


#endif // IMAGE_FILTERING_H
