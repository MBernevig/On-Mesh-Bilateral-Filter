#ifndef POINT_CLOUD_UTILS_H
#define POINT_CLOUD_UTILS_H

#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/meshio.h"
#include "geometrycentral/surface/vertex_position_geometry.h"
#include "geometrycentral/surface/surface_point.h"
#include "geometrycentral/surface/poisson_disk_sampler.h"
#include "geometrycentral/surface/heat_method_distance.h"

#include "geometrycentral/pointcloud/point_cloud.h"
#include "geometrycentral/pointcloud/point_position_geometry.h"
#include "geometrycentral/pointcloud/point_cloud_io.h"
#include "geometrycentral/pointcloud/sample_cloud.h"

#include "geometrycentral/surface/direction_fields.h"

#include <tuple>

#include "Eigen/Core"

using namespace geometrycentral;
using namespace geometrycentral::surface;
using namespace geometrycentral::pointcloud;

/***********************************************************************************
                                                                                   *
                        Helper Functions to populate point clouds                  *
                                                                                   *
************************************************************************************/

/**
 * @brief Structure to represent a face with its vertices and texture coordinates.
 * @param f The face.
 * @param v0 First vertex position.
 * @param v1 Second vertex position.
 * @param v2 Third vertex position.
 * @param t0 Texture coordinate at the first vertex.
 * @param t1 Texture coordinate at the second vertex.
 * @param t2 Texture coordinate at the third vertex.
 */
struct FaceStructure {
    Face f;
    Vector3 v0;
    Vector3 v1;
    Vector3 v2;
    Vector2 t0;
    Vector2 t1;
    Vector2 t2;
};

/**
 * @brief Structure to represent a point on a face with its texture coordinate.
 * @param f The face.
 * @param p Point position.
 * @param t Texture coordinate of the point.
 */
struct PointStructure {
    Face f;
    Vector3 p;
    Vector2 t;
};

/**
 * @brief Computes the barycentric coordinates of a point with respect to a triangle in 3D space.
 * 
 * This function calculates the barycentric coordinates (u, v, w) for a point p with respect to a
 * triangle defined by vertices a, b, and c. These coordinates are useful for interpolation and 
 * determining point inclusion within the triangle.
 * 
 * @param p The point for which to compute barycentric coordinates.
 * @param a The first vertex of the triangle.
 * @param b The second vertex of the triangle.
 * @param c The third vertex of the triangle.
 * @return The barycentric coordinates as Vector3.
 */
Vector3 barycentricCoordinates(const Vector3& p, const Vector3& a, const Vector3& b, const Vector3& c);

/**
 * @brief Computes the barycentric coordinates of a point with respect to a triangle in 2D space.
 * 
 * This function calculates the barycentric coordinates (u, v, w) for a point p with respect to a
 * triangle defined by vertices a, b, and c. These coordinates are useful for interpolation and 
 * determining point inclusion within the triangle.
 * 
 * @param p The point for which to compute barycentric coordinates.
 * @param a The first vertex of the triangle.
 * @param b The second vertex of the triangle.
 * @param c The third vertex of the triangle.
 * @return The barycentric coordinates as Vector3.
 */
Vector3 barycentricCoordinates(const Vector2& p, const Vector2& a, const Vector2& b, const Vector2& c);

/**
 * @brief Interpolates the texture coordinates for a point inside a face structure.
 * 
 * This function uses barycentric coordinates to interpolate the texture coordinates
 * for a given point p inside a face structure. It calculates the weighted average
 * of the texture coordinates at the triangle vertices.
 * 
 * @param face The face structure containing triangle vertices and texture coordinates.
 * @param p The point inside the triangle.
 * @return The interpolated texture coordinates as Vector2.
 */
Vector2 interpolateTextureCoordinates(const FaceStructure& face, Vector3 p);

/**
 * @brief Retrieves the face structure corresponding to a given surface point.
 * 
 * This function searches through a list of face structures to find the one
 * that corresponds to the specified surface point. It matches the face ID
 * of the surface point with the face ID of the face structures.
 * 
 * @param sp The surface point for which to find the face structure.
 * @param faceStructures The list of face structures to search.
 * @return The corresponding face structure.
 */
FaceStructure getFaceStructure(SurfacePoint sp, const std::vector<FaceStructure>& faceStructures);

/**
 * @brief Generates a face structure from corners, geometry, and texture coordinates.
 * 
 * This function creates a face structure by extracting vertex positions and texture coordinates
 * from the specified corners, using the provided geometry and texture coordinate data.
 * 
 * @param c0 The first corner of the face.
 * @param c1 The second corner of the face.
 * @param c2 The third corner of the face.
 * @param geometry The vertex position geometry.
 * @param texCoords The texture coordinates for the corners.
 * @return The generated face structure.
 */
FaceStructure generateFaceStructure(Corner c0, Corner c1, Corner c2, const VertexPositionGeometry& geometry, const CornerData<Vector2>& texCoords);

/**
 * @brief Generates a point structure from a surface point, geometry, and face structures.
 * 
 * This function creates a point structure by interpolating the position and texture coordinates
 * of a surface point using the provided geometry and face structures.
 * 
 * @param sp The surface point for which to generate the point structure.
 * @param geometry The vertex position geometry.
 * @param faceStructures The list of face structures.
 * @return The generated point structure.
 */
PointStructure generatePointStructure(SurfacePoint sp, const VertexPositionGeometry& geometry, const std::vector<FaceStructure>& faceStructures);

/**
 * @brief Generates face structures for a mesh using geometry and texture coordinates.
 * 
 * This function creates a list of face structures by iterating over the faces of a mesh,
 * extracting vertex positions and texture coordinates for each face, and storing them
 * in face structures.
 * 
 * @param mesh The manifold surface mesh.
 * @param geometry The vertex position geometry.
 * @param texCoords The texture coordinates for the mesh corners.
 * @return A vector of generated face structures.
 */
std::vector<FaceStructure> generateFaceStructures(ManifoldSurfaceMesh& mesh, VertexPositionGeometry& geometry, CornerData<Vector2>& texCoords);

/**
 * @brief Computes the 3D position corresponding to UV coordinates within a face structure.
 * 
 * This function calculates the 3D position of a point given its UV coordinates and the
 * vertices of a face structure. It uses barycentric coordinates for the computation.
 * 
 * @param uv The UV coordinates of the point.
 * @param face The face structure containing triangle vertices and texture coordinates.
 * @return The computed 3D position as Vector3.
 */
Vector3 get3DPosition(Vector2 uv, const FaceStructure& face);

/**
 * @brief Finds the closest point in a point cloud to a given position.
 * 
 * This function iterates through the points in a point cloud to find the one
 * that is closest to the specified 3D position. It returns the closest point.
 * 
 * @param p The 3D position to which the closest point is to be found.
 * @param cloud The point cloud containing the points.
 * @param geometry The point position geometry.
 * @return The closest point in the point cloud.
 */
Point getClosestPoint(Vector3 p, PointCloud& cloud, PointPositionGeometry& geometry);



#endif // POINT_CLOUD_UTILS_H
