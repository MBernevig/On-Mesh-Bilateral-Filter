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
 * @brief Calculate barycentric coordinates for a point with respect to a triangle.
 * @param p Point to find barycentric coordinates for.
 * @param a First vertex of the triangle.
 * @param b Second vertex of the triangle.
 * @param c Third vertex of the triangle.
 * @return Barycentric coordinates of point p.
 */
Vector3 barycentricCoordinates(const Vector3& p, const Vector3& a, const Vector3& b, const Vector3& c);

/**
 * @brief Interpolate texture coordinates for a point on a face.
 * @param face The face structure containing texture coordinates.
 * @param p The point on the face.
 * @return Interpolated texture coordinates.
 */
Vector2 interpolateTextureCoordinates(const FaceStructure& face, Vector3 p);

/**
 * @brief Get the face structure corresponding to a surface point.
 * @param sp The surface point.
 * @param faceStructures List of face structures.
 * @return The face structure containing the surface point.
 */
FaceStructure getFaceStructure(SurfacePoint sp, const std::vector<FaceStructure>& faceStructures);

/**
 * @brief Generate a face structure from corners and geometry data.
 * @param c0 First corner.
 * @param c1 Second corner.
 * @param c2 Third corner.
 * @param geometry Geometry data for vertex positions.
 * @param texCoords Texture coordinates data for corners.
 * @return Generated face structure.
 */
FaceStructure generateFaceStructure(Corner c0, Corner c1, Corner c2, const VertexPositionGeometry& geometry, const CornerData<Vector2>& texCoords);

/**
 * @brief Generate a point structure from a surface point and geometry data.
 * @param sp The surface point.
 * @param geometry Geometry data for vertex positions.
 * @param faceStructures List of face structures.
 * @return Generated point structure.
 */
PointStructure generatePointStructure(SurfacePoint sp, const VertexPositionGeometry& geometry, const std::vector<FaceStructure>& faceStructures);

/**
 * @brief Generate a list of face structures from a mesh and geometry data.
 * @param mesh The surface mesh.
 * @param geometry Geometry data for vertex positions.
 * @param texCoords Texture coordinates data for corners.
 * @return List of generated face structures.
 */
std::vector<FaceStructure> generateFaceStructures(ManifoldSurfaceMesh* mesh, VertexPositionGeometry* geometry, CornerData<Vector2>* texCoords);

/**
 * @brief Get the 3D position from 2D texture coordinates and a face structure.
 * @param uv The 2D texture coordinates.
 * @param face The face structure.
 * @return The 3D position corresponding to the texture coordinates.
 */
Vector3 get3DPosition(Vector2 uv, const FaceStructure& face);

/**
 * @brief Find the closest point in a point cloud to a given point.
 * @param p The point to find the closest point to.
 * @param cloud The point cloud.
 * @param geometry The geometry data for the point cloud.
 * @return The closest point in the point cloud to the given point.
 */
Point getClosestPoint(Vector3 p, PointCloud& cloud, PointPositionGeometry& geometry);



#endif // POINT_CLOUD_UTILS_H
