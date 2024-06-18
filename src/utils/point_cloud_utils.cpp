#include "point_cloud_utils.h"

Vector3 barycentricCoordinates(const Vector3& p, const Vector3& a, const Vector3& b, const Vector3& c) {
    Vector3 v0 = b - a;
    Vector3 v1 = c - a;
    Vector3 v2 = p - a;
    double d00 = dot(v0, v0);
    double d01 = dot(v0, v1);
    double d11 = dot(v1, v1);
    double d20 = dot(v2, v0);
    double d21 = dot(v2, v1);
    double denom = d00 * d11 - d01 * d01;
    double v = (d11 * d20 - d01 * d21) / denom;
    double w = (d00 * d21 - d01 * d20) / denom;
    double u = 1.0 - v - w;
    return {u, v, w};
}

Vector3 barycentricCoordinates(const Vector2& p, const Vector2& a, const Vector2& b, const Vector2& c) {
    Vector2 v0 = b - a;
    Vector2 v1 = c - a;
    Vector2 v2 = p - a;
    double d00 = dot(v0, v0);
    double d01 = dot(v0, v1);
    double d11 = dot(v1, v1);
    double d20 = dot(v2, v0);
    double d21 = dot(v2, v1);
    double denom = d00 * d11 - d01 * d01;
    double v = (d11 * d20 - d01 * d21) / denom;
    double w = (d00 * d21 - d01 * d20) / denom;
    double u = 1.0 - v - w;
    return {u, v, w};
}

Vector2 interpolateTextureCoordinates(const FaceStructure& face, Vector3 p) {
    Vector3 barycentric = barycentricCoordinates(p, face.v0, face.v1, face.v2);
    return barycentric.x * face.t0 + barycentric.y * face.t1 + barycentric.z * face.t2;
}

FaceStructure getFaceStructure(SurfacePoint sp, const std::vector<FaceStructure>& faceStructures) {
    for (const FaceStructure& fs : faceStructures) {
        if (sp.face == fs.f) {
            return fs;
        }
    }
    return FaceStructure();
}

FaceStructure generateFaceStructure(Corner c0, Corner c1, Corner c2, const VertexPositionGeometry& geometry, const CornerData<Vector2>& texCoords) {
    FaceStructure fs;
    fs.f = c0.face();
    fs.v0 = geometry.vertexPositions[c0.vertex()];
    fs.v1 = geometry.vertexPositions[c1.vertex()];
    fs.v2 = geometry.vertexPositions[c2.vertex()];
    fs.t0 = texCoords[c0];
    fs.t1 = texCoords[c1];
    fs.t2 = texCoords[c2];
    return fs;
}

PointStructure generatePointStructure(SurfacePoint sp, const VertexPositionGeometry& geometry, const std::vector<FaceStructure>& faceStructures) {
    PointStructure ps;
    FaceStructure fs = getFaceStructure(sp, faceStructures);
    Vector3 p = sp.interpolate(geometry.vertexPositions);
    Vector2 t = interpolateTextureCoordinates(fs, p);
    ps.f = fs.f;
    ps.p = p;
    ps.t = t;
    return ps;
}

std::vector<FaceStructure> generateFaceStructures(ManifoldSurfaceMesh& mesh, VertexPositionGeometry& geometry, CornerData<Vector2>& texCoords) {
    std::vector<FaceStructure> faceStructures;
    for (Face f : mesh.faces()) {
        Halfedge he = f.halfedge();
        Corner c0 = he.corner();
        Corner c1 = he.next().corner();
        Corner c2 = he.next().next().corner();
        FaceStructure fs = generateFaceStructure(c0, c1, c2, geometry, texCoords);
        faceStructures.push_back(fs);
    }
    return faceStructures;
}

//given vec2 uv and face structure, return the 3d position
Vector3 get3DPosition(Vector2 uv, const FaceStructure& face) {
    Vector3 barycentric = barycentricCoordinates(uv, face.t0, face.t1, face.t2);
    return barycentric.x * face.v0 + barycentric.y * face.v1 + barycentric.z * face.v2;
}

Point getClosestPoint(Vector3 p, PointCloud& cloud, PointPositionGeometry& geometry) {
    double minDist = std::numeric_limits<double>::max();
    Point closestPoint;
    for (Point point : cloud.points()) {
        double dist = norm(geometry.positions[point] - p);
        if (dist < minDist) {
            minDist = dist;
            closestPoint = point;
        }
    }

    // std::cout<<"Closest Point to "<<p<<" is "<<geometry.positions[closestPoint]<<std::endl;

    // if(minDist > 0.012) {
    //     std::cout<<"Closest Point to "<<p<<" is "<<geometry.positions[closestPoint]<<" at distance: "<<minDist<<std::endl;
    // }
    return closestPoint;
}



// std::tuple<PointCloud, PointPositionGeometry, PointData<Vector2>> populatePointCloud(std::vector<SurfacePoint> sps, ManifoldSurfaceMesh* mesh, VertexPositionGeometry* meshGeom, CornerData<Vector2>* texCoords) {
//     std::vector<FaceStructure> faceStructures = generateFaceStructures(mesh, meshGeom, texCoords);

//     std::vector<Vector3> samplePositions;
//     std::vector<Vector2> sampleUVs;
//     for (SurfacePoint p : sps) {
//         if (p.face == Face()) continue; // skip invalid points (e.g. on non-manifold vertices)
//         Vector3 pos = p.interpolate(meshGeom->vertexPositions);
//         samplePositions.push_back(pos);
//         FaceStructure fs = getFaceStructure(p, faceStructures);
//         Vector2 uv = interpolateTextureCoordinates(fs, pos);
//         sampleUVs.push_back(uv);
//     }

//     // Create a new PointCloud with the correct number of points
//     PointCloud cloud(samplePositions.size());

//     // Initialize PointData for positions and UVs
//     PointData<Vector3> pointPos(cloud);
//     PointData<Vector2> pointUVs(cloud);

//     for (size_t i = 0; i < samplePositions.size(); i++) {
//         pointPos[cloud.point(i)] = samplePositions[i];
//         pointUVs[cloud.point(i)] = sampleUVs[i];
//     }

//     // Create PointPositionGeometry and PointData for UVs
//     PointPositionGeometry cloudGeom(cloud, pointPos);
//     PointData<Vector2> uvs(cloud);

//     for (size_t i = 0; i < sampleUVs.size(); i++) {
//         uvs[cloud.point(i)] = sampleUVs[i];
//     }

//     std::cout << "Point cloud populated" << std::endl;

//     return std::make_tuple(std::move(cloud), std::move(cloudGeom), std::move(uvs));
// }
