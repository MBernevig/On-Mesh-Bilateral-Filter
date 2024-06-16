#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/meshio.h"
#include "geometrycentral/surface/vertex_position_geometry.h"
#include "geometrycentral/surface/surface_point.h"
#include "geometrycentral/surface/poisson_disk_sampler.h"
#include "geometrycentral/surface/heat_method_distance.h"

#include "geometrycentral/pointcloud/point_cloud.h"
#include "geometrycentral/pointcloud/point_position_geometry.h"
#include "geometrycentral/pointcloud/point_cloud_io.h"
#include "geometrycentral/pointcloud/point_cloud.h"
#include "geometrycentral/pointcloud/sample_cloud.h"
#include "geometrycentral/pointcloud/point_cloud_heat_solver.h"

#include "geometrycentral/surface/direction_fields.h"

#include "Eigen/Core"

#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/point_cloud.h"

#include "args/args.hxx"
#include "imgui.h"

#include "stb_image.h"
#include "opencv2/opencv.hpp"

#include <memory>
#include <vector>
#include <iostream>

using namespace geometrycentral;
using namespace geometrycentral::surface;
using namespace geometrycentral::pointcloud;

// == Geometry-central data
std::unique_ptr<ManifoldSurfaceMesh> mesh;
std::unique_ptr<VertexPositionGeometry> geometry;
std::unique_ptr<CornerData<Vector2>> texCoords;

std::unique_ptr<PointCloud> cloud;
std::unique_ptr<PointPositionGeometry> cloudGeometry;

// == Image data
cv::Mat textureImage;

// Polyscope visualization handle, to quickly add data to the surface
polyscope::SurfaceMesh *psMesh;

polyscope::PointCloud *psCloud;

// Some algorithm parameters
float param1 = 42.0;

// Sampling params
float rCoef = 0.01;
int kCandidates = 30;
bool showSampling = false;

std::unique_ptr<PointCloudHeatSolver> phsolver;


int pointIndex = 0;
// Example computation function -- this one computes and registers a scalar
// quantity
void doWork() {
    polyscope::warning("Computing Gaussian curvature.\nalso, parameter value = " +
                       std::to_string(param1));

    geometry->requireVertexGaussianCurvatures();
    psMesh->addVertexScalarQuantity("curvature",
                                    geometry->vertexGaussianCurvatures,
                                    polyscope::DataType::SYMMETRIC);
}

void sampleMesh() {
    polyscope::warning("Sampling mesh");

    // Assuming `mesh` and `geometry` are already defined and initialized
    PoissonDiskSampler pds(*mesh, *geometry);
    std::vector<SurfacePoint> samples = pds.sample(rCoef, kCandidates);

    // Container to hold the 3D positions of the samples
    std::vector<Vector3> samplePositions;

    // Convert SurfacePoints to 3D positions
    for (SurfacePoint p : samples) {
        if(p.face == Face()) continue; // skip invalid points (e.g. on non-manifold vertices
        Vector3 pos = p.interpolate(geometry->vertexPositions);
        //get uv coordinates
        //TODO: implement this
        samplePositions.push_back(pos);
    }

    auto pCloud = std::make_unique<PointCloud>(samplePositions.size());
    std::cout << pCloud->nPoints() << std::endl;

    PointData<Vector3> pointPos(*pCloud);

    for(size_t i = 0; i < samplePositions.size(); i++) {
        pointPos[pCloud->point(i)] = samplePositions[i];
    }

    auto pCloudGeometry = std::make_unique<PointPositionGeometry>(*pCloud, pointPos);
    auto phsolver1 = std::make_unique<PointCloudHeatSolver>(*pCloud, *pCloudGeometry);

    cloud = std::move(pCloud);
    cloudGeometry = std::move(pCloudGeometry);
    phsolver = std::move(phsolver1);

    // PointData<double> distances = phsolver.computeDistance(pCloud.point(0));

    // Add the samples to the visualization as a point cloud
    // polyscope::registerPointCloud("Sampled Points", samplePositions);

    psCloud = polyscope::registerPointCloud("Sampled Points", pointPos);
    // psCloud->addScalarQuantity("Geodesic Distance", distances);

    // Show the visualization
    polyscope::show();
}

void computeGeodesicsForPoint(int pointId) {
    PointData<double> distances = phsolver->computeDistance(cloud->point(pointId));

    psCloud->addScalarQuantity("Geodesic Distance", distances);
    polyscope::show();
}


void myCallback() {

    ImGui::SliderFloat("rCoef", &rCoef, 0.f, 0.1f);
    ImGui::SliderInt("kCandidates", &kCandidates, 1, 100);
    if (ImGui::Button("sample mesh")) {
        sampleMesh();
    }

    if(ImGui::Button("Output texture coordinates")) {
        for(Corner c : mesh->corners()) {
            Vector2 z = (*texCoords)[c];
            Face f = c.face();
            Vertex v = c.vertex();
            Vector3 pos = geometry->vertexPositions[v];
            std::cout<<pos.x<<" "<<pos.y<<" "<<pos.z<<" "<<std::endl;
            std::cout<<z.x<<" "<<z.y<<std::endl;
            std::cout<<std::endl;
        }
    }

    ImGui::InputInt("Point ID", &pointIndex);
    if(ImGui::Button("Compute geodesic distance")) {
        //If psCloud is not initialized, warn and return
        if(!psCloud) {
            polyscope::warning("Point cloud not initialized, sample the mesh first");
            return;
        }
        //if pointIndex is out of bounds, warn and return
        if(pointIndex < 0 ||  pointIndex >= (int) cloud->nPoints()) {
            polyscope::warning("Point index out of bounds");
            return;
        }
        computeGeodesicsForPoint(pointIndex);
    }
}

int main(int argc, char **argv) {
    // Configure the argument parser
    args::ArgumentParser parser("geometry-central & Polyscope example project");
    args::Positional<std::string> inputFilename(parser, "mesh", "A mesh file.");
    args::Positional<std::string> textureFilename(parser, "texture", "A texture image file.");

    // Parse args
    try {
        parser.ParseCLI(argc, argv);
    } catch (args::Help &h) {
        std::cout << parser;
        return 0;
    } catch (args::ParseError &e) {
        std::cerr << e.what() << std::endl;
        std::cerr << parser;
        return 1;
    }

    // Make sure a mesh name was given
    if (!inputFilename) {
        std::cerr << "Please specify a mesh file as argument" << std::endl;
        return EXIT_FAILURE;
    }

    if(!textureFilename) {
        std::cerr << "Please specify a texture image file as argument" << std::endl;
        return EXIT_FAILURE;
    }

    // Point image to textureImage
    textureImage = cv::imread(args::get(textureFilename), cv::IMREAD_COLOR);

    polyscope::view::setUpDir(polyscope::UpDir::ZUp);
    polyscope::view::setFrontDir(polyscope::FrontDir::YFront);

    // Initialize polyscope
    polyscope::init();

    // Set the callback function
    polyscope::state::userCallback = myCallback;

    // Load mesh
    // std::tie(mesh, geometry) = readManifoldSurfaceMesh(args::get(inputFilename));
    std::tie(mesh, geometry, texCoords) = readParameterizedManifoldSurfaceMesh(args::get(inputFilename));

    // Register the mesh with polyscope
    psMesh = polyscope::registerSurfaceMesh(
        polyscope::guessNiceNameFromPath(args::get(inputFilename)),
        geometry->inputVertexPositions, mesh->getFaceVertexList(),
        polyscopePermutations(*mesh));

    // Set vertex tangent spaces
    // geometry->requireVertexTangentBasis();
    // VertexData<Vector3> vBasisX(*mesh);
    // VertexData<Vector3> vBasisY(*mesh);
    // for (Vertex v : mesh->vertices()) {
    //     vBasisX[v] = geometry->vertexTangentBasis[v][0];
    //     vBasisY[v] = geometry->vertexTangentBasis[v][1];
    // }

    // auto vField =
    //     geometrycentral::surface::computeSmoothestVertexDirectionField(*geometry);
    // psMesh->addVertexTangentVectorQuantity("VF", vField, vBasisX, vBasisY);

    // Give control to the polyscope gui
    polyscope::show();

    return EXIT_SUCCESS;
}