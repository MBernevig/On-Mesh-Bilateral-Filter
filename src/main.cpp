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
#include <filesystem>

#include "utils/point_cloud_utils.h"
#include "utils/image_filtering.h"

using namespace geometrycentral;
using namespace geometrycentral::surface;
using namespace geometrycentral::pointcloud;

// == Geometry-central data
std::unique_ptr<ManifoldSurfaceMesh> mesh;
std::unique_ptr<VertexPositionGeometry> geometry;
std::unique_ptr<CornerData<Vector2>> texCoords;

std::unique_ptr<PointCloud> cloud;
std::unique_ptr<PointPositionGeometry> cloudGeometry;
PointData<Vector2> cloudTexCoords;

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

//Gaussian param
double sigmaSpatial = 0.01;
double maxDistance = 0.15;
double sigmaRange = 70;

std::unique_ptr<PointCloudHeatSolver> phsolver;


int pointIndex = 0;

/**
 * @brief Samples points from a mesh using Poisson disk sampling and visualizes the result.
 * 
 * This function performs Poisson disk sampling on a mesh to generate a set of sample points. It then
 * converts these sample points into 3D positions and corresponding UV coordinates. The points are stored
 * in a point cloud and visualized using Polyscope.
 */
void sampleMesh() {
    polyscope::warning("Sampling mesh");

    //Free the previous point cloud
    cloud.reset();
    cloudGeometry.reset();
    phsolver.reset();

    //clear previous point cloud visualization
    if(psCloud) {
        polyscope::removePointCloud("Sampled Points");
        psCloud = nullptr;
    }
    

    // Assuming `mesh` and `geometry` are already defined and initialized
    PoissonDiskSampler pds(*mesh, *geometry);
    std::vector<SurfacePoint> samples = pds.sample(rCoef, kCandidates);

    // Container to hold the 3D positions of the samples
    std::vector<Vector3> samplePositions;
    std::vector<Vector2> sampleUVs;

    std::vector<FaceStructure> faceStructures = generateFaceStructures(*mesh, *geometry, *texCoords);

    // Convert SurfacePoints to 3D positions
    for (SurfacePoint p : samples) {
        if (p.face == Face()) continue; // skip invalid points 
        Vector3 pos = p.interpolate(geometry->vertexPositions);
        samplePositions.push_back(pos);
        FaceStructure fs = getFaceStructure(p, faceStructures);
        Vector2 uv = interpolateTextureCoordinates(fs, pos);
        sampleUVs.push_back(uv);
    }

    auto pCloud = std::make_unique<PointCloud>(samplePositions.size());
    std::cout << pCloud->nPoints() << std::endl;

    PointData<Vector3> pointPos(*pCloud);
    PointData<Vector2> pointUVs(*pCloud);

    for(size_t i = 0; i < samplePositions.size(); i++) {
        pointPos[pCloud->point(i)] = samplePositions[i];
        pointUVs[pCloud->point(i)] = sampleUVs[i];
    }

    auto pCloudGeometry = std::make_unique<PointPositionGeometry>(*pCloud, pointPos);
    auto phsolver1 = std::make_unique<PointCloudHeatSolver>(*pCloud, *pCloudGeometry);

    cloud = std::move(pCloud);
    cloudGeometry = std::move(pCloudGeometry);
    phsolver = std::move(phsolver1);
    cloudTexCoords = std::move(pointUVs);

    psCloud = polyscope::registerPointCloud("Sampled Points", pointPos);

    // Show the visualization
    polyscope::show();
}



/**
 * @brief Computes geodesic distances from a specific point in the point cloud.
 * 
 * This function calculates the geodesic distances from a given point in the point cloud to all other
 * points using the heat method. The distances are visualized as a scalar quantity in Polyscope.
 * 
 * @param pointId The ID of the point from which to compute geodesic distances.
 */
void computeGeodesicsForPoint(int pointId) {
    PointData<double> distances = phsolver->computeDistance(cloud->point(pointId));

    psCloud->addScalarQuantity("Geodesic Distance", distances);
    polyscope::show();
}


/**
 * @brief Outputs the UV coordinates of a specific point in the point cloud.
 * 
 * This function retrieves and prints the UV coordinates of a specified point in the point cloud.
 * The UV coordinates are printed to the standard output.
 * 
 * @param pointId The ID of the point for which to output UV coordinates.
 */
void outputUVForPoint(int pointId) {
    Vector2 uv = cloudTexCoords[cloud->point(pointId)];
    std::cout << "UV for point " << pointId << ": " << uv.x << ", " << uv.y << std::endl;
}


/**
 * @brief Displays the current texture image.
 * 
 * This function opens a window to display the current texture image using OpenCV.
 * The window remains open until a key is pressed.
 */
void displayImage() {
    cv::imshow("Texture Image", textureImage);
    cv::waitKey(0);
}


/**
 * @brief Applies a Gaussian filter to the texture image and displays the result.
 * 
 * This function applies a Gaussian filter to the texture image, using the specified parameters
 * for the mesh and point cloud. The filtered image is displayed and saved to a file.
 */
void gaussianFilterImage() {
    cv::Mat image = textureImage.clone();
    cv::Mat filteredImage = applyGaussianFilterForMesh(image, *mesh, *geometry, *texCoords, *cloud, *cloudGeometry, cloudTexCoords, sigmaSpatial, maxDistance);
    cv::imshow("Filtered Image", filteredImage);
    // std::string filename = getNextAvailableFileName("gaussian_filtered_image.jpg");
    cv::imwrite("gaussian_image.jpg", filteredImage);
    cv::waitKey(0);
}


/**
 * @brief Applies a bilateral filter to the texture image and displays the result.
 * 
 * This function applies a bilateral filter to the texture image, using the specified parameters
 * for the mesh and point cloud. The filtered image is displayed and saved to a file. Note that
 * this process can be time-consuming for large textures and many samples.
 */
void bilateralFilterImage() {
    //takes around 25 mins for 12 face and 65k samples 256x256 texture
    cv::Mat image = textureImage.clone();
    cv::Mat filteredImage = applyBilateralFilterForMesh(image, *mesh, *geometry, *texCoords, *cloud, *cloudGeometry, cloudTexCoords, sigmaSpatial, maxDistance, sigmaRange);
    cv::imshow("Filtered Image", filteredImage);
    cv::imwrite("bilat_image.jpg", filteredImage);
    cv::waitKey(0);
}


void myCallback() {

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

    ImGui::Separator();
    ImGui::Text("Sampling parameters, recommended values: \nrCoef = 0.01, kCandidates = 30");
    ImGui::InputFloat("rCoef", &rCoef);
    ImGui::SliderInt("kCandidates", &kCandidates, 1, 100);
    if (ImGui::Button("sample mesh")) {
        sampleMesh();
    }

    ImGui::Separator();
    ImGui::Text("After sampling:");
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

    if(ImGui::Button("Output UV")) {
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
        outputUVForPoint(pointIndex);
    }

    ImGui::InputDouble("sigmaSpatial", &sigmaSpatial);
    ImGui::InputDouble("maxDistance", &maxDistance);
    if(ImGui::Button("Gaussian filter image")) {
        gaussianFilterImage();
    }

    ImGui::InputDouble("sigmaRange", &sigmaRange);
    if(ImGui::Button("Bilateral filter image")) {
        bilateralFilterImage();
    }

    ImGui::Separator();
    ImGui::Text("Display texture image");
    if (ImGui::Button("Display Image")) {
        displayImage();
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

    std::tie(mesh, geometry, texCoords) = readParameterizedManifoldSurfaceMesh(args::get(inputFilename));

    // Register the mesh with polyscope
    psMesh = polyscope::registerSurfaceMesh(
        polyscope::guessNiceNameFromPath(args::get(inputFilename)),
        geometry->inputVertexPositions, mesh->getFaceVertexList(),
        polyscopePermutations(*mesh));

    // Give control to the polyscope gui
    polyscope::show();

    return EXIT_SUCCESS;
}