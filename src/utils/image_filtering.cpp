#include "image_filtering.h"



float gaussianWeight(float distance, float sigma) {
    return exp(-(distance*distance) / (2 * sigma * sigma));
}

cv::Point uvToPixel(const Vector2& uv, const cv::Mat& image) {
    return cv::Point(uv.x * image.cols, (1-uv.y) * image.rows);
}

Vector2 uvToVec2(const Vector2& p, const cv::Mat& image) {
    return Vector2{p.x / (double)image.cols, 1 - p.y / (double)image.rows};
}

Vector2 pixelToUV(const cv::Point& p, const cv::Mat& image) {
    return Vector2{(double)p.x / image.cols, 1 - (double)p.y / image.rows};
}

bool isInTriangle(const Vector2& p, const Vector2& a, const Vector2& b, const Vector2& c) {
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
    return (v >= 0) && (w >= 0) && (v + w <= 1);
}

std::tuple<cv::Point, cv::Point> boundingBox(FaceStructure fs, const cv::Mat& image) {
    cv::Point p0 = uvToPixel(fs.t0, image);
    cv::Point p1 = uvToPixel(fs.t1, image);
    cv::Point p2 = uvToPixel(fs.t2, image);
    // std::cout<<p0<<std::endl;
    cv::Point min = cv::Point(std::min(p0.x, std::min(p1.x, p2.x)) - 1, std::min(p0.y, std::min(p1.y, p2.y)) - 1);
    cv::Point max = cv::Point(std::max(p0.x, std::max(p1.x, p2.x)) + 1, std::max(p0.y, std::max(p1.y, p2.y)) + 1) ;

    //clamp  min
    min.x = std::max(0, min.x);
    min.y = std::max(0, min.y);

    //clamp max
    max.x = std::min(image.cols - 1, max.x);
    max.y = std::min(image.rows - 1, max.y);

    return std::make_tuple(min, max);
}

cv::Vec3b applyGaussianForTexel(const cv::Mat& image, cv::Point& p, Vector3& pos, PointCloud& pCloud, PointPositionGeometry& geom, PointData<Vector2>& uvs, PointCloudHeatSolver& phsolver, double sigmaSpatial, double maxDistance) {
    Point closestPoint = getClosestPoint(pos, pCloud, geom);

    PointData<double> distances = phsolver.computeDistance(closestPoint);

    cv::Vec3b imageColor = image.at<cv::Vec3b>(p);
    double initialWeight = gaussianWeight(0, sigmaSpatial);

    Vector3 color = Vector3{imageColor[0], imageColor[1], imageColor[2]} * initialWeight;

    // std::cout<<"Image Color:"<<imageColor<<std::endl;
    // std::cout<<"Initial Color:"<<color<<std::endl;

    double totalWeight = initialWeight;

    for(Point p : pCloud.points()) {
        //Discard if over distance
        if(distances[p] > maxDistance) {
            continue;
        }
        double d = distances[p];
        double weight = gaussianWeight(d, sigmaSpatial);
        Vector2 uv = uvs[p];

        cv::Point pixel = uvToPixel(uv, image);
        cv::Vec3b texelColor = image.at<cv::Vec3b>(pixel);
        Vector3 pixelColor = Vector3{texelColor[0], texelColor[1], texelColor[2]};

        color += pixelColor * weight;
        totalWeight += weight;


        // std::cout<<"Weight:"<<weight<<std::endl;
        // std::cout<<"Pixel Color:"<<pixelColor<<std::endl;
        // std::cout<<"Color:"<<color<<std::endl;
        // std::cout<<"Total Weight:"<<totalWeight<<std::endl;

    }

    // std::cout<<"Final Color:"<<color<<std::endl;
    // std::cout<<"Final Total Weight:"<<totalWeight<<std::endl;
    // std::cout<<"Final Color/Total Weight:"<<color / totalWeight<<std::endl;

    Vector3 finalColor = color / totalWeight;

    //clamp the values
    finalColor.x = std::min(255.0, std::max(0.0, finalColor.x));
    finalColor.y = std::min(255.0, std::max(0.0, finalColor.y));
    finalColor.z = std::min(255.0, std::max(0.0, finalColor.z));

    //convert to cv::Vec3b
    cv::Vec3b result = cv::Vec3b{finalColor.x, finalColor.y, finalColor.z};

    // std::cout<<"Result:"<<result<<std::endl;

    return result;
}

void processFaceStructure(cv::Mat& image, cv::Mat& result, FaceStructure fs, PointCloud& pCloud, PointPositionGeometry& geom, PointData<Vector2>& uvs, PointCloudHeatSolver& phsolver, double sigmaSpatial, double maxDistance) {
    cv::Point min, max;

    // std::cout<<"Point Cloud Size:"<<pCloud.nPoints()<<std::endl;

    std::tie(min, max) = boundingBox(fs, image);

    cv::Point pv0 = uvToPixel(fs.t0, image);
    cv::Point pv1 = uvToPixel(fs.t1, image);
    cv::Point pv2 = uvToPixel(fs.t2, image);

    Vector2 v0 = Vector2{pv0.x,pv0.y};
    Vector2 v1 = Vector2{pv1.x,pv1.y};
    Vector2 v2 = Vector2{pv2.x,pv2.y};

    for(int i = min.x; i <= max.x; i++) {
        for(int j = min.y; j <= max.y; j++) {
            cv::Point p(i, j);

            if(isInTriangle(Vector2{(double)i,(double)j}, v0, v1, v2)) {
                Vector3 pos = get3DPosition(pixelToUV(p, image), fs);
                cv::Vec3b color = applyGaussianForTexel(image, p, pos, pCloud, geom, uvs, phsolver, sigmaSpatial, maxDistance);
                result.at<cv::Vec3b>(p) = color;
            }
        }
    }
}

cv::Mat applyGaussianFilterForMesh(cv::Mat& image, ManifoldSurfaceMesh& mesh, VertexPositionGeometry& geometry, CornerData<Vector2>& texCoords, PointCloud& pCloud, PointPositionGeometry& geom, PointData<Vector2>& uvs, double sigmaSpatial, double maxDistance) {
    std::vector<FaceStructure> faceStructures = generateFaceStructures(mesh, geometry, texCoords);
    PointCloudHeatSolver phsolver(pCloud, geom);
    cv::Mat result = image.clone();

    for(FaceStructure fs : faceStructures) {
        processFaceStructure(image, result, fs, pCloud, geom, uvs, phsolver, sigmaSpatial, maxDistance);
        std::cout<<"Processed face: "<<fs.t0<<" "<<fs.t1<<" "<<fs.t2<<std::endl;
    }

    // processFaceStructure(image, result, faceStructures[0], pCloud, geom, uvs, phsolver, sigmaSpatial, maxDistance);

    return result;
}

// std::queue<int> faceStructureIndices;
// std::mutex mtx;
// std::condition_variable cv;

// cv::Mat applyGaussianFilterMultithreaded(cv::Mat& image, ManifoldSurfaceMesh& mesh, VertexPositionGeometry& geometry, CornerData<Vector2>& texCoords, PointCloud& pCloud, PointPositionGeometry& geom, PointData<Vector2>& uvs, double sigmaSpatial, double maxDistance) {
//     int numThreads = std::thread::hardware_concurrency();
//     std::vector<std::thread> threads;


// }