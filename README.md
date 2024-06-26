# On-Mesh Bilateral Filter

On-Mesh Bilateral Filter, created with [OpenCV](https://opencv.org/), [geometry-central](http://geometry-central.net/) and [Polyscope](http://polyscope.run/).

The code in this repository is the result of a research project conducted as part of CSE3000 at TUDelft.

You can read more about this project in the research paper accompanying this code, at [On-Mesh Bilateral Filter: Bridging the gap between Texture and Object Space](google.com)


### Get the code

Clone the project 
```
git clone https://github.com/MBernevig/On-Mesh-Bilateral-Filter.git
```

### Build the code

**Unix-like machines**: configure (with cmake) and compile
```
cd /path/to/repository
mkdir build
cd build
cmake ..
make -j6
```

**Windows / Visual Studio**

Install CMake, and use either the CMake GUI or the command line interface (as on unix) to generate a Visual Studio solution.  Build the solution with Visual Studio.

**Dependencies**

Several dependencies need to fulfilled for OpenCV, geometry-central and Polyscope. More information regarding these can be found on their respective websites. 
CMake will warn you if a dependency is not fulfilled, and you can iteratively fix everything this way.

### Run the code

```
./bin/onmeshbf /path/to/a/mesh /path/to/a/texture
```

A new window should appear where you can see the mesh you loaded. 
You should now sample your mesh, choose appropriate spatial and range sigma values, then filter your texture image. 
The filtered texture image will appear on screen once the filtering process is finished.

### Edit the code

Modify the main file `src/main.cpp` to start implementing your own algorithms. `CMakeLists.txt` contains a few comments for adding additional files.  
`src/utils` contains the main meat of the functions. `src/utils/image_filtering.cpp` contains the implementation of both the Bilateral Filter and the Gaussian Filter.

## Templates used

This project was built using the [gc-polyscope-project-template](https://github.com/nmwsharp/gc-polyscope-project-template/tree/master).

## Additional Information

Sampling the Mesh: The `sampleMesh` function samples points from the mesh using Poisson disk sampling, converts these points into 3D positions and corresponding UV coordinates, and visualizes them using Polyscope.

Geodesic Distance Computation: The `computeGeodesicsForPoint` function calculates geodesic distances from a specified point in the point cloud and visualizes these distances using Polyscope.

UV Coordinate Output: The `outputUVForPoint` function retrieves and prints the UV coordinates of a specified point in the point cloud.

Image Display: The `displayImage` function displays the current texture image using OpenCV.

Image Filtering: The `gaussianFilterImage` and `bilateralFilterImage` functions apply Gaussian and bilateral filters to the texture image respectively, displaying and saving the filtered images.

For detailed information on these functions and more, refer to the source code in the repository.