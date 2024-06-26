# On-Mesh Bilateral Filter
On-Mesh Bilateral Filter, created with [OpenCV](https://opencv.org/) [geometry-central](http://geometry-central.net/) and [Polyscope](http://polyscope.run/).


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

### Edit the code

Modify the main file `src/main.cpp` to start implementing your own algorithms. `CMakeLists.txt` contains a few comments for adding additional files.  
`src/utils` contains the main meat of the functions. `src/utils/image_filtering.cpp` contains the implementation of both the Bilateral Filter and the Gaussian Filter.

## Templates used

This project was built using the template [gc-polyscope-project-template](https://github.com/nmwsharp/gc-polyscope-project-template/tree/master).
