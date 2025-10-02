# 🔬 On-Mesh Bilateral Filter

<div align="center">

[![C++](https://img.shields.io/badge/C%2B%2B-00599C?style=for-the-badge&logo=c%2B%2B&logoColor=white)](https://isocpp.org/)
[![CMake](https://img.shields.io/badge/CMake-064F8C?style=for-the-badge&logo=cmake&logoColor=white)](https://cmake.org/)
[![OpenCV](https://img.shields.io/badge/OpenCV-27338e?style=for-the-badge&logo=OpenCV&logoColor=white)](https://opencv.org/)
[![License](https://img.shields.io/badge/License-MIT-green?style=for-the-badge)](LICENSE)

*A novel implementation of bilateral filtering on textures mapped to meshes*

**[📖 Research Paper](https://resolver.tudelft.nl/uuid:998d1a03-af18-4628-ba5c-aa654f154622) • [🚀 Getting Started](#-getting-started) • [📚 Documentation](#-features)**

</div>

---

## ✨ Overview

The **On-Mesh Bilateral Filter** is an innovative approach to texture filtering that bridges the gap between texture space and object space filtering. This project, developed as part of CSE3000 research at TU Delft, implements a sophisticated bilateral filtering algorithm that operates directly on mesh surfaces.

### 🛠️ Built With

- **[OpenCV](https://opencv.org/)** - Computer vision and image processing
- **[geometry-central](http://geometry-central.net/)** - Robust geometric algorithms
- **[Polyscope](http://polyscope.run/)** - 3D visualization and mesh interaction

---

## 🚀 Getting Started

### 📥 Installation

1. **Clone the repository**
   ```bash
   git clone https://github.com/MBernevig/On-Mesh-Bilateral-Filter.git
   cd On-Mesh-Bilateral-Filter
   ```

### 🔨 Build Instructions

<details>
<summary><b>🐧 Unix-like Systems (Linux/macOS)</b></summary>

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
```
</details>

<details>
<summary><b>🪟 Windows (Visual Studio)</b></summary>

1. Install [CMake](https://cmake.org/download/)
2. Generate Visual Studio solution:
   ```cmd
   mkdir build && cd build
   cmake ..
   ```
3. Open the generated `.sln` file in Visual Studio and build

</details>

### ⚙️ Dependencies

The following dependencies are automatically managed by CMake:

| Library | Purpose | Auto-installed |
|---------|---------|---------------|
| **OpenCV** | Image processing | ✅ |
| **geometry-central** | Mesh operations | ✅ |
| **Polyscope** | 3D visualization | ✅ |

> 💡 **Note**: CMake will provide detailed warnings for any missing dependencies

---

## 🎮 Usage

### Basic Command

```bash
./bin/onmeshbf /path/to/mesh.obj /path/to/texture.png
```

### 📋 Workflow

1. **🔄 Load your mesh and texture** - The application opens with your 3D model
2. **📍 Sample the mesh** - Generate sampling points using Poisson disk sampling
3. **⚙️ Configure parameters** - Adjust spatial and range sigma values
4. **🎨 Apply filtering** - Process your texture with the bilateral filter
5. **👀 View results** - Load your newly generated texture image in a 3D visualization program to see results.

---

## 🔬 Technical Overview

### 🧮 Core Algorithms

- **🎯 Poisson Disk Sampling** - Uniform point distribution across mesh surfaces
- **📐 Geodesic Distance Computation** - Heat method for accurate surface distance calculation
- **🔄 Barycentric Interpolation** - Seamless UV coordinate mapping
- **🎨 Bilateral Filtering** - Edge-preserving texture smoothing with dual-domain filtering
- **📊 Gaussian Filtering** - Spatial smoothing with customizable kernels

### 🏗️ Architecture

```
src/
├── main.cpp                 # Main application with Polyscope GUI
└── utils/
    ├── image_filtering.*    # Bilateral & Gaussian filter implementations
    └── point_cloud_utils.*  # Mesh sampling and coordinate utilities
```

### 🔬 Algorithm Details

The implementation features a novel approach to texture filtering that operates in the **mesh domain** rather than traditional texture space:

1. **Mesh Sampling**: Uses Poisson disk sampling to generate uniformly distributed points across the mesh surface
2. **UV Mapping**: Each sample point maintains both 3D position and corresponding UV texture coordinates
3. **Geodesic Distances**: Computes accurate surface distances using the heat method
4. **Spatial-Range Filtering**: Applies bilateral filtering using both geodesic distance and color similarity

### ⚙️ Key Parameters

| Parameter | Description | Recommended Value |
|-----------|-------------|-------------------|
| `rCoef` | Poisson disk sampling radius coefficient | `0.01` |
| `kCandidates` | Number of candidate points for sampling | `30` |
| `sigmaSpatial` | Spatial smoothing parameter | `0.01` |
| `maxDistance` | Maximum geodesic distance for filtering | `0.15` |
| `sigmaRange` | Color difference threshold | `70` |

---

## 🛠️ Development

### 📁 Project Structure

<details>
<summary><b>📂 Expand to see full directory structure</b></summary>

```
On-Mesh-Bilateral-Filter/
├── 📄 CMakeLists.txt           # Build configuration
├── 📄 README.md                # This file
├── 📁 src/                     # Source code
│   ├── 📄 main.cpp             # Main application entry point
│   └── 📁 utils/               # Core algorithm implementations
│       ├── 📄 image_filtering.* # Filter implementations
│       └── 📄 point_cloud_utils.* # Mesh utilities
├── 📁 deps/                    # Dependencies (auto-managed)
│   ├── 📁 geometry-central/    # Geometric algorithms
│   └── 📁 polyscope/          # 3D visualization
├── 📁 input_data/             # Sample meshes and textures
└── 📁 console/                # Additional utilities
```
</details>

### 🔧 Core Functions

| Function | Purpose | Location |
|----------|---------|----------|
| `sampleMesh()` | Poisson disk sampling of mesh surface | `main.cpp` |
| `applyBilateralFilterForMesh()` | Main bilateral filtering algorithm | `image_filtering.cpp` |
| `computeGeodesicsForPoint()` | Geodesic distance computation | `main.cpp` |
| `interpolateTextureCoordinates()` | UV coordinate interpolation | `point_cloud_utils.cpp` |

### 🎯 Customization

**Modify `src/main.cpp`** to implement your own algorithms:
- Adjust filtering parameters in the GUI
- Add new visualization features
- Implement custom sampling strategies

**Extend `src/utils/`** for new algorithms:
- `image_filtering.cpp` - Add new filter types
- `point_cloud_utils.cpp` - Enhance mesh processing utilities

---

## 📜 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## � Acknowledgments

- **[geometry-central](http://geometry-central.net/)** - Robust geometric computing library
- **[Polyscope](http://polyscope.run/)** - Beautiful 3D visualization framework
- **[OpenCV](https://opencv.org/)** - Comprehensive computer vision library
- **[gc-polyscope-project-template](https://github.com/nmwsharp/gc-polyscope-project-template/)** - Project foundation


