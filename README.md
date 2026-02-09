# ARAP Mesh Deformer

As-Rigid-As-Possible (ARAP) mesh deformation tool built with libigl and C++.

## Dependencies

**System packages** (Ubuntu/Debian):

```bash
sudo apt-get install -y cmake g++ libglfw3-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev libxext-dev libgl-dev
```

libigl and Eigen are fetched automatically by CMake.

## Build

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
```

## Run

```bash
./build/ArapDeformer
```

Place `.obj` or `.off` mesh files in the `data/` directory for testing.

## Project Structure

```
├── CMakeLists.txt
├── src/
│   └── main.cpp
└── data/           # test meshes
```
