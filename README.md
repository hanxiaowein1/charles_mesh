# Welcome!
This project is a study project for half-edge based triangle mesh operation. If you also want to study similar knowledge, this project might be suitable for you.
# Build
Build this project might be a little tricky, so I will descript it as detailed as I can. There are four sub projects, however, currently only three of them are useful, and one of them is unit test, we will jump its compilation.
## Before start
You need to compile my personal library [hello_symengine](https://github.com/hanxiaowein1/hello_math) first. This is a history problem, I used to use it to solve multivariable extream problem. However, after watching Justin Solomon's [Shape Analysis](https://www.youtube.com/playlist?list=PLQ3UicqQtfNtUcdTMLgKSTTOiEsCw2VBW) course, I realized it's merely a Quadratic Minimization problem, then I solved it with Eigen. And the symengine will be removed from this project in the future.
## Build mesh library
This is a lib library. You can call interface from charles_mesh.h, or load/save mesh file from/to obj type mesh file.
Here are steps on how to build mesh library.
```text
1. Download vcpkg, install GTest/PkgConfig/gmp/SymEngine/Boost/Eigen.
2. modify CMAKE_TOOLCHAIN_FILE to your own vcpkg toolchain file location.
3. set CMAKE_PREFIX_PATH to you install path of hello_symengine.
```
I believe the above steps are enough to build a local mesh library. However, if you encounter problems, feel free to contact [me](hanwein2@gmail.com).
## Build mesh processing visualization tool
This executable tool is more tricky to compile, because it integrate many functions from other library. for example, [marching cubes 33](https://github.com/hanxiaowein1/hello_cg) and [mesh viewer](https://github.com/hanxiaowein1/hello_opengl) are all required to build first.

After building and install the dependent libraries, you can follow the steps bellow to complete the compilation.
```text
1. install vcpkg, then use it to intall PkgConfig/gmp/SymEngine/Boost/Qt5/Eigen packages.
2. modify the CMAKE_TOOLCHAIN_FILE value to your personal vcpkg toolchain file in CMakeLists.txt.
3. modify the CMAKE_PREFIX_PATH to the installation path of the dependent libraries.
```
I guess the previous steps would be enough to compile the mesh processing tools locally, if you have problems, feel ease to contact [me](hanwein2@gmail.com).

There is still a **compiled version** of this tool, you can find it [here](https://github.com/hanxiaowein1/charles_mesh/releases/tag/v0.2).
# Usage
We provided unit tests for nearly all functions. If you have problems on how to call those interface, you can take them as a reference.

Here are some sample code to use it locally.
```c++
#include "mesh_factory.h"
#include "charles_mesh.h"

int main()
{
    std::string obj_model_path = ".../your_model.obj";
    ObjMeshIO mesh_loader;
    auto your_mesh = mesh_loader.load_mesh(obj_model_path);
    mesh->edge_collapse(1);
    mesh->save_obj("parent_path", "model name without suffix, for example: bunny");
    return 0;
}
```
# Features
## Edge flip
We provide two functions, one is deprecated because it's not safe.
```c++
// deprecated, it has no intersection detect
Mesh<VData>::edge_flip(/*...*/HalfEdge/*...*/);
// if there is intersection after flip, it will return false
Mesh<VData>::edge_flip_with_intersection_detect(/*...*/HalfEdge/*...*/);
```
## Edge collapse
We provide three functions.
```c++
// collapse by Quadric Error Metrics(QEM), we will pick the edge with largest metrics to collapse first
void edge_collapse(int collapse_times = 10);
// TODO: currently still under development
void edge_collapse(std::shared_ptr<HalfEdge<VData>> half_edge);
// detect if an edge can be collapsed, false represents disability.
bool edge_collapse_intersection_detect(std::shared_ptr<HalfEdge<VData>> half_edge);
```
## Mesh simplification
We provide one function
```c++
// collapse by QEM, we will pick the edge with largest metrics to collapse first
void edge_collapse(int collapse_times = 10);
```
**Notice** that current mesh simplication is extremely **slow**. Previously I used Eigen to replace symengine to solve multivariable extream problems, it helped to speed up 18x times(**MATH IS FASCINATING**) to solve QEM problem. But after this, we still encountered another efficiency bottleneck, which is the bvh tree. It will be called a greate many times, we are working on it and see if there is a good algorithm to cut the call times of it or just speed up its own speed.
## Bvh tree
We provide bvh tree structure to speed up the intersection detection. This code can be found in the file charles_bvh.h.

# Output Examples
<figure style="display: inline-block; text-align: center">
  <div>
    <img src="image/origin_bunny.png" style="width: 50%; height: auto;"/>
    <img src="image/origin_bunny_info.png" style="width: 30%; height: auto;"/>
  </div>

  <br>
  <img src="image/simplified_bunny.png" style="width: 50%; height: auto;"/>
  <img src="image/simplified_bunny_info.png" style="width: 30%; height: auto;"/>
  <figcaption style=" margin-top: 8px; text-align: center; font-style: italic; color: #666;">original bunny <b>VS</b> simplified bunny(10 edges are simplified)</figcaption>
</figure>

# Future Roadmap
```text
laplacian;
geodesic;
curvature;
and something else...
```