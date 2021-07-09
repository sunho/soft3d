# soft3d

3D renderer on cpu supporting raytracer/rasterizer

## Screenshots

|  | |
| :----: | :-----: |
| ![](resources/screen2.png) | ![](resources/screen.png) |
| ![](resources/screen3.png) | ![](resources/screen4.png) |

## Features

These features are all built from scrath by referring to Fundamentals of Computer Graphics.
- Phong shading
- Reflection
- Shadow
- Soft shadow
- Dielectric material (glass, water)
- 2D curves (Bezier/bspline)
- Image filters including Gaussian/bspline filter
- Antialiased texture sampling (spherial/vertex uv)
- Basic graphics pipeline
- Perspective correction
- Parallel shading

## Build 
```
git clone --recursive https://github.com/sunho/soft3d
cd soft3d
cmake --build build
```
build folder will contain the build file (visual studio project, xcode project, makefile, etc)
