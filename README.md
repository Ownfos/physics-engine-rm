# 2D Impulse Based Physics Engine

## What does this do?
This is basically a 2D rigidbody simulator.  
You can spawn circles and convex polygons and let them collide!  
In addition, you can dynamically create custom polygon objects  
or create a spring that connectes two arbitrary objects using right-click.

## 'impulse based'?
Collisions are handled by finding an appropriate  
impulse vector which satisfies [Newton's law of restitution](https://en.wikipedia.org/wiki/Coefficient_of_restitution).  

## Screenshots
#### dragging objects
![drag](https://github.com/Ownfos/physics-engine-rm/blob/main/images/drag.gif)
#### spawning custom polygons
![spawn](https://github.com/Ownfos/physics-engine-rm/blob/main/images/spawn.gif)
#### connecting objects with springs
![spring](https://github.com/Ownfos/physics-engine-rm/blob/main/images/spring.gif)

## How to build
### Step 1) clone vcpkg
```
git clone https://github.com/microsoft/vcpkg
./vcpkg/bootstrap-vcpkg.bat
```
### Step 2) cmake
```
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE="vcpkg/scripts/buildsystems/vcpkg.cmake"
cmake --build build
```
#### You can also use CMake extension on VSCode to configure and build.

## Dependencies
- [SFML](https://github.com/SFML/SFML)
- [ImGui-SFML](https://github.com/SFML/imgui-sfml)