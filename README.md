# 2D Impulse Based Physics Engine

## Screenshots
![drag](https://github.com/Ownfos/physics-engine-rm/blob/main/images/drag.gif)
![spawn](https://github.com/Ownfos/physics-engine-rm/blob/main/images/spawn.gif)
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