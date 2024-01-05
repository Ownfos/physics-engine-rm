add_executable(${PROJECT_NAME} main.cpp Circle.cpp ConvexPolygon.cpp)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_20)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/include)

# Link SFML
find_package(SFML COMPONENTS graphics CONFIG REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE sfml-graphics)

# Link ImGUI
find_package(ImGui-SFML CONFIG REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE ImGui-SFML::ImGui-SFML)