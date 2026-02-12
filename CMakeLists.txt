cmake_minimum_required(VERSION 3.16)
project(ArapDeformer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable libigl optional modules before fetching
set(LIBIGL_OPENGL ON CACHE BOOL "" FORCE)
set(LIBIGL_GLFW   ON CACHE BOOL "" FORCE)
set(LIBIGL_IMGUI  ON CACHE BOOL "" FORCE)

include(FetchContent)
FetchContent_Declare(
    libigl
    GIT_REPOSITORY https://github.com/libigl/libigl.git
    GIT_TAG        v2.5.0
)
FetchContent_MakeAvailable(libigl)

# Main executable
add_executable(${PROJECT_NAME}
    src/main.cpp
    src/app/viewer_app.cpp
    src/solver/arap_deformer.cpp
    src/ui/control_panel.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/src)

target_link_libraries(${PROJECT_NAME} PUBLIC
    igl::core
    igl::opengl
    igl::glfw
    igl::imgui
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
    DATA_DIR="${CMAKE_SOURCE_DIR}/data"
)
