cmake_minimum_required(VERSION 3.11)
project(rmagine_viewer)

add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)

set(CPM_USE_LOCAL_PACKAGES TRUE)

include(cmake/CPM.cmake)

CPMAddPackage(
    NAME rmagine
    GIT_REPOSITORY git@gitlab.informatik.uni-osnabrueck.de:ba_fageers/rmagine.git
    GIT_TAG testTag
    OPTIONS
        "BUILD_TESTS OFF"
        "BUILD_TOOLS OFF"
)

CPMAddPackage(
    NAME polyscope 
    GITHUB_REPOSITORY nmwsharp/polyscope
    VERSION 2.4.0
    OPTIONS
        "POLYSCOPE_BACKEND_OPENGL3_GLFW ON"
        "POLYSCOPE_BACKEND_OPENGL3_EGL OFF"
        "POLYSCOPE_BACKEND_OPENGL_MOCK OFF"
        "BUILD_TESTING OFF"
)

CPMAddPackage(
    NAME portable-file-dialogs
    GITHUB_REPOSITORY samhocevar/portable-file-dialogs
    GIT_TAG 0.1.0
)

add_executable(rmagine_viewer src/rmagine_viewer.cpp)
target_link_libraries(rmagine_viewer
    polyscope
    rmagine::core
    rmagine::embree
    rmagine::ouster
    portable_file_dialogs)

add_executable(rmagine_viewer_optix src/rmagine_viewer.cpp)
target_compile_definitions(rmagine_viewer_optix PUBLIC WITH_OPTIX)
target_link_libraries(rmagine_viewer_optix
    polyscope
    rmagine::core
    rmagine::cuda
    rmagine::optix
    rmagine::ouster
    portable_file_dialogs)

add_executable(rmagine_multi_viewer src/rmagine_multi_viewer.cpp)
target_link_libraries(rmagine_multi_viewer
    polyscope
    rmagine::core
    rmagine::embree
    rmagine::cuda
    rmagine::optix
    rmagine::vulkan
    rmagine::ouster
    portable_file_dialogs)
