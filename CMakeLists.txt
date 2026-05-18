cmake_minimum_required(VERSION 3.16)
project(lib_gl_tp)

list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Libigl
include(libigl)

# Enable the target igl::glfw
igl_include(glfw)
igl_include(imgui)

# Un seul exécutable avec tous les fichiers source
add_executable(main
  src/main.cpp
  src/Creation_Primitive.cpp
  src/SDF.cpp
)

target_link_libraries(main PUBLIC igl::glfw igl::imgui)