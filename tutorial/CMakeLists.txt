# Choose which chapters to compile
# Download tutorial data
include(libigl_tutorial_data)

if(NOT TARGET igl::glfw)
    message(WARNING "Most tutorial executables depend on OpenGL and glfw. Use `cmake -DLIBIGL_GLFW=ON ..`")
endif()



igl_add_tutorial(FastWindingNumber_CA igl::glfw igl_copyleft::cgal)
igl_copy_dll(FastWindingNumber_CA)
