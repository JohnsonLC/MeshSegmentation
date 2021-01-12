FIND_PACKAGE(OpenGL REQUIRED)
FIND_PATH(OPENGL_INCLUDE_SUBDIR
        NAMES gl.h
        PATHS ${OPENGL_INCLUDE_DIR}
        PATH_SUFFIXES GL OpenGL Headers
        NO_DEFAULT_PATH)

FIND_PACKAGE(GLUT REQUIRED)
FIND_PATH(GLUT_INCLUDE_SUBDIR
        NAMES glut.h
        PATHS ${GLUT_INCLUDE_DIR}
        PATH_SUFFIXES GL GLUT Headers
        NO_DEFAULT_PATH)

FIND_PACKAGE(GLEW REQUIRED)

find_package(Eigen3 REQUIRED)
message(STATUS "Eigen3 found: ${EIGEN3_INCLUDE_DIRS}")
message(STATUS "Eigen3 libs: ${EIGEN3_LIBRARIES}")

SET(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_FLAGS "-g -O3 -Wno-deprecated-declarations -Wno-unused-result")

add_executable(mesh_segment main.cpp init_shader.cpp)
target_link_libraries(mesh_segment
    ${OPENGL_LIBRARIES}
    ${GLUT_LIBRARY} 
    ${GLEW_LIBRARIES}
    /usr/lib/libassimp.so
    /usr/lib/x86_64-linux-gnu/libglut.so
)
target_include_directories(mesh_segment PRIVATE include ${OPENGL_INCLUDE_SUBDIR}  ${GLUT_INCLUDE_SUBDIR} ${GLEW_INCLUDE_DIRS})

add_executable(test_assimp test_assimp.cpp)
target_link_libraries(test_assimp
    ${OPENGL_LIBRARIES}
    ${GLUT_LIBRARY} 
    ${GLEW_LIBRARIES}
    /usr/lib/libassimp.so
)
target_include_directories(test_assimp PRIVATE include ${OPENGL_INCLUDE_SUBDIR}  ${GLUT_INCLUDE_SUBDIR} ${GLEW_INCLUDE_DIRS})