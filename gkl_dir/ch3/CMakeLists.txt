cmake_minimum_required( VERSION 2.8 )
project( Eigen )

# find_package(OPENGL REQUIRED)
include_directories("/usr/include/eigen3")

add_executable(eigenMatrix eigenMatrix.cpp)

add_executable(useGeometry useGeometry.cpp)

add_executable(exampleProblem exampleProblem.cpp)


# for adding Pangolin
find_package( Pangolin )
include_directories( ${Pangolin_INCLUDE_DIRS} )

# include_directories(/work/Learning/slambook2/3rdparty/Pangolin/include)
# include_directories(/work/Learning/slambook2/3rdparty/Pangolin/build/src/include)
add_executable(trajectoryTrack trajectoryTrack.cpp)


# # Link Pangolin library for trajectoryTrack
target_link_libraries(trajectoryTrack
    ${Pangolin_LIBRARIES}
)
