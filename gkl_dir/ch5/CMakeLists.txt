cmake_minimum_required( VERSION 2.8 )
project( Eigen )

# find_package(OPENGL REQUIRED)
include_directories("/usr/include/eigen3")


# for adding Pangolin
find_package( Pangolin )
include_directories( ${Pangolin_INCLUDE_DIRS} )

# # Link Pangolin library for trajectoryTrack
target_link_libraries(trajectoryTrack
    ${Pangolin_LIBRARIES}
)

# add opencv
find_package(OpenCV REQUIRED)

#link libraries

include_directories( ${OpenCV_INCLUDE_DIRS})

add_executable(imageBasics imageBasics.cpp)

#target link
target_link_libraries(imageBasics PRIVATE ${OpenCV_LIBS})