cmake_minimum_required( VERSION 2.8 )
project( Eigen )

# find_package(OPENGL REQUIRED)
include_directories("/usr/include/eigen3")


# # for adding Pangolin
# find_package( Pangolin )
# include_directories( ${Pangolin_INCLUDE_DIRS} )

# # Link Pangolin library for trajectoryTrack
# target_link_libraries(trajectoryTrack
    # ${Pangolin_LIBRARIES}
# )

# add opencv
find_package(OpenCV REQUIRED)
set(Sophus_DIR "/usr/local/share/sophus/cmake")
find_package(Sophus REQUIRED)

find_package(Pangolin REQUIRED)
#link libraries

include_directories( ${OpenCV_INCLUDE_DIRS})

include_directories("/usr/include/Eigen3")

include_directories( ${Pangolin_INCLUDE_DIRS} )

# find_package(Sophus REQUIRED)

add_executable( jointMap jointMap.cpp)

target_link_libraries( jointMap PRIVATE 
                        Sophus::Sophus
                        ${OpenCV_LIBS}
                        ${Pangolin_LIBRARIES})

