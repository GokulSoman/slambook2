#include <iostream>
#include <fstream>
// #include <pangolin/pangolin.h>
// #include <vector>
#include <opencv2/opencv.hpp>
// this seems to include vector as well
#include <sophus/se3.hpp>
// #include <Eigen/Dense> including sophus includes eigen as well
// atleast the ones that are used here

#include <boost/format.hpp>

using namespace std;

// using namespace Eigen;

typedef vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> TrajectoryType;

// add vector6d as it is not standad type

typedef Eigen::Matrix<double, 6, 1> Vector6d;

int main(int argc, char **argv){

    vector<cv::Mat> color_imgs, depth_imgs;
    TrajectoryType poses;

    ifstream fin("../pose.txt");

    if (!fin) {
        cerr << "Pose.txt file not found" << endl;
        return 1;
    }

    boost::format fmt("../%s/%d.%s");
    // works like placeholders

    for (int i=1; i <=5; i++){
        color_imgs.push_back(cv::imread((fmt % "color" % i % "png").str()));
        depth_imgs.push_back(cv::imread((fmt % "depth" % i % "pgm").str(), -1)); // -1 is depth image flag
    
    
    }


    



    return 0;
}