#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <string>
#include <vector>
#include <pangolin/pangolin.h>

using namespace std;
using namespace Eigen;

string left_image = "../left.png";
string right_image = "../right.png";

void showPointCloud(
    const vector<Vector4d, Eigen::aligned_allocator<Vector4d>> &pointcloud
);

int main(int argc, char **argv){

    //intrinsics
    double fx = 718.856, fy = 718.856, cx = 607.1928, cy = 185.2157;

    // baseline
    double b = 0.573;

    cv::Mat left = cv::imread(left_image, 0);
    cv::Mat right = cv::imread(right_image, 0);

    cv::imshow("Left Image", left);
    cv::imshow("Right Image", right);
    cv::waitKey(0);
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(
        0, 96, 9, 8*9*9, 32 * 9 * 9, 1, 63, 10, 100, 32
    );

    cv::Mat disparity_sgbm, disparity;

    sgbm -> compute(left, right, disparity_sgbm);

    disparity_sgbm.convertTo(disparity, CV_32F, 1.0 / 16.0f);

    // compute the point cloud

    vector<Vector4d, Eigen::aligned_allocator<Vector4d>> pointcloud;

    for (int v=0; v < left.rows; v++ ){
        for (int u =0; u < left.cols; u++){
            if (disparity.at<float>(v,u) <= 10.0 || disparity.at<float>(v,u) >= 96.0){
                continue; //skip
            }

            Vector4d point(0,0,0,left.at<uchar>(v,u) / 255.0); // 255.0 makes it a float?

            // compute depth from disparity

            double x = (u - cx) / fx;
            double y = (v - cy ) / fy;
            double depth = fx * b / (disparity.at<float>(v,u));

            point[0] = x * depth;
            point[1] = y * depth;
            point[3] = depth;

            pointcloud.push_back(point);
        }
    }

    cv::imshow("Disparity", disparity/ 96.0);
    cv::waitKey(0);


    return 0;




}

void showPointCloud(const vector<Vector4d, Eigen::aligned_allocator<Vector4d>> &pointcloud){

    if (pointcloud.empty())
}