#include <iostream>
#include <string>

#include <opencv2/opencv.hpp>

using namespace std;

string img_file = "../distorted.png";

int main(int argc, char **argv){

    // Give custom values for distortion
    // k1, k2 for radial dist., p1, p2 for tanfgential dist.
    double k1= -0.28340811, k2=0.07395907, p1 = 0.00019359, p2 = 1.76187114e-05;

    // Manually feed instrinsic matrix vals

    double fx = 458.654, fy = 457.296, cx = 367.215, cy = 248.375;

    // read image

    cv::Mat image = cv::imread(img_file, 0); // 0 -> read as grayscale

    int rows = image.rows, cols = image.cols;

    cv::Mat undistorted_img  = cv::Mat(rows, cols, CV_8UC1);

    // Perform undistortion

    for (int v = 0; v < rows; v++){
        for( int u = 0; u < cols; u ++){

            // find normalized coordinates x,y 
            // obtained by reversing intrinsic transform
            double x = (u - cx) / fx, y = (v - cy) / fy;

            // find x_distorted and y_distorted
            // find r
            double r = sqrt(x*x + y*y);
            double x_distorted = x * (1 + k1 * r*r + k2*r*r*r*r) + 2 * p1 * x * y + p2 * (r*r + 2 *x *x);
            double y_distorted = y * (1 + k1 * r*r + k2 * r * r * r * r) + p1 * (r * r + 2 * y * y )
                                        + p2 * 2 * x * y;

            // perform intrinsic transformation to get new u,v 

            double u_distorted = x_distorted * fx + cx;
            double v_distorted = y_distorted * fy + cy;

            if (u_distorted >= 0 &&
                v_distorted >= 0 &&
                u_distorted < cols &&
                v_distorted < rows){
                
                undistorted_img.at<uchar>(v,u) = image.at<uchar>((int)v_distorted, (int)u_distorted);
            }
            else {
                undistorted_img.at<uchar>(v,u) = 0;
            }

        }
    }

    cv::imshow("Original Image", image);
    cv::imshow("Undistorted Image", undistorted_img);
    cv::waitKey(0);

    return 0;

}
