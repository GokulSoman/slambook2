#include <iostream>
#include <fstream>
#include <pangolin/pangolin.h>
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


void showPointCloud(const vector<Vector6d, 
    Eigen::aligned_allocator<Vector6d>> &pointcloud);

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
    
        double data[7] = {0};
        
        cout << "Values in pose file :\n\tx\t\ty\t\tz\t\tq_x\t\tq_y\t\tq_z\t\tq_w" << endl;
        for (auto &d: data){
            fin >> d;
            cout << d << "\t" ;
        }

        Sophus::SE3d pose(Eigen::Quaterniond(data[6], data[3], data[4], data[5]),
                        Eigen::Vector3d(data[0], data[1], data[2]));
        poses.push_back(pose);
        cout << endl;
    }

    double cx = 325.5;
    double cy = 253.5;
    double fx = 518.0;
    double fy = 519.0;
    double depthScale = 1000.0;

    vector<Vector6d, Eigen::aligned_allocator<Vector6d>> pointcloud;
    pointcloud.reserve(1000000); // why?

    for (int i = 0; i < 5; i++) {
        cout << "Converting the RGBD images" << i + 1 << endl;

        cv::Mat color = color_imgs[i];
        cv::Mat depth = depth_imgs[i];
        Sophus::SE3d T = poses[i];

        if (depth.type() != CV_16U){
            cerr << "Depth image is not in correct format (16-bits)" << endl;
            return 1;
        }

        for (int v = 0; v < color.rows; v++){
            for ( int u = 0; u < color.cols; u++ ){
                unsigned int d = depth.ptr<unsigned short>(v)[u];

                if (d == 0) continue; // no valid value

                Eigen::Vector3d point;
                point[2] = (double)d / depthScale;
                point[0] = ( ( u - cx ) * point[2] ) / fx;
                point[1] = ( ( v - cy ) * point[2] ) / fy;

                // Tranform point to worldframe

                Eigen::Vector3d pointWorld = T * point;

                Vector6d p;
                p.head<3>() = pointWorld;

                // then the RGB points

                p[5] = color.ptr<uchar>(v)[u * color.channels()]; // B
                p[4] = color.ptr<uchar>(v)[u * color.channels() + 1]; // G
                p[3] = color.ptr<uchar>(v)[u * color.channels() + 2];

                // p[5] = color.data[v * color.step + u * color.channels()];   // blue
                // p[4] = color.data[v * color.step + u * color.channels() + 1]; // green
                // p[3] = color.data[v * color.step + u * color.channels() + 2]; // red

                pointcloud.push_back(p);
            }
        }
    }

    showPointCloud(pointcloud);
    



    return 0;
}

void showPointCloud(const vector<Vector6d, Eigen::aligned_allocator<Vector6d>> &pointcloud) {

    if (pointcloud.empty()){
        cerr << "Empty pointcloud " << endl;
        return;
    }

    cout << "Creating Pointcloud" << endl;

    pangolin::CreateWindowAndBind("Pointcloud", 1024, 768);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
        .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false) {

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glPointSize(2);
        glBegin(GL_POINTS);

        for (auto &p:pointcloud){
            glColor3d(p[3]/255.0, p[4] / 255.0, p[5]/255.0);
            glVertex3d(p[0], p[1], p[2]);
        }
        glEnd();
        pangolin::FinishFrame();
        usleep(5000);
    }

    return;
}