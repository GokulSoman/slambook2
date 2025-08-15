#include <iostream>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>
using namespace Eigen;

int main(int argc, char** argv){
    
    Quaterniond q1(  0.35, 0.2, 0.3, 0.1);

    Quaterniond q1_a(  0.35, 0.2, 0.3, 0.1);
    q1_a.normalize();

    Vector3d t1 (0.3, 0.1, 0.1);

    Quaterniond q2( -0.5, 0.4, -0.1, 0.2);

    Quaterniond q2_a( -0.5, 0.4, -0.1, 0.2);
    q2_a.normalize();

    Vector3d t2(-0.1, 0.5, 0.3);

    Vector3d p1(0.5, 0.0, 0.2);

    // Quaterniond q_p1( 0, 0.5, 0, 0.2);

    // rotate p1 to world cordinates

    cout << "Original Vector: " << p1.transpose() << endl;

    // normalize quaternions
    q1.normalize();
    q2.normalize();

    Vector3d p1_w = q1.inverse() * p1;

    Isometry3d T1_a = Isometry3d(q1_a);


    cout << "Rotated vector to world: " << p1_w.transpose() << endl;

    cout << "Actual after rotation : " << (T1_a.inverse() * p1).transpose() << endl;

    p1_w = p1_w - (q1.inverse() * t1);

    cout << "Translated vector to world: " << p1_w.transpose() << endl;

    T1_a.pretranslate(t1);
    Vector3d p1_a = (T1_a.inverse() * p1).transpose() ;
    cout << "Actual after rotation and translation : " << p1_a.transpose() << endl;


    // moving to robot 2 frame of reference
    
    Vector3d p2 = q2 * p1_w;
    cout << "Rotated vector to p2: " << p2.transpose() << endl;
    Isometry3d T2_a = Isometry3d(q2);
    Vector3d p2_a = T2_a * p1_a;
    cout << "Actual after rotation: " << p2_a.transpose() << endl;

    p2 = p2 + t2;
    cout << "Translated vector to p2: " << p2.transpose() << endl;

    T2_a.pretranslate(t2);
    cout << "Actual after translation: " << (T2_a * p1_a).transpose() << endl;

    
    return 0;
}