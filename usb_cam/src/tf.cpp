#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;


int main(int argc, char *argv[])
{
    Eigen::Matrix3d rotation_matrix2;
    rotation_matrix2 << 0.016324, -0.999830, -0.008525, 
                        0.999729, 0.016463, -0.016482,
                        0.016619, -0.008254, 0.999828;

    double x = -0.038596;
    double y = 0.114021;
    double z = 0.019666;

    //ZYX顺序，即先绕x轴roll,再绕y轴pitch,最后绕z轴yaw,0表示X轴,1表示Y轴,2表示Z轴
    Eigen::Vector3d euler_angles = rotation_matrix2.eulerAngles(2, 1, 0); 
    cout << "yaw(z) pitch(y) roll(x) = " << euler_angles.transpose() << endl;

}